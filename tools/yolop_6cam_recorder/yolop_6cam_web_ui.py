#!/usr/bin/env python3
"""Browser UI for live preview, recording, and playback of six OAK-D PoE cameras."""

from __future__ import annotations

import argparse
import base64
import errno
from fractions import Fraction
import json
import queue
import re
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, BinaryIO, Dict, List, Mapping, Optional, Sequence, TextIO, Tuple
from urllib.parse import parse_qs, unquote, urlparse

import depthai as dai

from yolop_6cam_recorder import (
    CAMERAS,
    CAMERA_STALL_TIMEOUT_SEC,
    DCU_IP,
    INPUT_HEIGHT,
    INPUT_WIDTH,
    MIN_FREE_BYTES_AT_START,
    MIN_FREE_BYTES_DURING_RECORDING,
    RecorderError,
    create_manip,
    device_time_seconds,
    discover_cameras,
    iso_utc_now,
    print_discovery,
)


SESSION_PATTERN = re.compile(r"^yolop_\d{8}_\d{6}(?:_\d{2})?$")
CAMERA_FILE_PATTERN = re.compile(r"^cam_0[1-6]\.mp4$")
STATUS_INTERVAL_SEC = 5.0
MAX_WEB_CLIENTS = 3
WEB_SESSION_TTL_SEC = 30.0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Six-camera OAK-D PoE live Web UI and recorder")
    parser.add_argument("--host", default="127.0.0.1", help="Web UI bind address")
    parser.add_argument(
        "--local-host", default="127.0.0.1",
        help="additional localhost bind address for highest-priority control",
    )
    parser.add_argument("--port", type=int, default=8080, help="Web UI TCP port")
    parser.add_argument("--output", default="recordings", help="recording parent directory")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--bitrate-kbps", type=int, default=2000, help="H.265 bitrate per camera")
    parser.add_argument("--preview-quality", type=int, default=75, help="MJPEG quality 1..100")
    parser.add_argument("--auth-user", help="HTTP Basic authentication user")
    parser.add_argument(
        "--auth-password-file",
        help="file containing the HTTP Basic authentication password",
    )
    return parser


def validate_args(args: argparse.Namespace) -> Path:
    if not (0 < args.fps <= 60):
        raise RecorderError("--fps must be greater than 0 and no more than 60")
    if args.bitrate_kbps <= 0:
        raise RecorderError("--bitrate-kbps must be greater than 0")
    if not (1 <= args.preview_quality <= 100):
        raise RecorderError("--preview-quality must be between 1 and 100")
    if not (1 <= args.port <= 65535):
        raise RecorderError("--port must be between 1 and 65535")
    auth_values = (args.auth_user, args.auth_password_file)
    if any(auth_values) and not all(auth_values):
        raise RecorderError("--auth-user and --auth-password-file must be used together")
    if args.host not in ("127.0.0.1", "localhost", "::1") and not all(auth_values):
        raise RecorderError(
            "A non-loopback bind requires --auth-user and --auth-password-file"
        )
    output = Path(args.output).expanduser().resolve()
    try:
        output.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise RecorderError(f"Cannot create output directory '{output}': {exc}") from exc
    return output


def build_ui_pipeline(fps: float, bitrate_kbps: int, preview_quality: int) -> dai.Pipeline:
    try:
        pipeline = dai.Pipeline()
        camera = pipeline.create(dai.node.ColorCamera)
        camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camera.setFps(fps)
        camera.setInterleaved(False)
        camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        manip = create_manip(pipeline, dai.RawImgFrame.Type.NV12)
        camera.video.link(manip.inputImage)

        h265 = pipeline.create(dai.node.VideoEncoder)
        h265.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
        h265.setBitrateKbps(bitrate_kbps)
        h265.setKeyframeFrequency(max(1, int(round(fps))))
        manip.out.link(h265.input)

        h265_out = pipeline.create(dai.node.XLinkOut)
        h265_out.setStreamName("h265")
        h265.bitstream.link(h265_out.input)

        preview = pipeline.create(dai.node.VideoEncoder)
        preview.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.MJPEG)
        preview.setQuality(preview_quality)
        manip.out.link(preview.input)

        preview_out = pipeline.create(dai.node.XLinkOut)
        preview_out.setStreamName("preview")
        preview.bitstream.link(preview_out.input)
        return pipeline
    except Exception as exc:
        raise RecorderError(f"Web UI pipeline creation failed: {exc}") from exc


class PreviewStore:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frames: Dict[str, bytes] = {}
        self._updated_at: Dict[str, float] = {}

    def update(self, camera_name: str, jpeg: bytes) -> None:
        with self._condition:
            self._frames[camera_name] = jpeg
            self._updated_at[camera_name] = time.monotonic()
            self._condition.notify_all()

    def get(self, camera_name: str, timeout: float = 1.0) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while camera_name not in self._frames:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return self._frames[camera_name]

    def ages(self) -> Dict[str, Optional[float]]:
        now = time.monotonic()
        with self._condition:
            return {
                camera: round(now - self._updated_at[camera], 3) if camera in self._updated_at else None
                for camera in CAMERAS
            }


@dataclass
class SessionFiles:
    video: BinaryIO
    jsonl: TextIO
    ready_for_keyframe: bool = False
    saved_frames: int = 0


def h265_has_random_access_unit(data: bytes) -> bool:
    """Return True when an Annex-B packet contains VPS or an IRAP NAL unit."""
    index = 0
    length = len(data)
    while index + 5 < length:
        if data[index:index + 3] == b"\x00\x00\x01":
            nal_index = index + 3
            index = nal_index + 1
        elif data[index:index + 4] == b"\x00\x00\x00\x01":
            nal_index = index + 4
            index = nal_index + 1
        else:
            index += 1
            continue
        nal_type = (data[nal_index] >> 1) & 0x3F
        if nal_type in (19, 20, 21, 32):
            return True
    return False


class RecordingManager:
    def __init__(self, output_root: Path, fps: float, bitrate_kbps: int) -> None:
        self.output_root = output_root
        self.fps = fps
        self.bitrate_kbps = bitrate_kbps
        self._lock = threading.RLock()
        self._active = False
        self._session_dir: Optional[Path] = None
        self._started_at: Optional[str] = None
        self._files: Dict[str, SessionFiles] = {}
        self._next_space_check = 0.0
        self._last_error: Optional[str] = None

    def _new_session_dir(self) -> Path:
        free = shutil.disk_usage(self.output_root).free
        if free < MIN_FREE_BYTES_AT_START:
            raise RecorderError(
                f"Insufficient storage: {free / (1024 ** 3):.2f} GiB free; at least 1 GiB required"
            )
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session = self.output_root / f"yolop_{stamp}"
        suffix = 1
        while session.exists():
            session = self.output_root / f"yolop_{stamp}_{suffix:02d}"
            suffix += 1
        session.mkdir()
        return session

    def _metadata(self, ended_at: Optional[str] = None) -> Dict[str, Any]:
        return {
            "created_at": self._started_at,
            "ended_at": ended_at,
            "dcu_ip": DCU_IP,
            "python_version": sys.version.split()[0],
            "depthai_version": getattr(dai, "__version__", "unknown"),
            "mode": "web_ui_record_only",
            "input_size": [INPUT_WIDTH, INPUT_HEIGHT],
            "resize_mode": "centered_letterbox_setResizeThumbnail",
            "fps": self.fps,
            "bitrate_kbps_per_camera": self.bitrate_kbps,
            "cameras": [
                {
                    "camera_name": camera,
                    "mx_id": mxid,
                    "saved_frames": self._files[camera].saved_frames if camera in self._files else 0,
                }
                for camera, mxid in CAMERAS.items()
            ],
        }

    def _write_metadata(self, ended_at: Optional[str] = None) -> None:
        if self._session_dir is None:
            return
        (self._session_dir / "session_metadata.json").write_text(
            json.dumps(self._metadata(ended_at), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    def start(self) -> Dict[str, Any]:
        with self._lock:
            if self._active:
                raise RecorderError("Recording is already active")
            session = self._new_session_dir()
            opened: Dict[str, SessionFiles] = {}
            try:
                for camera in CAMERAS:
                    opened[camera] = SessionFiles(
                        video=(session / f"{camera}.h265").open("wb", buffering=1024 * 1024),
                        jsonl=(session / f"{camera}.jsonl").open("w", encoding="utf-8", buffering=1),
                    )
                self._files = opened
                self._session_dir = session
                self._started_at = iso_utc_now()
                self._last_error = None
                self._next_space_check = time.monotonic() + STATUS_INTERVAL_SEC
                self._write_metadata()
                self._active = True
                return self.status()
            except BaseException:
                for entry in opened.values():
                    entry.video.close()
                    entry.jsonl.close()
                raise

    def write_packet(self, camera_name: str, packet: Any) -> None:
        with self._lock:
            if not self._active:
                return
            entry = self._files[camera_name]
            data = packet.getData().tobytes()
            if not entry.ready_for_keyframe:
                if not h265_has_random_access_unit(data):
                    return
                entry.ready_for_keyframe = True

            now = time.monotonic()
            if now >= self._next_space_check:
                free = shutil.disk_usage(self.output_root).free
                if free < MIN_FREE_BYTES_DURING_RECORDING:
                    raise RecorderError(
                        f"Storage space low: {free / (1024 ** 2):.1f} MiB free "
                        f"(< {MIN_FREE_BYTES_DURING_RECORDING / (1024 ** 2):.0f} MiB)"
                    )
                self._next_space_check = now + STATUS_INTERVAL_SEC

            try:
                entry.video.write(data)
                record = {
                    "host_time": iso_utc_now(),
                    "device_time_seconds": device_time_seconds(packet),
                    "camera_name": camera_name,
                    "mx_id": CAMERAS[camera_name],
                    "frame_number": int(packet.getSequenceNum()),
                    "objects": [],
                    "drivable_area": None,
                    "lane_area": None,
                    "mode": "record_only",
                }
                entry.jsonl.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
                entry.saved_frames += 1
            except OSError as exc:
                if exc.errno == errno.ENOSPC:
                    raise RecorderError(f"Storage space exhausted: {exc}") from exc
                raise RecorderError(f"Recording write failed for {camera_name}: {exc}") from exc

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if not self._active:
                raise RecorderError("Recording is not active")
            self._active = False
            ended_at = iso_utc_now()
            errors: List[str] = []
            for camera, entry in self._files.items():
                try:
                    entry.video.flush()
                    entry.video.close()
                    entry.jsonl.close()
                except OSError as exc:
                    errors.append(f"{camera}: {exc}")
            try:
                self._write_metadata(ended_at)
            except OSError as exc:
                errors.append(f"metadata: {exc}")
            result = self.status()
            if errors:
                self._last_error = "; ".join(errors)
                raise RecorderError(f"Recording stopped with file errors: {self._last_error}")
            return result

    def close(self) -> None:
        with self._lock:
            if self._active:
                try:
                    self.stop()
                except RecorderError as exc:
                    self._last_error = str(exc)

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "active": self._active,
                "session": self._session_dir.name if self._session_dir else None,
                "session_path": str(self._session_dir) if self._session_dir else None,
                "started_at": self._started_at,
                "saved_frames": {
                    camera: entry.saved_frames for camera, entry in self._files.items()
                },
                "waiting_for_keyframe": [
                    camera for camera, entry in self._files.items()
                    if self._active and not entry.ready_for_keyframe
                ],
                "last_error": self._last_error,
            }


class PlaybackManager:
    def __init__(self, output_root: Path, fps: float) -> None:
        self.output_root = output_root
        self.fps = fps
        self.ffmpeg = shutil.which("ffmpeg")
        self.gstreamer = shutil.which("gst-launch-1.0")
        self.backend = "ffmpeg" if self.ffmpeg else ("gstreamer" if self.gstreamer else None)
        self._lock = threading.Lock()
        self._state: Dict[str, Dict[str, Any]] = {}

    def _session_dir(self, session_name: str) -> Path:
        if not SESSION_PATTERN.fullmatch(session_name):
            raise RecorderError("Invalid recording directory name")
        path = (self.output_root / session_name).resolve()
        if path.parent != self.output_root or not path.is_dir():
            raise RecorderError(f"Recording directory does not exist: {session_name}")
        return path

    def list_sessions(self) -> List[Dict[str, Any]]:
        sessions: List[Dict[str, Any]] = []
        for path in sorted(self.output_root.iterdir(), reverse=True):
            if not path.is_dir() or not SESSION_PATTERN.fullmatch(path.name):
                continue
            videos = sum(1 for camera in CAMERAS if (path / f"{camera}.h265").is_file())
            ready = all((path / ".playback" / f"{camera}.mp4").is_file() for camera in CAMERAS)
            sessions.append({"name": path.name, "camera_files": videos, "playback_ready": ready})
        return sessions

    def prepare(self, session_name: str) -> Dict[str, Any]:
        session = self._session_dir(session_name)
        if self.backend is None:
            raise RecorderError("ffmpeg/GStreamer is not installed; playback preparation is unavailable")
        with self._lock:
            current = self._state.get(session_name)
            if current and current["state"] == "converting":
                return dict(current)
            playback_dir = session / ".playback"
            if all((playback_dir / f"{camera}.mp4").is_file() for camera in CAMERAS):
                self._state[session_name] = {"state": "ready", "completed": 6, "total": 6, "error": None}
                return dict(self._state[session_name])
            self._state[session_name] = {"state": "converting", "completed": 0, "total": 6, "error": None}
            thread = threading.Thread(
                target=self._convert_session,
                args=(session,),
                name=f"playback-{session_name}",
                daemon=True,
            )
            thread.start()
            return dict(self._state[session_name])

    def _convert_session(self, session: Path) -> None:
        playback_dir = session / ".playback"
        playback_dir.mkdir(exist_ok=True)
        try:
            for camera in CAMERAS:
                source = session / f"{camera}.h265"
                target = playback_dir / f"{camera}.mp4"
                if not source.is_file():
                    raise RecorderError(f"Missing recording: {source.name}")
                if not target.is_file() or target.stat().st_mtime < source.stat().st_mtime:
                    if self.backend == "ffmpeg":
                        command = [
                            str(self.ffmpeg), "-hide_banner", "-loglevel", "error", "-y",
                            "-framerate", f"{self.fps:g}", "-i", str(source),
                            "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(target),
                        ]
                    else:
                        rate = Fraction(self.fps).limit_denominator(1000)
                        command = [
                            str(self.gstreamer), "-q",
                            "filesrc", f"location={source}", "!", "h265parse", "!", "avdec_h265", "!",
                            "videorate", "!", f"video/x-raw,framerate={rate.numerator}/{rate.denominator}", "!",
                            "videoconvert", "!", "x264enc", "speed-preset=ultrafast", "!", "h264parse", "!",
                            "mp4mux", "faststart=true", "!", "filesink", f"location={target}",
                        ]
                    completed = subprocess.run(command, capture_output=True, text=True, timeout=3600)
                    if completed.returncode != 0:
                        raise RecorderError(
                            f"{self.backend} failed for {camera}: "
                            f"{completed.stderr.strip() or completed.returncode}"
                        )
                with self._lock:
                    self._state[session.name]["completed"] += 1
            with self._lock:
                self._state[session.name]["state"] = "ready"
        except BaseException as exc:
            with self._lock:
                self._state[session.name] = {
                    "state": "error", "completed": 0, "total": 6, "error": str(exc)
                }

    def status(self, session_name: Optional[str]) -> Dict[str, Any]:
        if not session_name:
            return {"state": "idle", "completed": 0, "total": 6, "error": None}
        with self._lock:
            return dict(self._state.get(
                session_name, {"state": "idle", "completed": 0, "total": 6, "error": None}
            ))

    def video_path(self, session_name: str, filename: str) -> Path:
        session = self._session_dir(session_name)
        if not CAMERA_FILE_PATTERN.fullmatch(filename):
            raise RecorderError("Invalid playback filename")
        path = (session / ".playback" / filename).resolve()
        expected_parent = (session / ".playback").resolve()
        if path.parent != expected_parent or not path.is_file():
            raise RecorderError("Playback file is not ready")
        return path


class CameraWorker(threading.Thread):
    def __init__(
        self,
        camera_name: str,
        h265_queue: Any,
        preview_queue: Any,
        previews: PreviewStore,
        recorder: RecordingManager,
        stop_event: threading.Event,
        errors: "queue.Queue[Tuple[str, BaseException]]",
    ) -> None:
        super().__init__(name=f"ui-worker-{camera_name}", daemon=False)
        self.camera_name = camera_name
        self.h265_queue = h265_queue
        self.preview_queue = preview_queue
        self.previews = previews
        self.recorder = recorder
        self.stop_event = stop_event
        self.errors = errors
        self.last_packet_monotonic = time.monotonic()
        self.h265_frames = 0
        self.preview_frames = 0
        self._lock = threading.Lock()

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "h265_frames": self.h265_frames,
                "preview_frames": self.preview_frames,
                "last_packet_monotonic": self.last_packet_monotonic,
            }

    def run(self) -> None:
        try:
            while not self.stop_event.is_set():
                did_work = False
                while True:
                    packet = self.h265_queue.tryGet()
                    if packet is None:
                        break
                    self.recorder.write_packet(self.camera_name, packet)
                    with self._lock:
                        self.h265_frames += 1
                        self.last_packet_monotonic = time.monotonic()
                    did_work = True
                while True:
                    packet = self.preview_queue.tryGet()
                    if packet is None:
                        break
                    self.previews.update(self.camera_name, packet.getData().tobytes())
                    with self._lock:
                        self.preview_frames += 1
                        self.last_packet_monotonic = time.monotonic()
                    did_work = True
                if not did_work:
                    time.sleep(0.002)
        except BaseException as exc:
            self.errors.put((self.camera_name, exc))
            self.stop_event.set()


class WebAccessError(RecorderError):
    def __init__(self, message: str, status: HTTPStatus) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class WebClientSession:
    token: str
    address: str
    is_admin: bool
    last_seen: float

    @property
    def label(self) -> str:
        if self.is_admin:
            return "DCU localhost 관리자"
        return f"외부 접속자 {self.token[:6]}"


class AccessManager:
    """Track browser sessions and serialize recording control ownership."""

    def __init__(self, max_clients: int = MAX_WEB_CLIENTS, ttl: float = WEB_SESSION_TTL_SEC) -> None:
        self.max_clients = max_clients
        self.ttl = ttl
        self._lock = threading.RLock()
        self._sessions: Dict[str, WebClientSession] = {}
        self._recording_controller: Optional[str] = None

    @staticmethod
    def _is_admin_address(address: str) -> bool:
        return address in ("127.0.0.1", "::1")

    def _prune(self, now: float) -> None:
        expired = [
            token for token, session in self._sessions.items()
            if now - session.last_seen > self.ttl
        ]
        for token in expired:
            del self._sessions[token]

    def touch(self, token: Optional[str], address: str) -> Tuple[WebClientSession, bool]:
        now = time.monotonic()
        is_admin = self._is_admin_address(address)
        with self._lock:
            self._prune(now)
            if token and token in self._sessions:
                session = self._sessions[token]
                session.last_seen = now
                if is_admin:
                    session.is_admin = True
                    session.address = address
                return session, False

            if len(self._sessions) >= self.max_clients:
                if is_admin:
                    remote_sessions = [s for s in self._sessions.values() if not s.is_admin]
                    if remote_sessions:
                        oldest = min(remote_sessions, key=lambda item: item.last_seen)
                        del self._sessions[oldest.token]
                    else:
                        raise WebAccessError(
                            f"동시 접속은 최대 {self.max_clients}개입니다.",
                            HTTPStatus.SERVICE_UNAVAILABLE,
                        )
                else:
                    raise WebAccessError(
                        f"동시 접속은 최대 {self.max_clients}개입니다. 잠시 후 다시 접속하세요.",
                        HTTPStatus.SERVICE_UNAVAILABLE,
                    )

            session = WebClientSession(
                token=secrets.token_urlsafe(24),
                address=address,
                is_admin=is_admin,
                last_seen=now,
            )
            self._sessions[session.token] = session
            return session, True

    def start_recording(self, session: WebClientSession, recorder: RecordingManager) -> Dict[str, Any]:
        with self._lock:
            self._prune(time.monotonic())
            controller = self._sessions.get(self._recording_controller or "")
            if controller is not None and controller.token != session.token and not session.is_admin:
                raise WebAccessError(
                    f"{controller.label}가 녹화를 제어하고 있습니다.", HTTPStatus.LOCKED
                )
            result = recorder.start()
            self._recording_controller = session.token
            return result

    def stop_recording(self, session: WebClientSession, recorder: RecordingManager) -> Dict[str, Any]:
        with self._lock:
            self._prune(time.monotonic())
            controller = self._sessions.get(self._recording_controller or "")
            if controller is not None and controller.token != session.token and not session.is_admin:
                raise WebAccessError(
                    f"{controller.label}만 녹화를 정지할 수 있습니다.", HTTPStatus.LOCKED
                )
            result = recorder.stop()
            self._recording_controller = None
            return result

    def status(self, session: WebClientSession, recording_active: bool) -> Dict[str, Any]:
        with self._lock:
            self._prune(time.monotonic())
            controller = self._sessions.get(self._recording_controller or "")
            owns_control = controller is not None and controller.token == session.token
            can_stop = recording_active and (session.is_admin or owns_control or controller is None)
            return {
                "role": "admin" if session.is_admin else "operator",
                "role_label": "localhost 관리자" if session.is_admin else "외부 사용자",
                "connected_clients": len(self._sessions),
                "max_clients": self.max_clients,
                "session_timeout_seconds": self.ttl,
                "recording_controller": controller.label if controller else None,
                "owns_recording_control": owns_control,
                "can_start_recording": not recording_active,
                "can_stop_recording": can_stop,
            }


@dataclass
class WebContext:
    html: bytes
    cameras: Sequence[str]
    previews: PreviewStore
    recorder: RecordingManager
    playback: PlaybackManager
    workers: Mapping[str, CameraWorker]
    started_monotonic: float
    auth_header: Optional[str]
    access: AccessManager


def json_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def make_handler(context: WebContext) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "OAK6WebUI/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[web] {self.address_string()} {fmt % args}")

        def _send_bytes(self, status: int, content_type: str, payload: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            session_cookie = getattr(self, "_new_session_cookie", None)
            if session_cookie:
                self.send_header(
                    "Set-Cookie",
                    f"oak_session={session_cookie}; Path=/; HttpOnly; SameSite=Strict",
                )
                self._new_session_cookie = None
            self.end_headers()
            self.wfile.write(payload)

        def _authorized(self) -> bool:
            expected = context.auth_header
            if expected is None:
                return True
            supplied = self.headers.get("Authorization", "")
            return secrets.compare_digest(supplied, expected)

        def _require_auth(self) -> bool:
            if self._authorized():
                return True
            payload = json_bytes({"ok": False, "error": "Authentication required"})
            self.send_response(HTTPStatus.UNAUTHORIZED)
            self.send_header("WWW-Authenticate", 'Basic realm="OAK-D 6 Camera", charset="UTF-8"')
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
            return False

        def _session_token(self) -> Optional[str]:
            cookie = SimpleCookie()
            try:
                cookie.load(self.headers.get("Cookie", ""))
            except Exception:
                return None
            morsel = cookie.get("oak_session")
            return morsel.value if morsel else None

        def _require_access(self) -> bool:
            try:
                session, created = context.access.touch(
                    self._session_token(), self.client_address[0]
                )
            except WebAccessError as exc:
                self._error(exc.status, str(exc))
                return False
            self.web_session = session
            self._new_session_cookie = session.token if created else None
            return True

        def _send_json(self, status: int, value: Any) -> None:
            self._send_bytes(status, "application/json; charset=utf-8", json_bytes(value))

        def _error(self, status: int, message: str) -> None:
            self._send_json(status, {"ok": False, "error": message})

        def _read_json(self) -> Dict[str, Any]:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise RecorderError("Invalid Content-Length") from exc
            if length > 1024 * 1024:
                raise RecorderError("Request body is too large")
            if length == 0:
                return {}
            try:
                value = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise RecorderError("Invalid JSON request") from exc
            if not isinstance(value, dict):
                raise RecorderError("JSON request must be an object")
            return value

        def _status_payload(self, session: Optional[str] = None) -> Dict[str, Any]:
            now = time.monotonic()
            camera_status = {}
            for camera, worker in context.workers.items():
                snapshot = worker.snapshot()
                camera_status[camera] = {
                    "preview_fps": round(snapshot["preview_frames"] / max(now - context.started_monotonic, 1e-9), 2),
                    "encoder_fps": round(snapshot["h265_frames"] / max(now - context.started_monotonic, 1e-9), 2),
                    "packet_age_seconds": round(now - snapshot["last_packet_monotonic"], 3),
                }
            recording = context.recorder.status()
            return {
                "ok": True,
                "recording": recording,
                "access": context.access.status(self.web_session, recording["active"]),
                "playback": context.playback.status(session),
                "camera_status": camera_status,
                "preview_age_seconds": context.previews.ages(),
                "playback_backend": context.playback.backend,
            }

        def do_GET(self) -> None:  # noqa: N802
            if not self._require_auth():
                return
            if not self._require_access():
                return
            parsed = urlparse(self.path)
            try:
                if parsed.path == "/":
                    self._send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", context.html)
                    return
                if parsed.path.startswith("/snapshot/"):
                    camera = unquote(parsed.path.removeprefix("/snapshot/"))
                    if camera not in context.cameras:
                        self._error(HTTPStatus.NOT_FOUND, "Unknown camera")
                        return
                    frame = context.previews.get(camera)
                    if frame is None:
                        self._error(HTTPStatus.SERVICE_UNAVAILABLE, "Preview frame is not ready")
                        return
                    self._send_bytes(HTTPStatus.OK, "image/jpeg", frame)
                    return
                if parsed.path == "/api/status":
                    session = parse_qs(parsed.query).get("session", [None])[0]
                    self._send_json(HTTPStatus.OK, self._status_payload(session))
                    return
                if parsed.path == "/api/sessions":
                    self._send_json(HTTPStatus.OK, {"ok": True, "sessions": context.playback.list_sessions()})
                    return
                if parsed.path.startswith("/playback/"):
                    parts = [unquote(part) for part in parsed.path.split("/") if part]
                    if len(parts) != 3:
                        raise RecorderError("Invalid playback URL")
                    self._send_video(context.playback.video_path(parts[1], parts[2]))
                    return
                self._error(HTTPStatus.NOT_FOUND, "Not found")
            except RecorderError as exc:
                self._error(HTTPStatus.BAD_REQUEST, str(exc))
            except BrokenPipeError:
                pass
            except BaseException as exc:
                self._error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))

        def do_POST(self) -> None:  # noqa: N802
            if not self._require_auth():
                return
            if not self._require_access():
                return
            parsed = urlparse(self.path)
            try:
                body = self._read_json()
                if parsed.path == "/api/recording/start":
                    recording = context.access.start_recording(self.web_session, context.recorder)
                    self._send_json(HTTPStatus.OK, {"ok": True, "recording": recording})
                    return
                if parsed.path == "/api/recording/stop":
                    recording = context.access.stop_recording(self.web_session, context.recorder)
                    self._send_json(HTTPStatus.OK, {"ok": True, "recording": recording})
                    return
                if parsed.path == "/api/playback/prepare":
                    session = str(body.get("session", ""))
                    recording = context.recorder.status()
                    if recording["active"] and recording["session"] == session:
                        raise RecorderError("현재 녹화 중인 디렉터리는 녹화 정지 후 재생할 수 있습니다")
                    self._send_json(
                        HTTPStatus.ACCEPTED,
                        {"ok": True, "session": session, "playback": context.playback.prepare(session)},
                    )
                    return
                self._error(HTTPStatus.NOT_FOUND, "Not found")
            except WebAccessError as exc:
                self._error(exc.status, str(exc))
            except RecorderError as exc:
                self._error(HTTPStatus.CONFLICT, str(exc))
            except BrokenPipeError:
                pass
            except BaseException as exc:
                self._error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))

        def _send_video(self, path: Path) -> None:
            size = path.stat().st_size
            start, end = 0, size - 1
            status = HTTPStatus.OK
            range_header = self.headers.get("Range")
            if range_header:
                match = re.fullmatch(r"bytes=(\d*)-(\d*)", range_header.strip())
                if not match:
                    self.send_error(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    return
                if match.group(1):
                    start = int(match.group(1))
                if match.group(2):
                    end = min(int(match.group(2)), size - 1)
                if start > end or start >= size:
                    self.send_error(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    return
                status = HTTPStatus.PARTIAL_CONTENT
            length = end - start + 1
            self.send_response(status)
            self.send_header("Content-Type", "video/mp4")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(length))
            session_cookie = getattr(self, "_new_session_cookie", None)
            if session_cookie:
                self.send_header(
                    "Set-Cookie",
                    f"oak_session={session_cookie}; Path=/; HttpOnly; SameSite=Strict",
                )
                self._new_session_cookie = None
            if status == HTTPStatus.PARTIAL_CONTENT:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            with path.open("rb") as stream:
                stream.seek(start)
                remaining = length
                while remaining > 0:
                    chunk = stream.read(min(1024 * 1024, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)

    return Handler


def load_html() -> bytes:
    path = Path(__file__).resolve().parent / "web_ui" / "index.html"
    try:
        return path.read_bytes()
    except OSError as exc:
        raise RecorderError(f"Cannot load Web UI asset '{path}': {exc}") from exc


def run(args: argparse.Namespace, output_root: Path) -> int:
    auth_header: Optional[str] = None
    if args.auth_password_file:
        password_path = Path(args.auth_password_file).expanduser().resolve()
        try:
            password = password_path.read_text(encoding="utf-8").strip()
        except OSError as exc:
            raise RecorderError(f"Cannot read authentication password file '{password_path}': {exc}") from exc
        if not password:
            raise RecorderError(f"Authentication password file is empty: {password_path}")
        token = base64.b64encode(f"{args.auth_user}:{password}".encode("utf-8")).decode("ascii")
        auth_header = f"Basic {token}"

    mapped, missing = discover_cameras()
    print_discovery(mapped, missing)
    if missing:
        raise RecorderError("Six-camera requirement is not satisfied; Web UI was not started")

    stop_event = threading.Event()
    errors: "queue.Queue[Tuple[str, BaseException]]" = queue.Queue()
    previews = PreviewStore()
    recorder = RecordingManager(output_root, args.fps, args.bitrate_kbps)
    playback = PlaybackManager(output_root, args.fps)
    devices: Dict[str, dai.Device] = {}
    queues: Dict[str, Tuple[Any, Any]] = {}
    workers: Dict[str, CameraWorker] = {}
    http_servers: List[ThreadingHTTPServer] = []
    server_threads: List[threading.Thread] = []
    old_handlers: Dict[int, Any] = {}

    def request_stop(signum: int, _frame: Any) -> None:
        print(f"\nSignal {signum} received; stopping Web UI, recording and cameras...", flush=True)
        stop_event.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, request_stop)

        for camera in CAMERAS:
            pipeline = build_ui_pipeline(args.fps, args.bitrate_kbps, args.preview_quality)
            try:
                device = dai.Device(pipeline, mapped[camera], dai.UsbSpeed.SUPER)
                if device.getMxId() != CAMERAS[camera]:
                    actual = device.getMxId()
                    device.close()
                    raise RecorderError(f"MX ID mismatch for {camera}: {actual}")
                devices[camera] = device
                queues[camera] = (
                    device.getOutputQueue("h265", maxSize=max(60, int(args.fps * 10)), blocking=False),
                    device.getOutputQueue("preview", maxSize=max(10, int(args.fps * 2)), blocking=False),
                )
                print(f"Connected {camera}: {device.getMxId()} ({device.getDeviceName()})")
            except Exception as exc:
                raise RecorderError(f"Failed to start {camera} ({CAMERAS[camera]}): {exc}") from exc

        for camera in CAMERAS:
            h265_queue, preview_queue = queues[camera]
            while h265_queue.tryGet() is not None:
                pass
            while preview_queue.tryGet() is not None:
                pass
            worker = CameraWorker(
                camera, h265_queue, preview_queue, previews, recorder, stop_event, errors
            )
            workers[camera] = worker
            worker.start()

        started = time.monotonic()
        context = WebContext(
            html=load_html(), cameras=tuple(CAMERAS), previews=previews,
            recorder=recorder, playback=playback, workers=workers,
            started_monotonic=started,
            auth_header=auth_header,
            access=AccessManager(),
        )
        bind_hosts = [args.host]
        if (
            args.local_host
            and args.local_host not in bind_hosts
            and args.host not in ("0.0.0.0", "::")
        ):
            bind_hosts.append(args.local_host)
        handler = make_handler(context)
        for bind_host in bind_hosts:
            httpd = ThreadingHTTPServer((bind_host, args.port), handler)
            httpd.daemon_threads = True
            server_thread = threading.Thread(
                target=httpd.serve_forever,
                name=f"web-ui-{bind_host}",
                daemon=True,
            )
            http_servers.append(httpd)
            server_threads.append(server_thread)
            server_thread.start()
            print(f"Web UI listener: http://{bind_host}:{args.port}")
        if args.host in ("127.0.0.1", "localhost"):
            print(f"Web UI: SSH tunnel required, then open http://127.0.0.1:{args.port}")
        else:
            print(f"Web UI: http://{DCU_IP}:{args.port}")
        print("Press Ctrl+C to stop.")

        next_status = started + STATUS_INTERVAL_SEC
        while not stop_event.is_set():
            try:
                camera, exc = errors.get_nowait()
            except queue.Empty:
                pass
            else:
                raise RecorderError(f"Fatal camera worker error [{camera}]: {exc}") from exc
            now = time.monotonic()
            if now >= next_status:
                status_parts = []
                for camera, worker in workers.items():
                    snapshot = worker.snapshot()
                    age = now - snapshot["last_packet_monotonic"]
                    if age > CAMERA_STALL_TIMEOUT_SEC:
                        raise RecorderError(f"Camera stalled [{camera}]: no packet for {age:.1f}s")
                    elapsed = max(now - started, 1e-9)
                    status_parts.append(f"{camera}={snapshot['preview_frames'] / elapsed:.1f}fps")
                print("Preview status: " + ", ".join(status_parts))
                next_status = now + STATUS_INTERVAL_SEC
            time.sleep(0.05)
        return 0
    finally:
        stop_event.set()
        for httpd in http_servers:
            httpd.shutdown()
            httpd.server_close()
        for server_thread in server_threads:
            server_thread.join(timeout=5.0)
        recorder.close()
        for worker in workers.values():
            worker.join(timeout=10.0)
        for camera, device in reversed(list(devices.items())):
            try:
                device.close()
                print(f"Closed {camera}")
            except Exception as exc:
                print(f"WARNING: failed to close {camera}: {exc}", file=sys.stderr)
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        print("Web UI, files and all DepthAI devices are closed.")


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        output_root = validate_args(args)
        return run(args, output_root)
    except RecorderError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
