from __future__ import annotations

import argparse
import base64
import ctypes
from ctypes import wintypes
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import shutil
import socket
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
from PIL import Image, ImageTk


DEFAULT_DEVICE_IP = "192.168.200.48"
DEFAULT_DEVICE_SERIAL = "1142227768"
OAKCTL_PATH = Path(r"C:\Program Files\oakctl\bin\oakctl.exe")
OAK_APP_PATH = Path(__file__).resolve().parents[1] / "apps" / "yolopv2_standalone"
DEFAULT_FPS = 10.0
PREVIEW_SIZE = (720, 405)
CREDENTIAL_FILE = (
    Path(os.environ.get("APPDATA", Path.home() / "AppData" / "Roaming"))
    / "OAK4DPro"
    / "credentials.json"
)


class DataBlob(ctypes.Structure):
    _fields_ = [("size", wintypes.DWORD), ("data", ctypes.POINTER(ctypes.c_byte))]


def _crypt_protect(secret: str) -> str:
    raw = secret.encode("utf-8")
    buffer = ctypes.create_string_buffer(raw)
    source = DataBlob(len(raw), ctypes.cast(buffer, ctypes.POINTER(ctypes.c_byte)))
    protected = DataBlob()
    if not ctypes.windll.crypt32.CryptProtectData(
        ctypes.byref(source),
        "OAK4D Pro recorder",
        None,
        None,
        None,
        0,
        ctypes.byref(protected),
    ):
        raise ctypes.WinError()
    try:
        encrypted = ctypes.string_at(protected.data, protected.size)
        return base64.b64encode(encrypted).decode("ascii")
    finally:
        ctypes.windll.kernel32.LocalFree(protected.data)


def _crypt_unprotect(encoded: str) -> str:
    raw = base64.b64decode(encoded)
    buffer = ctypes.create_string_buffer(raw)
    source = DataBlob(len(raw), ctypes.cast(buffer, ctypes.POINTER(ctypes.c_byte)))
    unprotected = DataBlob()
    if not ctypes.windll.crypt32.CryptUnprotectData(
        ctypes.byref(source), None, None, None, None, 0, ctypes.byref(unprotected)
    ):
        raise ctypes.WinError()
    try:
        secret = ctypes.string_at(unprotected.data, unprotected.size)
        return secret.decode("utf-8")
    finally:
        ctypes.windll.kernel32.LocalFree(unprotected.data)


def save_password(device_ip: str, password: str) -> None:
    credentials: dict[str, str] = {}
    try:
        loaded = json.loads(CREDENTIAL_FILE.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            credentials = loaded
    except (OSError, ValueError, TypeError):
        pass
    credentials[device_ip] = _crypt_protect(password)
    CREDENTIAL_FILE.parent.mkdir(parents=True, exist_ok=True)
    temporary = CREDENTIAL_FILE.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(credentials, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(CREDENTIAL_FILE)


def load_password(device_ip: str) -> str:
    try:
        credentials = json.loads(CREDENTIAL_FILE.read_text(encoding="utf-8"))
        if not isinstance(credentials, dict):
            return ""
        encrypted = credentials.get(device_ip)
        return _crypt_unprotect(encrypted) if encrypted else ""
    except (OSError, ValueError, TypeError):
        return ""


def create_video_writer(path: Path, frame) -> cv2.VideoWriter | None:
    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(
        str(path),
        cv2.CAP_MSMF,
        cv2.VideoWriter_fourcc(*"H264"),
        DEFAULT_FPS,
        (width, height),
    )
    if not writer.isOpened():
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            DEFAULT_FPS,
            (width, height),
        )
    return writer if writer.isOpened() else None


class CaptureSource:
    def __init__(self, key: str, label: str, url: str) -> None:
        self.key = key
        self.label = label
        self.url = url
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest_frame = None
        self.writer: cv2.VideoWriter | None = None
        self.temp_path: Path | None = None
        self.connected = False
        self.started = False

    def start(self) -> None:
        if self.started:
            return
        self.started = True
        threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"oak-capture-{self.key}",
        ).start()

    def _capture_loop(self) -> None:
        while not self.stop_event.is_set():
            capture = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                with self.lock:
                    self.connected = False
                self.stop_event.wait(1.0)
                continue

            with self.lock:
                self.connected = True
            while not self.stop_event.is_set():
                ok, frame = capture.read()
                if not ok:
                    break
                with self.lock:
                    self.latest_frame = frame
                    if self.writer is not None:
                        self.writer.write(frame)

            capture.release()
            with self.lock:
                self.connected = False

    def snapshot(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            return self.connected, frame

    def start_recording(self, path: Path) -> bool:
        connected, frame = self.snapshot()
        if not connected or frame is None:
            return False
        writer = create_video_writer(path, frame)
        if writer is None:
            return False
        with self.lock:
            self.writer = writer
            self.temp_path = path
        return True

    def stop_recording(self) -> Path | None:
        with self.lock:
            writer = self.writer
            path = self.temp_path
            self.writer = None
            self.temp_path = None
        if writer is not None:
            writer.release()
        return path

    def close(self) -> None:
        self.stop_event.set()
        self.stop_recording()


class RecorderApp:
    def __init__(
        self,
        root: tk.Tk,
        device_ip: str,
        device_serial: str,
        rgb_url: str | None,
        result_url: str | None,
        output_dir: Path,
    ) -> None:
        self.root = root
        self.device_ip = device_ip
        self.device_serial = device_serial
        self.rgb_url_override = rgb_url
        self.result_url_override = result_url
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.sources = {
            "rgb": CaptureSource("rgb", "원본 RGB", ""),
            "yolopv2": CaptureSource("yolopv2", "YOLO 결과", ""),
        }
        self.selected = {
            "rgb": tk.BooleanVar(value=True),
            "yolopv2": tk.BooleanVar(value=True),
        }
        self.preview_source = tk.StringVar(value="yolopv2")
        self.unsaved_paths: dict[str, Path] = {}
        self.recording_keys: list[str] = []
        self.recording_stamp = ""
        self.recording = False
        self.connect_requested = False
        self.connection_events: queue.SimpleQueue[tuple[str, str]] = queue.SimpleQueue()
        self.last_connection_state: tuple[bool, bool] | None = None

        self.root.title("OAK4-D Pro 영상 녹화")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self.preview = ttk.Label(
            root,
            anchor="center",
            text="영상 연결 버튼을 눌러 주세요",
        )
        self.preview.grid(row=0, column=0, columnspan=3, padx=12, pady=(12, 8))

        connection = ttk.Frame(root)
        connection.grid(row=1, column=0, columnspan=3, padx=12, pady=4, sticky="ew")
        ttk.Label(connection, text="OAK IP").grid(row=0, column=0, padx=(0, 6))
        self.ip_text = tk.StringVar(value=device_ip)
        self.ip_entry = ttk.Entry(connection, textvariable=self.ip_text, width=20)
        self.ip_entry.grid(row=0, column=1, padx=(0, 6), sticky="ew")
        self.discover_button = ttk.Button(
            connection,
            text="IP 자동검색",
            command=self.discover_ip,
        )
        self.discover_button.grid(row=0, column=2)
        ttk.Label(connection, text="비밀번호").grid(
            row=1,
            column=0,
            padx=(0, 6),
            pady=(6, 0),
        )
        self.password_text = tk.StringVar()
        self.password_entry = ttk.Entry(
            connection,
            textvariable=self.password_text,
            show="●",
            width=20,
        )
        self.password_entry.grid(
            row=1,
            column=1,
            columnspan=2,
            pady=(6, 0),
            sticky="ew",
        )
        ttk.Label(
            connection,
            text="입력하면 암호화 저장 · 빈칸이면 저장된 비밀번호 사용",
        ).grid(row=2, column=1, columnspan=2, sticky="w")
        connection.columnconfigure(1, weight=1)

        self.connect_button = ttk.Button(root, text="영상 연결", command=self.connect_video)
        self.connect_button.grid(row=2, column=0, columnspan=3, padx=12, pady=4, sticky="ew")

        choices = ttk.Frame(root)
        choices.grid(row=3, column=0, columnspan=3, padx=12, pady=4, sticky="w")
        ttk.Label(choices, text="녹화 대상:").pack(side=tk.LEFT, padx=(0, 8))
        self.rgb_check = ttk.Checkbutton(
            choices,
            text="원본 RGB",
            variable=self.selected["rgb"],
        )
        self.result_check = ttk.Checkbutton(
            choices,
            text="YOLO 결과",
            variable=self.selected["yolopv2"],
        )
        self.rgb_check.pack(side=tk.LEFT, padx=(0, 8))
        self.result_check.pack(side=tk.LEFT, padx=(0, 16))
        ttk.Label(choices, text="미리보기:").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Radiobutton(
            choices,
            text="YOLO 결과",
            variable=self.preview_source,
            value="yolopv2",
        ).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Radiobutton(
            choices,
            text="원본 RGB",
            variable=self.preview_source,
            value="rgb",
        ).pack(side=tk.LEFT)

        self.start_button = ttk.Button(
            root,
            text="녹화 시작",
            command=self.start_recording,
            state=tk.DISABLED,
        )
        self.stop_button = ttk.Button(
            root,
            text="녹화 멈춤",
            command=self.stop_recording,
            state=tk.DISABLED,
        )
        self.save_button = ttk.Button(
            root,
            text="파일 저장",
            command=self.save_recording,
            state=tk.DISABLED,
        )
        self.start_button.grid(row=4, column=0, padx=(12, 4), pady=4, sticky="ew")
        self.stop_button.grid(row=4, column=1, padx=4, pady=4, sticky="ew")
        self.save_button.grid(row=4, column=2, padx=(4, 12), pady=4, sticky="ew")

        self.status_text = tk.StringVar(value="영상 연결 대기 중")
        ttk.Label(root, textvariable=self.status_text, anchor="w").grid(
            row=5,
            column=0,
            columnspan=3,
            padx=12,
            pady=(4, 12),
            sticky="ew",
        )
        self.root.after(50, self._update_preview)

    def _selected_keys(self) -> list[str]:
        return [key for key, selected in self.selected.items() if selected.get()]

    def connect_video(self) -> None:
        if self.connect_requested:
            return
        entered_ip = self.ip_text.get().strip()
        if (
            self.rgb_url_override is None
            and self.result_url_override is None
            and (not entered_ip or entered_ip == DEFAULT_DEVICE_IP)
        ):
            discovered_ip = self._discover_device_ip()
            if discovered_ip is not None:
                entered_ip = discovered_ip
                self.ip_text.set(discovered_ip)
        if not entered_ip:
            messagebox.showwarning("영상 연결", "OAK IP를 입력해 주세요.")
            return

        self.device_ip = entered_ip
        password = self.password_text.get()
        if password:
            try:
                save_password(self.device_ip, password)
            except OSError as exc:
                messagebox.showwarning(
                    "비밀번호 저장",
                    f"비밀번호를 안전하게 저장하지 못했습니다.\n{exc}",
                )
        else:
            password = load_password(self.device_ip)
        self.password_text.set("")
        self.connect_requested = True
        self.connect_button.configure(text="OAK 앱 시작 중...", state=tk.DISABLED)
        self.ip_entry.configure(state=tk.DISABLED)
        self.password_entry.configure(state=tk.DISABLED)
        self.discover_button.configure(state=tk.DISABLED)
        self.status_text.set(f"{self.device_ip} OAK 영상 앱 확인 중...")
        threading.Thread(
            target=self._prepare_connection,
            args=(self.device_ip, password),
            daemon=True,
            name="oak-app-launcher",
        ).start()

    def discover_ip(self) -> None:
        self.status_text.set("OAK IP 검색 중...")
        discovered_ip = self._discover_device_ip()
        if discovered_ip is None:
            self.status_text.set("OAK 장비를 찾지 못했습니다")
            messagebox.showwarning("IP 자동검색", "연결된 OAK 장비를 찾지 못했습니다.")
            return
        self.device_ip = discovered_ip
        self.ip_text.set(discovered_ip)
        self.status_text.set(f"OAK IP 검색 완료: {discovered_ip}")

    @staticmethod
    def _rtsp_ports_open(ip: str) -> bool:
        for port in (8554, 8555):
            try:
                with socket.create_connection((ip, port), timeout=1.0):
                    pass
            except OSError:
                return False
        return True

    def _prepare_connection(self, ip: str, password: str) -> None:
        if self._rtsp_ports_open(ip):
            self.connection_events.put(("ready", ip))
            return
        if not OAKCTL_PATH.exists():
            self.connection_events.put(("error", f"oakctl을 찾을 수 없습니다: {OAKCTL_PATH}"))
            return

        command = [str(OAKCTL_PATH)]
        if password:
            command.extend(["--password", password])
        command.extend(
            [
                "app",
                "run",
                str(OAK_APP_PATH),
                "--detach",
                "-d",
                ip,
            ]
        )
        try:
            completed = subprocess.run(
                command,
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=900,
                creationflags=subprocess.CREATE_NO_WINDOW,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.connection_events.put(("error", str(exc)))
            return
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout or "OAK 앱 실행 실패").strip()
            self.connection_events.put(("error", detail[-1000:]))
            return

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if self._rtsp_ports_open(ip):
                self.connection_events.put(("ready", ip))
                return
            time.sleep(1.0)
        self.connection_events.put(
            (
                "error",
                "OAK 앱 실행 명령은 완료됐지만 60초 안에 RGB/YOLO 영상 포트가 열리지 않았습니다. "
                "장비 연결과 앱 상태를 확인해 주세요.",
            )
        )

    def _begin_receivers(self, ip: str) -> None:
        self.sources["rgb"].url = self.rgb_url_override or f"rtsp://{ip}:8555/rgb"
        self.sources["yolopv2"].url = (
            self.result_url_override or f"rtsp://{ip}:8554/yolopv2"
        )
        for source in self.sources.values():
            source.start()
        self.connect_button.configure(text="영상 연결 중...", state=tk.DISABLED)
        self.status_text.set(f"{ip} RGB 및 YOLO 스트림 연결 중...")

    def _drain_connection_events(self) -> None:
        while True:
            try:
                event, detail = self.connection_events.get_nowait()
            except queue.Empty:
                return
            if event == "ready":
                self._begin_receivers(detail)
                continue

            self.connect_requested = False
            self.connect_button.configure(text="영상 연결", state=tk.NORMAL)
            self.ip_entry.configure(state=tk.NORMAL)
            self.password_entry.configure(state=tk.NORMAL)
            self.discover_button.configure(state=tk.NORMAL)
            self.status_text.set("OAK 앱 실행 실패")
            messagebox.showerror("영상 연결", detail)

    def _discover_device_ip(self) -> str | None:
        if not OAKCTL_PATH.exists():
            return None
        try:
            completed = subprocess.run(
                [
                    str(OAKCTL_PATH),
                    "device",
                    "list",
                    "--all",
                    "--format",
                    "json",
                ],
                capture_output=True,
                text=True,
                timeout=15,
                creationflags=subprocess.CREATE_NO_WINDOW,
                check=True,
            )
            devices = json.loads(completed.stdout).get("items", [])
        except (OSError, subprocess.SubprocessError, json.JSONDecodeError):
            return None
        for device in devices:
            if str(device.get("serial_number")) != self.device_serial:
                continue
            addresses = device.get("ip_addresses") or []
            if addresses:
                return str(addresses[0])
        return None

    def _update_preview(self) -> None:
        self._drain_connection_events()
        result_connected, result_frame = self.sources["yolopv2"].snapshot()
        rgb_connected, rgb_frame = self.sources["rgb"].snapshot()
        preferred_key = self.preview_source.get()
        if preferred_key == "rgb":
            frame = rgb_frame if rgb_connected else result_frame
            preview_name = "원본 RGB" if rgb_connected else "YOLO 결과"
        else:
            frame = result_frame if result_connected else rgb_frame
            preview_name = "YOLO 결과" if result_connected else "원본 RGB"

        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            image.thumbnail(PREVIEW_SIZE, Image.Resampling.LANCZOS)
            photo = ImageTk.PhotoImage(image)
            self.preview.configure(image=photo, text=preview_name)
            self.preview.image = photo

        connection_state = (rgb_connected, result_connected)
        if rgb_connected or result_connected:
            self.connect_button.configure(text="영상 연결됨", state=tk.DISABLED)
            if not self.recording and not self.unsaved_paths:
                self.start_button.configure(state=tk.NORMAL)
        if (
            connection_state != self.last_connection_state
            and self.connect_requested
            and not self.recording
            and not self.unsaved_paths
        ):
            self.last_connection_state = connection_state
            states = [
                f"RGB {'연결됨' if rgb_connected else '연결 중'}",
                f"YOLO {'연결됨' if result_connected else '연결 중'}",
            ]
            self.status_text.set(" / ".join(states))

        self.root.after(50, self._update_preview)

    def _discard_unsaved(self) -> bool:
        if not self.unsaved_paths:
            return True
        discard = messagebox.askyesno(
            "저장하지 않은 영상",
            "이전 녹화가 아직 저장되지 않았습니다. 이전 녹화를 버리고 새로 시작할까요?",
        )
        if not discard:
            return False
        for path in self.unsaved_paths.values():
            path.unlink(missing_ok=True)
        self.unsaved_paths.clear()
        return True

    def start_recording(self) -> None:
        keys = self._selected_keys()
        if not keys:
            messagebox.showwarning("녹화 시작", "원본 RGB 또는 YOLO 결과를 선택해 주세요.")
            return
        if not self._discard_unsaved():
            return

        unavailable = [key for key in keys if not self.sources[key].snapshot()[0]]
        if unavailable:
            labels = ", ".join(self.sources[key].label for key in unavailable)
            messagebox.showwarning(
                "녹화 시작",
                f"다음 영상이 아직 연결되지 않았습니다: {labels}",
            )
            return

        self.recording_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        started: list[str] = []
        for key in keys:
            temp_path = self.output_dir / f".oak4dpro_{self.recording_stamp}_{key}.mp4"
            if not self.sources[key].start_recording(temp_path):
                for started_key in started:
                    path = self.sources[started_key].stop_recording()
                    if path is not None:
                        path.unlink(missing_ok=True)
                messagebox.showerror(
                    "녹화 시작",
                    f"{self.sources[key].label} MP4 파일을 열 수 없습니다.",
                )
                return
            started.append(key)

        self.recording_keys = started
        self.recording = True
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)
        self.save_button.configure(state=tk.DISABLED)
        self.rgb_check.configure(state=tk.DISABLED)
        self.result_check.configure(state=tk.DISABLED)
        labels = " + ".join(self.sources[key].label for key in started)
        self.status_text.set(f"녹화 중 ● {labels}")

    def stop_recording(self) -> None:
        if not self.recording:
            return
        self.unsaved_paths = {
            key: path
            for key in self.recording_keys
            if (path := self.sources[key].stop_recording()) is not None
        }
        self.recording = False
        self.recording_keys = []
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.DISABLED)
        self.save_button.configure(state=tk.NORMAL)
        self.rgb_check.configure(state=tk.NORMAL)
        self.result_check.configure(state=tk.NORMAL)
        self.status_text.set("녹화 멈춤 — 파일 저장 버튼을 눌러 주세요")

    def save_recording(self) -> None:
        if self.recording:
            self.stop_recording()
        existing = {
            key: path for key, path in self.unsaved_paths.items() if path.exists()
        }
        if not existing:
            messagebox.showinfo("파일 저장", "저장할 녹화 영상이 없습니다.")
            return

        if len(existing) == 1:
            key, source = next(iter(existing.items()))
            suffix = "rgb" if key == "rgb" else "yolopv2"
            destination = filedialog.asksaveasfilename(
                title="녹화 영상 저장",
                defaultextension=".mp4",
                initialfile=f"oak4dpro_{self.recording_stamp}_{suffix}.mp4",
                filetypes=[("MP4 동영상", "*.mp4")],
            )
            if not destination:
                return
            destinations = {key: Path(destination)}
        else:
            directory = filedialog.askdirectory(title="두 녹화 영상을 저장할 폴더 선택")
            if not directory:
                return
            folder = Path(directory)
            destinations = {
                "rgb": folder / f"oak4dpro_{self.recording_stamp}_rgb.mp4",
                "yolopv2": folder / f"oak4dpro_{self.recording_stamp}_yolopv2.mp4",
            }

        for key, source in existing.items():
            destination = destinations[key]
            if source.resolve() != destination.resolve():
                shutil.copy2(source, destination)
                source.unlink(missing_ok=True)

        self.unsaved_paths.clear()
        self.save_button.configure(state=tk.DISABLED)
        saved = ", ".join(str(path) for path in destinations.values())
        self.status_text.set(f"저장 완료: {saved}")

    def _close(self) -> None:
        if self.recording:
            self.stop_recording()
        if self.unsaved_paths:
            close = messagebox.askyesno(
                "저장하지 않은 영상",
                "저장하지 않은 녹화를 삭제하고 창을 닫을까요?",
            )
            if not close:
                return
            for path in self.unsaved_paths.values():
                path.unlink(missing_ok=True)
            self.unsaved_paths.clear()
        for source in self.sources.values():
            source.close()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OAK4-D Pro dual MP4 recorder")
    parser.add_argument("--device-ip", default=DEFAULT_DEVICE_IP)
    parser.add_argument("--device-serial", default=DEFAULT_DEVICE_SERIAL)
    parser.add_argument("--rgb-url", help="override the RGB RTSP URL")
    parser.add_argument("--result-url", help="override the YOLOPv2 RTSP URL")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "artifacts" / "recordings",
        help="temporary recording directory",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    RecorderApp(
        root,
        args.device_ip,
        args.device_serial,
        args.rgb_url,
        args.result_url,
        args.output_dir,
    )
    root.mainloop()


if __name__ == "__main__":
    main()
