from __future__ import annotations

import argparse
import base64
import ctypes
from ctypes import wintypes
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')
import cv2
from PIL import Image, ImageTk
from frame_csv_logger import FrameCsvLogger
from six_camera_tab import SixCameraTab
import oak_app_control
import video_input_settings
import settings_store
from video_file_sender import VideoFileSender, select_camera, parse_video_time, set_processing_mode, set_boundary_display, set_lane_overlay

RESAMPLE_LANCZOS = getattr(Image, "Resampling", Image).LANCZOS

DEFAULT_DEVICE_IP = "192.168.200.48"
DEFAULT_DEVICE_SERIAL = "1142227768"
IS_WINDOWS = os.name == "nt"
UI_FONT = "Malgun Gothic" if IS_WINDOWS else "Noto Sans CJK KR"
OAKCTL_PATH = Path(shutil.which("oakctl") or (
    r"C:\Program Files\oakctl\bin\oakctl.exe" if IS_WINDOWS else "/usr/local/bin/oakctl"
))
# WSL can use the authenticated Windows installation without installing a
# separate CLI or copying credentials into Ubuntu.
if not IS_WINDOWS and not OAKCTL_PATH.exists():
    windows_cli = Path('/mnt/c/Program Files/oakctl/bin/oakctl.exe')
    if windows_cli.exists():
        OAKCTL_PATH = windows_cli
OAK_APP_PATH = Path(__file__).resolve().parents[1] / "apps" / "yolopv2_standalone"
DEFAULT_FPS = 10.0
PREVIEW_SIZE = (480, 270)
CREDENTIAL_FILE = settings_store.DEFAULT_PATH


class DataBlob(ctypes.Structure):
    _fields_ = [("size", wintypes.DWORD), ("data", ctypes.POINTER(ctypes.c_byte))]


def _crypt_protect(secret: str) -> str:
    if not IS_WINDOWS:
        from windows_secret import crypt
        return crypt(secret)
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
    if not IS_WINDOWS:
        from windows_secret import crypt
        return crypt(encoded, decrypt=True)
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


def save_password(device_ip: str, password: str, path=None) -> None:
    credential_path=Path(path) if path is not None else CREDENTIAL_FILE
    credentials = settings_store.read(credential_path).get('credentials', {})
    if not isinstance(credentials,dict):raise ValueError('잘못된 암호화 비밀번호 설정')
    credentials[device_ip] = _crypt_protect(password)
    settings_store.update(credential_path, {'credentials':credentials})


def load_password(device_ip: str, path=None) -> str:
    try:
        credentials = settings_store.read(Path(path) if path is not None else CREDENTIAL_FILE).get('credentials', {})
        if not isinstance(credentials, dict):
            return ""
        encrypted = credentials.get(device_ip)
        return _crypt_unprotect(encrypted) if encrypted else ""
    except (OSError, ValueError, TypeError, subprocess.SubprocessError):
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
        self.writer_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest_frame = None
        self.frames_received = 0
        self.last_frame_at = 0.
        self.writer: cv2.VideoWriter | None = None
        self.temp_path: Path | None = None
        self.connected = False
        self.started = False
        self.thread = None
        self.record_started_at = 0.
        self.record_written = 0
        self.reconnects = 0
        self.connections = 0
        self.record_error = ''
        self.record_finalize_errors = []

    def start(self) -> None:
        if self.started:
            return
        self.started = True
        self.thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"oak-capture-{self.key}",
        )
        self.thread.start()

    def _capture_loop(self) -> None:
        while not self.stop_event.is_set():
            capture = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000,
            ])
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                with self.lock:
                    self.connected = False
                self.stop_event.wait(1.0)
                continue

            with self.lock:
                self.connected = True
                self.reconnects = self.connections
                self.connections += 1
                self.latest_frame = None
            while not self.stop_event.is_set():
                ok, frame = capture.read()
                if not ok:
                    break
                with self.lock:
                    self.latest_frame = frame
                    self.frames_received += 1
                    self.last_frame_at = time.monotonic()
                try:self._write_recording_frame(frame,time.monotonic())
                except Exception as exc:
                    if not self.record_error:self.record_error=str(exc)

            capture.release()
            with self.lock:
                self.connected = False

    def snapshot(self):
        with self.lock:
            frame,connected = self.latest_frame,self.connected
        return connected, None if frame is None else frame.copy()

    def record_snapshot(self):
        with self.lock:
            frame,connected,sequence,stamp = self.latest_frame,self.connected,self.frames_received,self.last_frame_at
        return connected, None if frame is None else frame.copy(), sequence, stamp

    def info(self):
        with self.lock:
            return dict(connected=self.connected,frames_received=self.frames_received,
                        last_frame_age_s=round(time.monotonic()-self.last_frame_at,3),url=self.url,
                        reconnects=self.reconnects,record_error=self.record_error,
                        record_finalize_errors=list(self.record_finalize_errors))

    def _write_recording_frame(self, frame, now):
        # Keep the existing 10-FPS recording timebase when live input is 15/20.
        # These recording-only repeats never enter inference or telemetry.
        with self.writer_lock:
            if self.writer is None:return
            target = int(max(0.,now-self.record_started_at)*DEFAULT_FPS)+1
            for _ in range(min(50,max(0,target-self.record_written))):
                self.writer.write(frame)
                self.record_written += 1

    def start_recording(self, path: Path) -> bool:
        connected, frame = self.snapshot()
        if not connected or frame is None:
            return False
        writer = create_video_writer(path, frame)
        if writer is None:
            return False
        with self.writer_lock:
            if self.writer is not None:
                writer.release()
                return False
            self.writer = writer
            self.temp_path = path
            self.record_started_at = time.monotonic()
            self.record_written = 0
            self.record_error = ''
            self.record_finalize_errors = []
        return True

    def stop_recording(self) -> Path | None:
        with self.writer_lock:
            writer = self.writer
            path = self.temp_path
            self.writer = None
            self.temp_path = None
            if writer is not None:
                try:writer.release()
                except Exception as exc:
                    self.record_finalize_errors.append(str(exc))
                    if not self.record_error:self.record_error=str(exc)
        return path

    def close(self) -> None:
        self.stop_event.set()
        self.stop_recording()
        if self.thread is not None:
            self.thread.join(timeout=6)


class RecorderApp:
    def __init__(
        self,
        root: tk.Tk,
        device_ip: str,
        device_serial: str,
        rgb_url: str | None,
        result_url: str | None,
        bev_url: str | None,
        output_dir: Path,
        camera_settings_path=None,
        use_saved_connection=False,
    ) -> None:
        self.root = root
        self.device_ip = device_ip
        self.device_serial = device_serial
        self.rgb_url_override = rgb_url
        self.result_url_override = result_url
        self.bev_url_override = bev_url
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_logger = None
        self.file_sender = None
        self.active_input = 'camera'
        self.source_events = queue.SimpleQueue()
        self.source_change_pending = False
        self.applied_input_settings = None
        self.processing_events = queue.SimpleQueue()
        self.processing_pending = False
        self.confirmed_full_processing = False
        self.boundary_display_pending = False
        self.confirmed_boundary_visible = True
        self.lane_preview_pending = False
        self.confirmed_lane_preview = ('normal',.5)
        self.source_closing = threading.Event()
        self.sources = {
            "rgb": CaptureSource("rgb", "원본 RGB", ""),
            "yolopv2": CaptureSource("yolopv2", "YOLO 결과", ""),
            "bev": CaptureSource("bev", "TOP 뷰 · 도로 RGB (미보정)", ""),
            "depth": CaptureSource("depth", "RGB 정렬 Depth", ""),
        }
        self.selected = {
            "rgb": tk.BooleanVar(value=True),
            "yolopv2": tk.BooleanVar(value=True),
        }
        self.unsaved_paths: dict[str, Path] = {}
        self.recording_keys: list[str] = []
        self.recording_stamp = ""
        self.recording = False
        self.connect_requested = False
        self.video_wait_started_at = None
        self.video_retry_available = False
        self.app_action_busy = False
        self.connection_events: queue.SimpleQueue[tuple[str, str]] = queue.SimpleQueue()
        self.last_connection_state: tuple[bool, ...] | None = None

        self.preview_labels: dict[str, ttk.Label] = {}
        self.preview_status: dict[str, tk.StringVar] = {}

        self.root.title("OAK 멀티뷰 · 인식 검증 / 6대 녹화")
        self.root.geometry("1280x900")
        self.root.minsize(1040, 760)
        self.root.configure(background="#0b1118")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("App.TFrame", background="#0b1118")
        style.configure("Header.TFrame", background="#111b25")
        style.configure("Panel.TFrame", background="#131f2a")
        style.configure(
            "Video.TLabel",
            background="#050a0f",
            foreground="#8da0af",
            anchor="center",
            font=(UI_FONT, 11),
        )
        style.configure(
            "Title.TLabel",
            background="#111b25",
            foreground="#f1f6f9",
            font=(UI_FONT, 16, "bold"),
        )
        style.configure(
            "Subtitle.TLabel",
            background="#111b25",
            foreground="#8da0af",
            font=(UI_FONT, 9),
        )
        style.configure(
            "PanelTitle.TLabel",
            background="#131f2a",
            foreground="#f1f6f9",
            font=(UI_FONT, 10, "bold"),
        )
        style.configure(
            "PanelStatus.TLabel",
            background="#131f2a",
            foreground="#21d3a1",
            font=(UI_FONT, 9),
        )
        style.configure(
            "App.TLabel",
            background="#0b1118",
            foreground="#c3d0d9",
        )
        style.configure(
            "App.TCheckbutton",
            background="#0b1118",
            foreground="#c3d0d9",
        )
        style.map(
            "App.TCheckbutton",
            background=[("active", "#0b1118")],
            foreground=[("active", "#f1f6f9")],
        )
        style.configure(
            "Control.TLabelframe",
            background="#0b1118",
            foreground="#dce7ed",
        )
        style.configure(
            "Control.TLabelframe.Label",
            background="#0b1118",
            foreground="#dce7ed",
        )

        self.notebook = ttk.Notebook(root)
        self.notebook.grid(row=0, column=0, sticky='nsew')
        self.perception_tab = ttk.Frame(self.notebook, style='App.TFrame')
        self.perception_tab.columnconfigure(0, weight=1)
        self.perception_tab.rowconfigure(1, weight=1)
        self.notebook.add(self.perception_tab, text='인식·검증')
        self.six_camera_frame = ttk.Frame(self.notebook, style='App.TFrame')
        self.notebook.add(self.six_camera_frame, text='6대 카메라 녹화')
        self.six_camera_tab = SixCameraTab(root, self.six_camera_frame,
                                         device_ip, device_serial, output_dir,
                                         owner=self,settings_path=camera_settings_path,
                                         restore_cam01=use_saved_connection)
        if use_saved_connection:
            self.device_ip=self.six_camera_tab.cameras[0]['ip'].get()
            self.device_serial=self.six_camera_tab.cameras[0]['serial'].get()

        header = ttk.Frame(self.perception_tab, style="Header.TFrame", padding=(18, 12))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        title_group = ttk.Frame(header, style="Header.TFrame")
        title_group.grid(row=0, column=0, sticky="w")
        ttk.Label(
            title_group,
            text="CAM01 Perception Lab",
            style="Title.TLabel",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            title_group,
            text="OAK 4D PRO · RGB / YOLOPv2 / Depth / BEV",
            style="Subtitle.TLabel",
        ).grid(row=1, column=0, sticky="w")
        ttk.Label(
            header,
            text="멀티뷰 UI · 개발 검증 화면",
            style="Subtitle.TLabel",
        ).grid(row=0, column=1, rowspan=2, sticky="e")

        self.settings_windows = {}
        menu_bar = ttk.Frame(header, style="Header.TFrame")
        menu_bar.grid(row=2, column=0, sticky="w", pady=(10, 0))
        for key, label in (("device", "장비 연결"), ("source", "영상 입력"),
                           ("display", "표시 설정")):
            ttk.Button(menu_bar, text=label,
                       command=lambda key=key: self._show_settings(key)).pack(
                           side=tk.LEFT, padx=(0, 6))
        camera_actions = ttk.Frame(header, style='Header.TFrame')
        camera_actions.grid(row=2, column=1, sticky='e', pady=(10, 0))
        self.app_led = tk.Label(camera_actions, text='●', fg='#8da0af', bg='#111b25',
                                font=(UI_FONT, 16))
        self.app_led.pack(side=tk.LEFT)
        self.app_state_text = tk.StringVar(value='상태 미확인')
        ttk.Label(camera_actions, textvariable=self.app_state_text,
                  style='Subtitle.TLabel').pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(camera_actions, text='카메라 상태 확인',
                   command=self.check_camera_status).pack(side=tk.LEFT, padx=4)
        ttk.Button(camera_actions, text='카메라 앱 실행',
                   command=lambda: self._camera_app_action('start')).pack(side=tk.LEFT)
        ttk.Button(camera_actions, text='앱 실행 멈춤',
                   command=lambda: self._camera_app_action('stop')).pack(side=tk.LEFT, padx=(4, 0))
        self.root.after(5000, self._poll_camera_app)

        device_window = self._settings_window("device", "장비 연결")
        self._settings_window("source", "영상 입력")
        self._settings_window("display", "표시 설정")

        previews = ttk.Frame(self.perception_tab, style="App.TFrame", padding=(14, 12, 14, 6))
        previews.grid(row=1, column=0, sticky="nsew")
        for index in range(2):
            previews.columnconfigure(index, weight=1, uniform="preview-column")
            previews.rowconfigure(index, weight=1, uniform="preview-row")

        preview_specs = (
            ("rgb", "원본 RGB", "영상 연결 대기"),
            ("yolopv2", "인식 결과", "영상 연결 대기"),
            ("depth", "Depth · 빨강 0.3m → 파랑 25m · 검정: 무효", "Depth 영상 연결 대기"),
            ("bev", "TOP 뷰 · 도로 RGB (미보정)", "TOP 뷰 연결 대기"),
        )
        for index, (key, title, placeholder) in enumerate(preview_specs):
            self._build_preview_panel(
                previews,
                index // 2,
                index % 2,
                key,
                title,
                placeholder,
            )

        controls = ttk.Frame(self.perception_tab, style="App.TFrame", padding=(14, 6, 14, 12))
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure(0, weight=1)

        connection = ttk.LabelFrame(
            device_window,
            text="장비 연결",
            style="Control.TLabelframe",
            padding=8,
        )
        connection.grid(row=0, column=0, sticky="ew", padx=14, pady=14)
        ttk.Label(connection, text="OAK IP", style="App.TLabel").grid(
            row=0, column=0, padx=(0, 6)
        )
        self.ip_text = tk.StringVar(value=self.device_ip)
        self.ip_entry = ttk.Entry(connection, textvariable=self.ip_text, width=20)
        self.ip_entry.grid(row=0, column=1, padx=(0, 6), sticky="ew")
        self.discover_button = ttk.Button(
            connection,
            text="IP 자동검색",
            command=self.discover_ip,
        )
        self.discover_button.grid(row=0, column=2)
        ttk.Label(connection, text="비밀번호", style="App.TLabel").grid(
            row=0, column=3, padx=(14, 6)
        )
        self.password_text = tk.StringVar()
        self.password_entry = ttk.Entry(
            connection,
            textvariable=self.password_text,
            show="●",
            width=20,
        )
        self.password_entry.grid(row=0, column=4, sticky="ew")
        ttk.Label(
            connection,
            text="IP·비밀번호 저장 버튼으로 저장 · 빈칸이면 저장된 비밀번호 사용",
            style="App.TLabel",
        ).grid(row=1, column=3, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Button(connection, text='IP·비밀번호 저장',
                   command=self.save_connection_settings).grid(
                       row=2, column=3, columnspan=2, sticky='e', pady=(8, 0))
        connection.columnconfigure(1, weight=1)
        connection.columnconfigure(4, weight=1)

        self.connect_button = ttk.Button(
            connection,
            text="영상 연결",
            command=self.connect_video,
        )
        self.connect_button.grid(row=0, column=5, rowspan=2, padx=(10, 0), sticky="ns")

        record_controls = ttk.Frame(controls, style="App.TFrame")
        record_controls.grid(row=1, column=0, pady=(8, 0), sticky="ew")
        record_controls.columnconfigure(1, weight=1)
        choices = ttk.Frame(record_controls, style="App.TFrame")
        choices.grid(row=0, column=0, sticky="w")
        ttk.Label(choices, text="녹화 대상:", style="App.TLabel").pack(
            side=tk.LEFT, padx=(0, 8)
        )
        self.rgb_check = ttk.Checkbutton(
            choices,
            text="원본 RGB",
            variable=self.selected["rgb"],
            style="App.TCheckbutton",
        )
        self.result_check = ttk.Checkbutton(
            choices,
            text="YOLO 결과",
            variable=self.selected["yolopv2"],
            style="App.TCheckbutton",
        )
        self.rgb_check.pack(side=tk.LEFT, padx=(0, 8))
        self.result_check.pack(side=tk.LEFT)

        self.start_button = ttk.Button(
            record_controls,
            text="녹화 시작",
            command=self.start_recording,
            state=tk.DISABLED,
        )
        self.stop_button = ttk.Button(
            record_controls,
            text="녹화 멈춤",
            command=self.stop_recording,
            state=tk.DISABLED,
        )
        self.save_button = ttk.Button(
            record_controls,
            text="파일 저장",
            command=self.save_recording,
            state=tk.DISABLED,
        )
        self.start_button.grid(row=0, column=2, padx=(4, 2))
        self.stop_button.grid(row=0, column=3, padx=2)
        self.save_button.grid(row=0, column=4, padx=(2, 0))

        self.status_text = tk.StringVar(value="영상 연결 대기 중")
        ttk.Label(
            record_controls,
            textvariable=self.status_text,
            anchor="w",
            style="App.TLabel",
        ).grid(
            row=1,
            column=0,
            columnspan=5,
            pady=(8, 0),
            sticky="ew",
        )
        self.root.after(50, self._update_preview)
        self.csv_status = tk.StringVar(value='CSV 대기 · 녹화 시작 시 함께 기록')
        ttk.Label(record_controls, textvariable=self.csv_status, style='App.TLabel',
                  wraplength=1100).grid(row=2, column=0, columnspan=5, sticky='ew')

        ttk.Label(controls,
                  text="경계선: 왼쪽 노랑 / 오른쪽 하늘색 · 영상 흰색선 우선 · 움직임 보정 추정 · 고정 폭 없음",
                  style="App.TLabel").grid(row=3, column=0, sticky="w", pady=(6, 0))
        self._build_source_controls(controls)
        self._restore_preferences()
        self.notebook.bind('<<NotebookTabChanged>>', self._hide_settings_popups)

    def _hide_settings_popups(self, event=None):
        for window in (*self.settings_windows.values(), *self.six_camera_tab.windows.values()):
            window.withdraw()

    def _build_source_controls(self, controls):
        panel = ttk.LabelFrame(self.settings_windows['source'], text='영상 입력 소스',
                               style='Control.TLabelframe', padding=8)
        panel.grid(row=0, column=0, sticky='ew', padx=14, pady=14)
        panel.columnconfigure(1, weight=1)
        self.input_mode = tk.StringVar(value='카메라')
        self.input_file = tk.StringVar()
        self.input_loop = tk.BooleanVar(value=True)
        self.input_start = tk.StringVar(value='0')
        self.input_end = tk.StringVar(value='')
        self.input_fps = tk.StringVar(value='10')
        ttk.Combobox(panel, textvariable=self.input_mode, state='readonly',
                     values=('카메라', 'PC 영상 파일'), width=15).grid(row=0,column=0,padx=(0,6))
        ttk.Entry(panel, textvariable=self.input_file, state='readonly').grid(
            row=0,column=1,sticky='ew')
        ttk.Button(panel, text='영상 파일 선택', command=self._choose_input_file).grid(
            row=0,column=2,padx=5)
        ttk.Checkbutton(panel, text='반복 재생', variable=self.input_loop,
                        style='App.TCheckbutton').grid(row=0,column=3)
        self.source_apply = ttk.Button(panel, text='소스 적용',
                                       command=self._apply_input_source, state=tk.DISABLED)
        self.source_apply.grid(row=0,column=4,padx=5)
        ttk.Button(panel, text='전송 중지', command=self._stop_input).grid(
            row=0,column=6,padx=(5,0))
        segment = ttk.Frame(panel, style='Panel.TFrame')
        segment.grid(row=1,column=0,columnspan=7,sticky='w',pady=(5,0))
        ttk.Label(segment,text='전송 구간 시작',style='App.TLabel').pack(side=tk.LEFT)
        self.source_start_entry = ttk.Entry(segment,textvariable=self.input_start,width=10)
        self.source_start_entry.pack(side=tk.LEFT,padx=5)
        ttk.Label(segment,text='종료',style='App.TLabel').pack(side=tk.LEFT)
        self.source_end_entry = ttk.Entry(segment,textvariable=self.input_end,width=10)
        self.source_end_entry.pack(side=tk.LEFT,padx=5)
        ttk.Label(segment,text='PC 전송 FPS',style='App.TLabel').pack(side=tk.LEFT,padx=(8,0))
        self.source_fps_choice = ttk.Combobox(segment,textvariable=self.input_fps,state='readonly',
                                             values=('10','15','20'),width=4)
        self.source_fps_choice.pack(side=tk.LEFT,padx=5)
        ttk.Label(segment,text='초 또는 분:초 · 종료 공란: 영상 끝 · 변경 후 소스 적용',
                  style='App.TLabel').pack(side=tk.LEFT)
        source_panel = panel
        panel = ttk.LabelFrame(self.settings_windows['display'], text='인식·표시 설정',
                               style='Control.TLabelframe', padding=8)
        panel.grid(row=0, column=0, sticky='ew', padx=14, pady=14)
        panel.columnconfigure(3, weight=1)
        self.full_processing = tk.BooleanVar(value=False)
        self.processing_check = ttk.Checkbutton(panel,text='전체 후처리 (경계 추적·가상 경계·정지선)',
            variable=self.full_processing,command=self._apply_processing_mode,state=tk.DISABLED,
            style='App.TCheckbutton')
        self.processing_check.grid(row=4,column=0,columnspan=3,sticky='w',pady=(4,0))
        self.processing_status = tk.StringVar(value='기본 인식: 객체·주행 영역·차선 마스크')
        ttk.Label(panel,textvariable=self.processing_status,style='App.TLabel').grid(
            row=4,column=3,columnspan=4,sticky='w')
        self.boundary_visible=tk.BooleanVar(value=True)
        self.boundary_display_check=ttk.Checkbutton(panel,text='차선·가상 경계선 표시',
            variable=self.boundary_visible,command=self._apply_boundary_display,state=tk.DISABLED,
            style='App.TCheckbutton')
        self.boundary_display_check.grid(row=5,column=0,columnspan=3,sticky='w',pady=(4,0))
        self.boundary_display_status=tk.StringVar(value='표시만 변경 · 경계 계산·추종선 유지')
        ttk.Label(panel,textvariable=self.boundary_display_status,style='App.TLabel').grid(
            row=5,column=3,columnspan=4,sticky='w')
        lane_controls=ttk.Frame(panel,style='Panel.TFrame')
        lane_controls.grid(row=6,column=0,columnspan=7,sticky='w',pady=(4,0))
        ttk.Label(lane_controls,text='YOLO 결과 화면',style='App.TLabel').pack(side=tk.LEFT)
        self.lane_preview_modes={'일반 인식 결과':'normal','원본 RGB':'raw_rgb',
            '점수 구간색 (검정 배경)':'lane_bands','원본 + 구간색 겹침':'lane_bands_rgb',
            '원본·구간색·겹침 3분할':'lane_compare',
            '차선 점수 RGB':'lane_score','임계값 마스크 RGB':'lane_mask'}
        self.lane_preview_view=tk.StringVar(value='일반 인식 결과')
        self.lane_preview_choice=ttk.Combobox(lane_controls,textvariable=self.lane_preview_view,
            values=tuple(self.lane_preview_modes),state='readonly',width=24)
        self.lane_preview_choice.pack(side=tk.LEFT,padx=5)
        self.lane_preview_choice.bind('<<ComboboxSelected>>',self._apply_lane_preview)
        ttk.Label(lane_controls,text='차선 점수 임계값',style='App.TLabel').pack(side=tk.LEFT)
        self.lane_preview_threshold=tk.DoubleVar(value=.5)
        self.lane_preview_scale=ttk.Scale(lane_controls,from_=0,to=1,variable=self.lane_preview_threshold,length=130)
        self.lane_preview_scale.pack(side=tk.LEFT,padx=5)
        self.lane_preview_scale.bind('<ButtonRelease-1>',self._apply_lane_preview)
        self.lane_preview_spin=ttk.Spinbox(lane_controls,from_=0,to=1,increment=.01,
            textvariable=self.lane_preview_threshold,width=5,format='%.2f')
        self.lane_preview_spin.pack(side=tk.LEFT,padx=3)
        self.lane_preview_spin.bind('<Return>',self._apply_lane_preview)
        self.lane_preview_apply=ttk.Button(lane_controls,text='적용',command=self._apply_lane_preview,state=tk.DISABLED)
        self.lane_preview_apply.pack(side=tk.LEFT,padx=5)
        self.lane_preview_status=tk.StringVar(value='미리보기 전용 · 차선 마스크 픽셀별 점수색 · 인식용 임계값 0.50 유지')
        ttk.Label(panel,textvariable=self.lane_preview_status,style='App.TLabel').grid(
            row=7,column=0,columnspan=7,sticky='w')
        progress = ttk.LabelFrame(controls, text='영상 재생',
                                  style='Control.TLabelframe', padding=8)
        progress.grid(row=0,column=0,sticky='ew',pady=(4,0))
        progress.columnconfigure(1,weight=1)
        ttk.Label(progress,text='PC 전송 위치',style='App.TLabel').grid(row=0,column=0,padx=(0,6))
        self.source_progress_value = tk.DoubleVar(value=0.)
        self.source_progress_dragging = False
        self.source_progress_seek_id = 0
        self.source_progress = ttk.Scale(progress,from_=0,to=100,variable=self.source_progress_value,
                                         state=tk.DISABLED,command=self._preview_seek)
        self.source_progress.grid(row=0,column=1,sticky='ew')
        self.source_progress.bind('<ButtonPress-1>',self._begin_seek)
        self.source_progress.bind('<ButtonRelease-1>',self._finish_seek)
        for key in ('Left','Right','Up','Down','Home','End'):
            self.source_progress.bind(f'<KeyPress-{key}>',self._begin_seek)
            self.source_progress.bind(f'<KeyRelease-{key}>',self._finish_seek)
        self.source_progress_text = tk.StringVar(value='PC 영상 대기')
        ttk.Label(progress,textvariable=self.source_progress_text,style='App.TLabel',width=46).grid(
            row=0,column=2,padx=(8,0))
        transport = ttk.Frame(progress, style='App.TFrame')
        transport.grid(row=1,column=0,columnspan=3,pady=(8,0))
        self.source_start_button = ttk.Button(transport, text='전송 시작',
            command=self._start_input_transfer, state=tk.DISABLED)
        self.source_start_button.pack(side=tk.LEFT, padx=4)
        self.source_back_button = ttk.Button(transport, text='5초 뒤로', state=tk.DISABLED)
        self.source_back_button.pack(side=tk.LEFT, padx=4)
        self.source_pause = ttk.Button(transport, text='일시정지 / 재생',
                                       command=self._pause_input, state=tk.DISABLED)
        self.source_pause.pack(side=tk.LEFT, padx=4)
        self.source_forward_button = ttk.Button(transport, text='5초 앞으로', state=tk.DISABLED)
        self.source_forward_button.pack(side=tk.LEFT, padx=4)
        self.source_status = tk.StringVar(value='PC 전송 10·15·20 FPS 선택 · 원본/장치 상한 적용')
        ttk.Label(source_panel, textvariable=self.source_status, style='App.TLabel').grid(
            row=3,column=0,columnspan=7,sticky='w',pady=(4,0))
        self._restore_input_settings()

    def _restore_input_settings(self):
        try:
            saved = video_input_settings.load(self.six_camera_tab.settings_path)
        except (OSError, ValueError):
            self.source_status.set('저장된 영상 입력 설정을 읽지 못했습니다 · 다시 선택 후 소스 적용')
            return
        if saved is None:
            return
        for variable, key in ((self.input_mode,'mode'), (self.input_file,'file'),
                              (self.input_loop,'loop'), (self.input_start,'start'),
                              (self.input_end,'end'), (self.input_fps,'fps')):
            variable.set(saved[key])
        if saved['mode'] == 'PC 영상 파일':
            if not Path(saved['file']).is_file():
                self.source_status.set('저장된 영상 파일이 없습니다 · 파일을 다시 선택 후 소스 적용')
                return
            self.applied_input_settings = (saved['mode'], saved['file'], saved['loop'],
                parse_video_time(saved['start']), parse_video_time(saved['end']) if saved['end'].strip() else None,
                saved['fps'])
            self.source_start_button.configure(state=tk.NORMAL)
        self.source_status.set('영상 입력 설정 복원됨 · 카메라는 소스 적용 / PC 영상은 연결 후 전송 시작')

    def _save_preferences(self):
        try:
            six=self.six_camera_tab
            data=settings_store.preferences(dict(
                display=dict(full_processing=self.confirmed_full_processing,
                    boundary_visible=self.confirmed_boundary_visible,
                    lane_mode=self.confirmed_lane_preview[0],threshold=self.confirmed_lane_preview[1]),
                selected={key:var.get() for key,var in self.selected.items()},
                recording={key:getattr(six,key).get() for key in
                    ('output_dir','session_name','resolution','fps','verify_output')}))
            settings_store.update(six.settings_path,{'preferences':data})
            return True
        except (OSError,ValueError,KeyError,TypeError):
            self.status_text.set('설정 저장 실패 · 설정 파일 경로와 내용을 확인해 주세요.')
            return False

    def _restore_preferences(self):
        try:
            data=settings_store.read(self.six_camera_tab.settings_path).get('preferences')
            if data is None:return
            data=settings_store.preferences(data);display=data['display']
            self.confirmed_full_processing=display['full_processing']
            self.full_processing.set(display['full_processing'])
            self.confirmed_boundary_visible=display['boundary_visible']
            self.boundary_visible.set(display['boundary_visible'])
            self.confirmed_lane_preview=(display['lane_mode'],display['threshold'])
            self.lane_preview_view.set(next(k for k,v in self.lane_preview_modes.items() if v==display['lane_mode']))
            self.lane_preview_threshold.set(display['threshold'])
            for key,value in data['selected'].items():self.selected[key].set(value)
            for key,value in data['recording'].items():getattr(self.six_camera_tab,key).set(value)
        except (OSError,ValueError,KeyError,TypeError):
            self.status_text.set('저장된 표시·녹화 설정을 읽지 못했습니다 · 기본값 사용')

    def _settings_window(self, key, title):
        """Keep controls alive when a settings popup is closed or reopened."""
        window = tk.Toplevel(self.root)
        window.withdraw()
        window.title(title)
        window.configure(background='#0b1118')
        window.transient(self.root)
        window.resizable(True, False)
        window.columnconfigure(0, weight=1)
        window.protocol('WM_DELETE_WINDOW', window.withdraw)
        window.bind('<Escape>', lambda event: window.withdraw())
        self.settings_windows[key] = window
        return window

    def _show_settings(self, key):
        for other_key, other in self.settings_windows.items():
            if other_key != key:
                other.withdraw()
        window = self.settings_windows[key]
        window.update_idletasks()
        x = max(0, self.root.winfo_rootx() + (self.root.winfo_width() - window.winfo_reqwidth()) // 2)
        y = max(0, self.root.winfo_rooty() + 90)
        window.geometry(f'+{x}+{y}')
        window.deiconify()
        window.lift()
        window.focus_set()

    @staticmethod
    def _video_clock(seconds):
        tenths = max(0,round(seconds*10))
        minutes, rest = divmod(tenths,600)
        return f'{minutes:02d}:{rest//10:02d}.{rest%10}'

    def _apply_processing_mode(self):
        if self.processing_pending or not self.connect_requested:
            return
        desired=bool(self.full_processing.get())
        self.processing_pending=True
        self.processing_check.configure(state=tk.DISABLED)
        self.processing_status.set('장치 처리 모드 적용 중...')
        ip=self.device_ip
        def change():
            try:
                set_processing_mode(ip,'full' if desired else 'basic')
                self.processing_events.put(('ok',desired))
            except (OSError,ValueError,RuntimeError,EOFError) as exc:
                self.processing_events.put(('error',str(exc)))
        threading.Thread(target=change,daemon=True,name='processing-control').start()

    def _apply_boundary_display(self):
        if self.boundary_display_pending or not self.connect_requested:
            return
        desired=bool(self.boundary_visible.get())
        self.boundary_display_pending=True
        self.boundary_display_check.configure(state=tk.DISABLED)
        self.boundary_display_status.set('경계선 표시 적용 중...')
        ip=self.device_ip
        def change():
            try:
                set_boundary_display(ip,desired)
                self.processing_events.put(('boundary_ok',desired))
            except (OSError,ValueError,RuntimeError,EOFError) as exc:
                self.processing_events.put(('boundary_error',str(exc)))
        threading.Thread(target=change,daemon=True,name='boundary-display-control').start()

    def _apply_lane_preview(self,event=None):
        if self.lane_preview_pending or not self.connect_requested:
            return
        try:
            mode=self.lane_preview_modes[self.lane_preview_view.get()]
            if (mode in ('lane_bands','lane_bands_rgb','lane_compare') and event is not None and
                    getattr(event,'widget',None) is self.lane_preview_choice and
                    self.confirmed_lane_preview[0] not in ('lane_bands','lane_bands_rgb','lane_compare')):
                self.lane_preview_threshold.set(.05)
            threshold=round(float(self.lane_preview_threshold.get()),2)
            if not math.isfinite(threshold) or not 0<=threshold<=1:
                raise ValueError('차선 점수 임계값은 0~1 범위입니다')
        except (ValueError,KeyError,tk.TclError) as exc:
            self.lane_preview_status.set(f'입력 확인: {exc}')
            return
        self.lane_preview_threshold.set(threshold)
        self.lane_preview_pending=True
        self.lane_preview_apply.configure(state=tk.DISABLED)
        self.lane_preview_choice.configure(state=tk.DISABLED)
        self.lane_preview_scale.configure(state=tk.DISABLED)
        self.lane_preview_spin.configure(state=tk.DISABLED)
        self.lane_preview_status.set('차선 RGB 미리보기 적용 중...')
        ip=self.device_ip
        def change():
            try:
                set_lane_overlay(ip,mode,threshold)
                self.processing_events.put(('lane_preview_ok',(mode,threshold)))
            except (OSError,ValueError,RuntimeError,EOFError) as exc:
                self.processing_events.put(('lane_preview_error',str(exc)))
        threading.Thread(target=change,daemon=True,name='lane-preview-control').start()

    def _begin_seek(self, event=None):
        self.source_progress_dragging = str(self.source_progress.cget('state')) != 'disabled'

    def _preview_seek(self, value):
        if self.source_progress_dragging and self.file_sender:
            status = self.file_sender.snapshot()
            start, end = status.get('range_start_s',0), status.get('range_end_s')
            if end is not None:
                target = start+(end-start)*float(value)/100
                self.source_progress_text.set(f'이동 예정 {self._video_clock(target)}')

    def _finish_seek(self, event=None):
        dragging, self.source_progress_dragging = self.source_progress_dragging, False
        if not dragging or not self.file_sender:
            return
        status = self.file_sender.snapshot()
        start, end = status.get('range_start_s',0), status.get('range_end_s')
        if end is not None:
            try:
                self.source_progress_seek_id = self.file_sender.seek(
                    start+(end-start)*self.source_progress_value.get()/100)
                self.source_progress_text.set('재생 위치 이동 중...')
            except ValueError as exc:
                self.source_progress_text.set(str(exc))

    def _update_source_progress(self, status=None):
        if status is None:
            self.source_progress.configure(state=tk.DISABLED)
            self.source_progress_value.set(0.)
            self.source_progress_text.set('PC 영상 대기')
            return
        start, end = status.get('range_start_s',0), status.get('range_end_s')
        ready = end is not None and end > start
        active = status['state'] in ('전송 중','일시정지') and ready and not self.source_change_pending
        self.source_progress.configure(state=tk.NORMAL if active else tk.DISABLED)
        if self.source_progress_dragging:
            return
        if active and status.get('seek_applied',0) < self.source_progress_seek_id:
            return
        current = status['media_time_s']
        ratio = max(0,min(1,(current-start)/(end-start))) if ready else 0
        if status['state'] == '재생 완료' and ready:
            ratio, current = 1., end
        self.source_progress_value.set(ratio*100)
        end_text = self._video_clock(end) if end is not None else '끝 미상'
        self.source_progress_text.set(f'{self._video_clock(current)} / {end_text} · '
                                      f'{ratio:.0%} · {status.get("loop_count",1)}회차')

    def _choose_input_file(self):
        if self.recording:
            messagebox.showwarning('영상 소스', '녹화를 멈춘 뒤 영상 소스를 변경해 주세요.')
            return
        path = filedialog.askopenfilename(title='OAK로 전송할 영상', filetypes=[
            ('영상 파일', '*.mp4 *.avi *.mkv *.mov *.m4v'), ('모든 파일', '*.*')])
        if path:
            self.input_file.set(path)
            self.input_mode.set('PC 영상 파일')

    def _apply_input_source(self):
        if hasattr(self, 'six_camera_tab') and self.six_camera_tab.is_cam01_recording:
            messagebox.showwarning('영상 소스', '6대 녹화 탭의 녹화를 정지한 뒤 소스를 변경해 주세요.')
            return
        if self.recording:
            messagebox.showwarning('영상 소스', '녹화를 멈춘 뒤 영상 소스를 변경해 주세요.')
            return
        if self.source_change_pending:
            return
        if not self.connect_requested:
            messagebox.showwarning('영상 소스', '먼저 OAK 영상 연결을 완료해 주세요.')
            return
        mode = self.input_mode.get()
        path, loop = self.input_file.get(), self.input_loop.get()
        if mode != '카메라' and not Path(path).is_file():
            messagebox.showwarning('영상 소스', '전송할 영상 파일을 선택해 주세요.')
            return
        start_s, end_s = 0., None
        fps = 20
        if mode != '카메라':
            try:
                start_s = parse_video_time(self.input_start.get())
                fps = int(self.input_fps.get())
                if fps not in (10,15,20):
                    raise ValueError('전송 FPS는 10, 15, 20 중 선택해 주세요')
                end_s = parse_video_time(self.input_end.get()) if self.input_end.get().strip() else None
                if end_s is not None and end_s <= start_s:
                    raise ValueError('종료 시간은 시작 시간보다 커야 합니다')
            except ValueError as exc:
                messagebox.showwarning('전송 구간',str(exc))
                return
        try:
            video_input_settings.save(self.six_camera_tab.settings_path,
                dict(mode=mode, file=path, loop=loop, start=self.input_start.get(),
                     end=self.input_end.get(), fps=int(self.input_fps.get())))
        except (OSError, ValueError) as exc:
            messagebox.showerror('영상 입력 설정 저장', str(exc))
            return
        self.applied_input_settings = (mode, path, loop, start_s, end_s, fps)
        if mode != '카메라':
            self.source_start_button.configure(state=tk.NORMAL)
            self.source_status.set('영상 설정 적용 완료 · 메인 화면의 전송 시작을 누르세요'
                                   + (' · 기존 전송은 유지됩니다' if self.file_sender else ''))
            return
        self.source_start_button.configure(state=tk.DISABLED)
        self._switch_input_source(self.applied_input_settings)

    def _start_input_transfer(self):
        if hasattr(self, 'six_camera_tab') and self.six_camera_tab.is_cam01_recording:
            messagebox.showwarning('영상 소스', '6대 녹화 탭의 녹화를 정지한 뒤 소스를 변경해 주세요.')
            return
        if self.recording:
            messagebox.showwarning('영상 소스', '녹화를 멈춘 뒤 영상 소스를 변경해 주세요.')
            return
        if self.source_change_pending:
            return
        if not self.connect_requested:
            messagebox.showwarning('영상 소스', '먼저 OAK 영상 연결을 완료해 주세요.')
            return
        if not self.applied_input_settings or self.applied_input_settings[0] == '카메라':
            messagebox.showwarning('영상 소스', '영상 입력에서 PC 영상 파일을 선택하고 소스를 적용해 주세요.')
            return
        if not Path(self.applied_input_settings[1]).is_file():
            messagebox.showwarning('영상 소스', '적용된 영상 파일을 찾을 수 없습니다. 소스를 다시 적용해 주세요.')
            return
        self._switch_input_source(self.applied_input_settings)

    def _switch_input_source(self, settings):
        mode, path, loop, start_s, end_s, fps = settings
        self.source_change_pending = True
        self.source_progress_dragging = False
        self.source_progress_seek_id = 0
        self.source_progress.configure(state=tk.DISABLED)
        self.source_apply.configure(state=tk.DISABLED)
        self.source_start_button.configure(state=tk.DISABLED)
        self.source_status.set('장치 영상 소스 전환 중...')
        old_sender, self.file_sender = self.file_sender, None
        ip = self.device_ip

        def change():
            try:
                if old_sender:
                    old_sender.close(timeout=5)
                    if old_sender.thread.is_alive():
                        raise RuntimeError('이전 영상 전송이 아직 종료되지 않았습니다')
                if self.source_closing.is_set():
                    return
                if mode == '카메라':
                    select_camera(ip)
                    self.source_events.put(('camera', None))
                else:
                    sender = VideoFileSender(ip, path, loop=loop, fps=fps, start_s=start_s, end_s=end_s)
                    sender.start()
                    if self.source_closing.is_set():
                        sender.close()
                        return
                    self.source_events.put(('sender', sender))
            except (OSError, RuntimeError, ValueError) as exc:
                self.source_events.put(('error', str(exc)))
        threading.Thread(target=change, daemon=True, name='source-control').start()

    def _pause_input(self):
        if self.file_sender:
            self.file_sender.toggle_pause()

    def _stop_input(self):
        if self.file_sender:
            self.file_sender.close(timeout=.1)
            self.source_pause.configure(state=tk.DISABLED)

    def _update_input_status(self):
        while True:
            try:event,data=self.processing_events.get_nowait()
            except queue.Empty:break
            if event in ('lane_preview_ok','lane_preview_error'):
                self.lane_preview_pending=False
                self.lane_preview_apply.configure(state=tk.NORMAL)
                self.lane_preview_choice.configure(state='readonly')
                self.lane_preview_scale.configure(state=tk.NORMAL)
                self.lane_preview_spin.configure(state=tk.NORMAL)
                if event=='lane_preview_ok':
                    self.confirmed_lane_preview=data
                    self._save_preferences()
                    color_note=' · 마스크 픽셀별 점수색: 낮음 보라→높음 빨강' if data[0]=='lane_score' else ''
                    if data[0] in ('lane_bands','lane_bands_rgb','lane_compare'):
                        color_note=' · 참고 구간색: 낮음 빨강→높음 보라 · 최소 표시 0.05'
                    self.lane_preview_status.set(f'미리보기 임계값 {data[1]:.2f} · 인식용 0.50 유지'+color_note)
                    if data[0]=='raw_rgb':
                        self.lane_preview_status.set('원본 RGB · 표시 임계값 미적용 · 인식용 0.50 유지')
                else:
                    mode,threshold=self.confirmed_lane_preview
                    self.lane_preview_view.set(next(k for k,v in self.lane_preview_modes.items() if v==mode))
                    self.lane_preview_threshold.set(threshold)
                    self.lane_preview_status.set(f'미리보기 적용 실패: {data}')
                continue
            if event in ('boundary_ok','boundary_error'):
                self.boundary_display_pending=False
                self.boundary_display_check.configure(state=tk.NORMAL)
                if event=='boundary_ok':
                    self.confirmed_boundary_visible=data
                    self._save_preferences()
                    self.boundary_visible.set(data)
                    self.boundary_display_status.set('경계선 표시 켜짐' if data else '경계선 숨김 · 계산·추종선 유지')
                else:
                    self.boundary_visible.set(self.confirmed_boundary_visible)
                    self.boundary_display_status.set(f'표시 적용 실패: {data}')
                continue
            self.processing_pending=False
            self.processing_check.configure(state=tk.NORMAL)
            if event=='ok':
                self.confirmed_full_processing=data
                self._save_preferences()
                self.full_processing.set(data)
                self.processing_status.set('전체 후처리 · 이후 입력부터 적용' if data else '기본 인식 · 이후 입력부터 적용')
            else:
                self.full_processing.set(self.confirmed_full_processing)
                self.processing_status.set(f'모드 적용 실패: {data}')
        while True:
            try:
                event, data = self.source_events.get_nowait()
            except queue.Empty:
                break
            self.source_change_pending = False
            self.source_apply.configure(state=tk.NORMAL)
            self.source_start_button.configure(state=tk.NORMAL if self.applied_input_settings
                and self.applied_input_settings[0] != '카메라' else tk.DISABLED)
            if event == 'camera':
                self.active_input = 'camera'
                self.source_status.set('카메라 입력 · 실제 RGB 정렬 Depth 사용')
                self.source_pause.configure(state=tk.DISABLED)
            elif event == 'sender':
                self.file_sender = data
                self.source_pause.configure(state=tk.NORMAL)
            else:
                self.source_status.set(f'입력 소스 오류: {data} · 장치 TCP 8558 수신 코드 확인')
        if self.file_sender:
            status = self.file_sender.snapshot()
            if status.get('source_selected'):
                self.active_input = 'pc'
            detail = status.get('error') or (f"보낸 프레임 {status['sent']} / "
                     f"장치 수신 누적 {status['acknowledged']} / 재생 {status['media_time_s']:.1f}초")
            if not status.get('error') and status.get('range_start_s') is not None:
                end = status.get('range_end_s')
                detail += (f" · 구간 {status['range_start_s']:g}~"
                           f"{f'{end:g}초' if end is not None else '끝'} / 회차 {status.get('loop_count',1)}")
            if status.get('effective_fps') is not None:
                detail += (f"\n설정{status['requested_fps']:g}/전송상한{status['effective_fps']:g} FPS"
                           f" (원본{status['source_fps']:g}·장치{status['device_fps']:g})")
            self.source_status.set(f"PC 영상 {status['state']} · {detail} · Depth/거리 없음")
            if status['state'] in ('오류', '전송 중지', '재생 완료'):
                self.source_pause.configure(state=tk.DISABLED)
            self._update_source_progress(status)
        else:
            self._update_source_progress()

    def _build_preview_panel(
        self,
        parent: ttk.Frame,
        row: int,
        column: int,
        key: str,
        title: str,
        placeholder: str,
    ) -> None:
        panel = ttk.Frame(parent, style="Panel.TFrame", padding=8)
        panel.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(1, weight=1)

        title_bar = ttk.Frame(panel, style="Panel.TFrame")
        title_bar.grid(row=0, column=0, pady=(0, 7), sticky="ew")
        title_bar.columnconfigure(0, weight=1)
        ttk.Label(title_bar, text=title, style="PanelTitle.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        status = tk.StringVar(value="대기")
        ttk.Label(
            title_bar,
            textvariable=status,
            style="PanelStatus.TLabel",
        ).grid(row=0, column=1, sticky="e")

        preview = ttk.Label(
            panel,
            text=placeholder,
            style="Video.TLabel",
            anchor="center",
        )
        preview.grid(row=1, column=0, sticky="nsew")
        self.preview_labels[key] = preview
        self.preview_status[key] = status

    def _selected_keys(self) -> list[str]:
        return [key for key, selected in self.selected.items() if selected.get()]

    def save_connection_settings(self) -> None:
        ip = self.ip_text.get().strip()
        try:
            import ipaddress
            ipaddress.ip_address(ip)
            self.six_camera_tab.cameras[0]['ip'].set(ip)
            if not self.six_camera_tab.save_camera_settings():
                return
            if self.password_text.get():
                save_password(ip, self.password_text.get(), self.six_camera_tab.settings_path)
                self.password_text.set('')
        except (OSError, ValueError, subprocess.SubprocessError):
            messagebox.showerror('연결 정보 저장', 'IP 또는 암호화 저장소를 확인해 주세요. 비밀번호 저장에 실패했습니다.')
            return
        self.status_text.set(f'{ip} 연결 정보 저장 완료 · 다음 실행에서 사용')

    def check_camera_status(self) -> None:
        self._camera_app_action('status')

    def _poll_camera_app(self):
        if self.source_closing.is_set():
            return
        if self.ip_text.get().strip():
            self._camera_app_action('poll')
        self.root.after(5000, self._poll_camera_app)

    def _camera_app_action(self, action):
        if self.app_action_busy or getattr(self, 'connection_launch_pending', False):
            if action != 'poll':
                messagebox.showinfo('카메라 앱', '카메라 명령을 처리 중입니다. 잠시 기다려 주세요.')
            return
        ip = self.ip_text.get().strip()
        if not ip:
            messagebox.showwarning('카메라 상태', 'OAK IP를 입력해 주세요.')
            return
        if action == 'stop' and (self.recording or self.six_camera_tab.is_recording):
            messagebox.showwarning('카메라 앱', '녹화를 먼저 종료한 후 앱을 멈춰 주세요.')
            return
        password = self.password_text.get() or load_password(ip, self.six_camera_tab.settings_path)
        if action == 'start':
            if self.connect_requested and ip != self.device_ip:
                messagebox.showwarning('카메라 앱', '현재 연결된 장치의 IP를 사용해 주세요.')
                return
            self.six_camera_tab.cameras[0]['ip'].set(ip)
            if not self.six_camera_tab.save_camera_settings():
                return
            if self.password_text.get():
                try:
                    save_password(ip, password, self.six_camera_tab.settings_path)
                    self.password_text.set('')
                except (OSError, subprocess.SubprocessError):
                    messagebox.showwarning('비밀번호 저장', '암호화 저장에 실패했습니다. 이번 실행에만 사용합니다.')
        self.app_action_busy = True
        self.app_state_text.set('상태 확인 중' if action in ('status', 'poll') else '명령 처리 중')
        self.app_led.configure(fg='#8da0af')
        def control():
            result = dict(ip=ip, action=action)
            try:
                current = oak_app_control.status(OAKCTL_PATH, ip, password)
                message = '앱이 실행 중입니다.' if current['state'] == 'running' else '앱이 멈춰 있습니다.'
                if action == 'start':
                    if current['state'] == 'running':
                        message = '카메라 앱이 이미 실행 중입니다.'
                    else:
                        if not self._prepare_connection(ip, password, force_run=True):
                            raise RuntimeError('카메라 앱 실행 또는 영상 포트 확인에 실패했습니다.')
                        current = oak_app_control.status(OAKCTL_PATH, ip, password)
                        if current['state'] != 'running':
                            raise RuntimeError('카메라 앱이 실행 상태가 아닙니다.')
                        message = '카메라 앱을 실행했습니다.'
                elif action == 'stop':
                    if current['state'] == 'running':
                        oak_app_control.stop(OAKCTL_PATH, ip, current['app_id'], password)
                        current = oak_app_control.status(OAKCTL_PATH, ip, password)
                        if current['state'] != 'stopped':
                            raise RuntimeError('카메라 앱 중지가 아직 확인되지 않았습니다.')
                    message = '카메라 앱이 멈춰 있습니다.'
                result.update(state=current['state'], message=message)
            except Exception:
                result.update(state='unknown', error='카메라 앱 상태 또는 명령을 확인하지 못했습니다. IP·비밀번호·네트워크를 확인해 주세요.')
            self.connection_events.put(('app_result', result))
        threading.Thread(target=control, daemon=True, name='camera-app-control').start()

    def connect_video(self) -> None:
        if self.app_action_busy:
            self.status_text.set('카메라 앱 명령을 처리 중입니다.')
            return
        if self.connect_requested and not self.video_retry_available:
            self.status_text.set('카메라 앱 연결이 진행 중이거나 이미 연결되어 있습니다.')
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
        if hasattr(self,'six_camera_tab'):
            self.six_camera_tab.cameras[0]['ip'].set(entered_ip)
            self.six_camera_tab.cameras[0]['serial'].set(self.device_serial)
            self.six_camera_tab.save_camera_settings(notify=False)
        password = self.password_text.get()
        if password:
            try:
                save_password(self.device_ip, password, self.six_camera_tab.settings_path)
            except (OSError, subprocess.SubprocessError) as exc:
                messagebox.showwarning(
                    "비밀번호 저장",
                    f"비밀번호를 안전하게 저장하지 못했습니다.\n{exc}",
                )
        else:
            password = load_password(self.device_ip, self.six_camera_tab.settings_path)
        self.password_text.set("")
        self.connect_requested = True
        self.video_retry_available = False
        self.connection_launch_pending = True
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
        for port in (8553, 8554, 8555, 8556):
            try:
                with socket.create_connection((ip, port), timeout=1.0):
                    pass
            except OSError:
                return False
        return True

    def _prepare_connection(self, ip: str, password: str, force_run=False) -> bool:
        if not force_run and self._rtsp_ports_open(ip):
            self.connection_events.put(("ready", ip))
            return True
        if not OAKCTL_PATH.exists():
            self.connection_events.put(("error", f"oakctl을 찾을 수 없습니다: {OAKCTL_PATH}"))
            return False

        command = [str(OAKCTL_PATH)]
        if password:
            command.extend(["--password", password])
        app_source = str(OAK_APP_PATH)
        if not IS_WINDOWS and OAKCTL_PATH.suffix.lower() == '.exe':
            try:
                app_source = subprocess.check_output(
                    ['wslpath', '-w', app_source], text=True, timeout=5).strip()
            except (OSError, subprocess.SubprocessError) as exc:
                self.connection_events.put(('error', f'WSL 경로 변환 실패: {exc}'))
                return False
        command.extend(
            [
                "app",
                "run",
                app_source,
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
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.connection_events.put(("error", 'OAK 앱 실행 명령에 실패했습니다. 연결·인증을 확인해 주세요.'))
            return False
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout or "OAK 앱 실행 실패").strip()
            self.connection_events.put(("error", detail[-1000:]))
            return False

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if self._rtsp_ports_open(ip):
                self.connection_events.put(("ready", ip))
                return True
            time.sleep(1.0)
        self.connection_events.put(
            (
                "error",
                "OAK 앱 실행 명령은 완료됐지만 60초 안에 RGB/YOLO/BEV/Depth 영상 포트가 열리지 않았습니다. "
                "장비 연결과 앱 상태를 확인해 주세요.",
            )
        )
        return False

    def _begin_receivers(self, ip: str) -> None:
        self.video_wait_started_at = time.monotonic()
        if hasattr(self, 'six_camera_tab') and not self.six_camera_tab.settings_reset_active:
            self.six_camera_tab.cameras[0]['ip'].set(ip)
        self.sources["rgb"].url = self.rgb_url_override or f"rtsp://{ip}:8555/rgb"
        self.sources["yolopv2"].url = (
            self.result_url_override or f"rtsp://{ip}:8554/yolopv2"
        )
        self.sources["bev"].url = self.bev_url_override or f"rtsp://{ip}:8553/bev"
        self.sources["depth"].url = f"rtsp://{ip}:8556/depth"
        for source in self.sources.values():
            source.start()
        self.connect_button.configure(text="영상 연결 중...", state=tk.DISABLED)
        self.status_text.set(f"{ip} RGB, YOLO, BEV, Depth 스트림 연결 중...")
        if hasattr(self, 'source_apply'):
            self.source_apply.configure(state=tk.NORMAL)
        if hasattr(self,'processing_check'):
            self.processing_check.configure(state=tk.NORMAL)
            self._apply_processing_mode()
        if hasattr(self,'boundary_display_check'):
            self.boundary_display_check.configure(state=tk.NORMAL)
            self._apply_boundary_display()
        if hasattr(self,'lane_preview_apply'):
            self.lane_preview_apply.configure(state=tk.NORMAL)
            self._apply_lane_preview()

    def _drain_connection_events(self) -> None:
        while True:
            try:
                event, detail = self.connection_events.get_nowait()
            except queue.Empty:
                return
            if event == 'app_result':
                self.app_action_busy = False
                if detail['ip'] != self.ip_text.get().strip():
                    self.app_led.configure(fg='#8da0af')
                    self.app_state_text.set('상태 미확인')
                    continue
                state = detail['state']
                self.app_led.configure(fg={'running':'#27cf73', 'stopped':'#ed5353'}.get(state, '#8da0af'))
                self.app_state_text.set({'running':'앱 실행 중', 'stopped':'앱 멈춤'}.get(state, '상태 확인 불가'))
                if detail['action'] != 'poll':
                    if 'error' in detail:
                        self.status_text.set(detail['error'])
                        messagebox.showerror('카메라 앱', detail['error'])
                    else:
                        self.status_text.set(detail['ip'] + ' ' + detail['message'])
                        messagebox.showinfo('카메라 앱', detail['message'])
                if state == 'stopped' and detail['action'] == 'stop':
                    self.connect_requested = False
                    self.connect_button.configure(text='영상 연결', state=tk.NORMAL)
                    self.ip_entry.configure(state=tk.NORMAL)
                    self.password_entry.configure(state=tk.NORMAL)
                    self.discover_button.configure(state=tk.NORMAL)
                continue
            if event == 'status':
                self.status_text.set(detail.replace('\n', ' · '))
                messagebox.showinfo('카메라 상태 확인', detail)
                continue
            if event == "ready":
                self.connection_launch_pending = False
                self.device_ip = detail
                self.connect_requested = True
                self._begin_receivers(detail)
                continue

            self.connect_requested = False
            self.connection_launch_pending = False
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
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
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
        self._update_input_status()
        if self.csv_logger is not None:
            log = self.csv_logger
            self.csv_status.set(f'CSV {log.status} | 프레임 {log.frames:,}건 / 객체 {log.objects:,}건 '
                                f'/ 누락 {log.gaps:,} | {log.path}')
        self._drain_connection_events()
        if self.recording:
            failures=[self.sources[key].record_error for key in self.recording_keys if self.sources[key].record_error]
            if failures:
                self.stop_recording()
                self.status_text.set('녹화 쓰기 오류: '+' / '.join(failures))
        result_connected, result_frame = self.sources["yolopv2"].snapshot()
        rgb_connected, rgb_frame = self.sources["rgb"].snapshot()
        if hasattr(self, 'six_camera_tab'):
            self.six_camera_tab.update_preview(rgb_connected, rgb_frame)
        bev_connected, bev_frame = self.sources["bev"].snapshot()
        depth_connected, depth_frame = self.sources["depth"].snapshot()
        preview_frames = {
            "rgb": (rgb_connected, rgb_frame),
            "yolopv2": (result_connected, result_frame),
            "bev": (bev_connected, bev_frame),
            "depth": (depth_connected, depth_frame),
        }
        for key, (connected, frame) in preview_frames.items():
            preview = self.preview_labels[key]
            if key == 'depth' and self.active_input == 'pc':
                preview.configure(image='', text='PC 영상 입력 · 실제 Depth/거리 없음')
                preview.image = None
                self.preview_status[key].set('사용 안 함')
                continue
            if not preview.winfo_ismapped():
                self.preview_status[key].set('연결됨' if connected else '연결 대기')
                continue
            if frame is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(rgb)
                # Fit the actual panel: fixed thumbnail dimensions clipped the
                # near boundary anchors in smaller multiview windows.
                available = (max(1,min(PREVIEW_SIZE[0],preview.winfo_width())),
                             max(1,min(PREVIEW_SIZE[1],preview.winfo_height())))
                image.thumbnail(available, RESAMPLE_LANCZOS)
                photo = ImageTk.PhotoImage(image)
                preview.configure(image=photo, text="")
                preview.image = photo
            elif not connected:
                preview.configure(image="", text="영상 연결 대기")
                preview.image = None
            self.preview_status[key].set("연결됨" if connected else "연결 대기")

        self._update_video_connection_ui(preview_frames)
        connection_state = (rgb_connected, result_connected, bev_connected, depth_connected)
        if any(connection_state):
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
                f"BEV {'연결됨' if bev_connected else '연결 중'}",
                f"Depth {'연결됨' if depth_connected else '연결 중'}",
            ]
            self.status_text.set(" / ".join(states))

        self.root.after(50, self._update_preview)

    def _update_video_connection_ui(self, preview_frames):
        """A listening RTSP port alone is not a successfully received video."""
        if not self.connect_requested or getattr(self, 'connection_launch_pending', False):
            return
        now = time.monotonic()
        received = [key for key, (connected, frame) in preview_frames.items()
                    if connected and frame is not None
                    and now - self.sources[key].last_frame_at < 5]
        expected = ['rgb', 'yolopv2', 'bev']
        if self.active_input != 'pc':
            expected.append('depth')
        if all(key in received for key in expected):
            self.video_wait_started_at = None
            self.video_retry_available = False
            self.connect_button.configure(text='영상 연결됨', state=tk.DISABLED)
            return
        if self.video_wait_started_at is None:
            self.video_wait_started_at = now
        if now - self.video_wait_started_at < 20:
            self.connect_button.configure(text='일부 연결 중...' if received else '영상 연결 중...', state=tk.DISABLED)
            return
        # Receivers continue retrying in the background. Let the user correct
        # the IP or retry; do not discard streams or interrupt any recordings.
        self.video_retry_available = True
        self.connect_button.configure(text='다시 연결', state=tk.NORMAL)
        self.ip_entry.configure(state=tk.NORMAL)
        self.password_entry.configure(state=tk.NORMAL)
        self.discover_button.configure(state=tk.NORMAL)
        missing = ', '.join(key.upper() for key in expected if key not in received)
        self.status_text.set(f'20초 동안 영상 수신 확인 실패: {missing}. IP·카메라 앱·영상 입력 소스를 확인하고 다시 연결해 주세요.')

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
        if self.recording:
            return
        if getattr(self, 'source_change_pending', False):
            messagebox.showwarning('녹화 시작', '영상 소스 적용이 끝난 뒤 녹화를 시작해 주세요.')
            return
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
        # A preview connection must not create logs. Each successful recording
        # gets its own logger/session; a stopped logger cannot be restarted.
        self.csv_logger = FrameCsvLogger(
            self.device_ip, Path(__file__).resolve().parents[1] / 'artifacts' / 'telemetry')
        self.csv_logger.start()
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
        if self.csv_logger is not None:
            self.csv_logger.close()
        self.recording = False
        self.recording_keys = []
        # Finalize video beside this recording's CSV files. On a filesystem
        # failure retain the temporary paths for retry/manual export.
        save_error = None
        session_path = self.csv_logger.path
        try:
            if self.unsaved_paths:
                session_path.mkdir(parents=True, exist_ok=True)
            for key, source in list(self.unsaved_paths.items()):
                destination = session_path / f'oak4dpro_{self.recording_stamp}_{key}.mp4'
                if destination.exists():
                    raise FileExistsError(f'이미 존재하는 파일: {destination}')
                shutil.move(str(source), str(destination))
                self.unsaved_paths.pop(key)
        except OSError as exc:
            save_error = str(exc)
        self.start_button.configure(state=tk.NORMAL)
        self.stop_button.configure(state=tk.DISABLED)
        self.save_button.configure(state=tk.NORMAL if self.unsaved_paths else tk.DISABLED)
        self.rgb_check.configure(state=tk.NORMAL)
        self.result_check.configure(state=tk.NORMAL)
        if save_error:
            self.status_text.set('자동 저장 실패 — 임시 영상 유지, 파일 저장 버튼으로 저장 가능')
            messagebox.showerror('녹화 자동 저장', save_error)
        else:
            self.status_text.set(f'녹화 종료 · 영상/로그 저장 위치: {session_path}')
        recording_errors=[self.sources[key].record_error for key in self.sources if self.sources[key].record_error]
        if recording_errors:
            self.status_text.set('녹화 쓰기/마무리 오류 · '+' / '.join(recording_errors))

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
        if getattr(self, 'source_closing', None):
            if hasattr(self, 'six_camera_tab') and not self.six_camera_tab.close():
                messagebox.showwarning('녹화 저장', '녹화 파일 또는 수신기를 마무리 중입니다. 잠시 후 다시 닫아 주세요.')
                return
            self.six_camera_tab.save_camera_settings(notify=False)
            self._save_preferences()
            self.source_closing.set()
        if getattr(self, 'file_sender', None):
            self.file_sender.close()
        for source in self.sources.values():
            source.stop_event.set()
        for source in self.sources.values():
            source.close()
        if self.csv_logger is not None:
            self.csv_logger.close()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OAK4-D Pro dual MP4 recorder")
    parser.add_argument("--device-ip", default=DEFAULT_DEVICE_IP)
    parser.add_argument("--device-serial", default=DEFAULT_DEVICE_SERIAL)
    parser.add_argument("--rgb-url", help="override the RGB RTSP URL")
    parser.add_argument("--result-url", help="override the YOLOPv2 RTSP URL")
    parser.add_argument("--bev-url", help="override the virtual boundary BEV RTSP URL")
    parser.add_argument("--auto-connect", action="store_true", help="connect after opening the window")
    parser.add_argument('--camera-settings',type=Path,help='camera registration JSON path')
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
    app = RecorderApp(
        root,
        args.device_ip,
        args.device_serial,
        args.rgb_url,
        args.result_url,
        args.bev_url,
        args.output_dir,
        camera_settings_path=args.camera_settings,
        use_saved_connection=not any(arg in ('--device-ip','--device-serial') or
            arg.startswith(('--device-ip=','--device-serial=')) for arg in sys.argv[1:]),
    )
    if args.auto_connect:
        root.after(200, app.connect_video)
    root.mainloop()


if __name__ == "__main__":
    main()
