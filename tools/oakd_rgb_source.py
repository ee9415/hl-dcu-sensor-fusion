"""OAK-D receiver with isolated DepthAI runtime and explicit virtual mode."""
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import cv2
import numpy as np


class OakdRgbSource:
    def __init__(self, name, ip='', mxid='', python=None, virtual=False,
                 width=1280, height=720, fps=10, retry_s=2, fail_after=0):
        self.name, self.ip, self.mxid = name, ip, mxid
        self.python, self.virtual = python or sys.executable, virtual
        self.width, self.height, self.fps = width, height, fps
        self.retry_s, self.fail_after = retry_s, fail_after
        self.url = ('virtual://' if virtual else 'depthai2://')+(name if virtual else ip or mxid)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.process = None
        self.frame = None
        self.connected = False
        self.sequence = 0
        self.received_at = 0.
        self.error = ''
        self.reconnects = 0
        self.actual_mxid = ''
        self.diagnostic_path = None
        self.last_failure = {}
        self.diagnostic_error = ''

    def start(self):
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self._run, name=self.name+'-receiver', daemon=True)
        self.thread.start()

    def snapshot(self):
        connected, frame, _, _ = self.record_snapshot()
        return connected, frame

    def record_snapshot(self):
        with self.lock:
            connected = self.connected and time.monotonic()-self.received_at < 3
            return connected, None if self.frame is None else self.frame.copy(), self.sequence, self.received_at

    def info(self):
        with self.lock:
            return dict(name=self.name, virtual=self.virtual, error=self.error,
                        reconnects=self.reconnects, mxid=self.actual_mxid or self.mxid,
                        frames_received=self.sequence,frame_age_s=round(time.monotonic()-self.received_at,3),
                        last_failure=dict(self.last_failure),diagnostic_error=self.diagnostic_error)

    def _diagnostic(self,event,**details):
        if self.diagnostic_path is None:return
        try:
            path=Path(self.diagnostic_path);path.parent.mkdir(parents=True,exist_ok=True)
            with path.open('a',encoding='utf-8') as stream:
                stream.write(json.dumps(dict(event=event,wall_time=time.time(),camera=self.name,
                    ip=self.ip,mxid=self.mxid,virtual=self.virtual,**details),ensure_ascii=False)+'\n')
        except OSError as exc:
            self.diagnostic_error=str(exc)

    def guard(self, _ip):
        with self.lock:
            if not self.connected or time.monotonic()-self.received_at >= 3:
                raise RuntimeError(self.name+' RGB 연결이 없습니다: '+self.error)
            return dict(source='virtual' if self.virtual else 'camera',
                        device_ip=self.ip, mxid=self.actual_mxid or self.mxid, reconnects=self.reconnects)

    def close(self, timeout=6):
        self.stop_event.set()
        with self.lock:
            process = self.process
        if process and process.poll() is None:
            process.terminate()
        if self.thread:
            self.thread.join(timeout)
        return self.thread is None or not self.thread.is_alive()

    def _run(self):
        while not self.stop_event.is_set():
            process = None
            diagnostics = []
            read_errors = None
            last_activity = [time.monotonic()]
            done = threading.Event()
            watchdog_reason = [None]
            try:
                command = [self.python, '-u', str(Path(__file__).with_name('oakd_rgb_worker.py')),
                           '--name', self.name, '--target', self.ip or self.mxid,
                           '--mxid', self.mxid, '--width', str(self.width), '--height', str(self.height),
                           '--fps', str(self.fps)]
                if self.virtual:
                    command += ['--virtual']
                if self.fail_after:
                    command += ['--fail-after', str(self.fail_after)]
                self._diagnostic('worker_start',attempt=self.reconnects)
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                with self.lock:
                    self.process = process
                def stderr_reader(process=process, diagnostics=diagnostics):
                    for line in iter(process.stderr.readline, b''):
                        diagnostics.append(line.decode('utf-8', errors='replace').strip())
                        del diagnostics[:-8]
                def watchdog(process=process, done=done, last_activity=last_activity,reason=watchdog_reason):
                    # A child stuck in discovery/read cannot freeze close or retries.
                    while not done.wait(.2):
                        if self.stop_event.is_set() or time.monotonic()-last_activity[0] > 20:
                            if not self.stop_event.is_set():
                                reason[0]='receiver_watchdog_timeout'
                            if process.poll() is None:
                                process.terminate()
                                try:
                                    process.wait(timeout=2)
                                except subprocess.TimeoutExpired:
                                    process.kill()
                            return
                read_errors = threading.Thread(target=stderr_reader, daemon=True)
                monitor = threading.Thread(target=watchdog, daemon=True)
                read_errors.start(); monitor.start()
                while not self.stop_event.is_set():
                    line = process.stdout.readline(4097)
                    if not line:
                        raise RuntimeError('worker 종료')
                    if not line.startswith(b'OAKRGB '):
                        continue
                    if len(line)>4096 or not line.endswith(b'\n'):
                        raise ValueError('잘못된 RGB 헤더')
                    header = json.loads(line[7:])
                    if not self.virtual and self.mxid and header.get('mxid') != self.mxid:
                        raise ValueError('수신한 MX ID가 설정과 다릅니다.')
                    length = header['length']
                    if type(length) is not int or not 0 < length <= 4*1024*1024:
                        raise ValueError('RGB packet size invalid')
                    data = process.stdout.read(length)
                    if len(data) != length:
                        raise RuntimeError('RGB packet truncated')
                    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        raise ValueError('RGB JPEG decode failed')
                    last_activity[0] = time.monotonic()
                    first_packet = False
                    with self.lock:
                        first_packet = not self.connected
                        self.frame = frame
                        self.sequence += 1
                        self.received_at = last_activity[0]
                        self.connected = True
                        self.error = ''
                        self.actual_mxid = str(header.get('mxid',''))
                    if first_packet:self._diagnostic('rgb_connected',actual_mxid=self.actual_mxid)
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
            finally:
                done.set()
                cleanup_terminated=False
                if process:
                    if process.poll() is None and not self.stop_event.is_set():
                        # EOF can arrive just before normal worker teardown.
                        # Preserve its real exit code instead of replacing it
                        # with the signal from our cleanup terminate call.
                        try:process.wait(timeout=.5)
                        except subprocess.TimeoutExpired:pass
                    if process.poll() is None:
                        cleanup_terminated=True
                        process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait(timeout=2)
                    if read_errors:
                        read_errors.join(timeout=1)
                    process.stdout.close(); process.stderr.close()
                with self.lock:
                    host_error=self.error
                    if diagnostics:
                        detail=' / '.join(diagnostics)[-700:]
                        self.error=(host_error+' · '+detail) if host_error else detail
                    self.connected = False
                    self.process = None
                    if not self.stop_event.is_set():
                        self.last_failure=dict(wall_time=time.time(),error=self.error,
                            host_error=host_error,worker_stderr=list(diagnostics),
                            cleanup_terminated=cleanup_terminated,
                            exit_code=None if process is None else process.returncode,
                            kind=watchdog_reason[0] or 'worker_exit',
                            last_packet_age_s=round(time.monotonic()-last_activity[0],3))
                self._diagnostic('stopped' if self.stop_event.is_set() else 'worker_failed',
                                 failure=dict(self.last_failure),stderr=diagnostics)
            if not self.stop_event.is_set():
                with self.lock:
                    self.reconnects += 1
                self.stop_event.wait(self.retry_s)
