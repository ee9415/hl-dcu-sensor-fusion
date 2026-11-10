"""Independent CAM01 RGB receiver; never starts the perception application."""
# CAM01(OAK4)의 RGB 전용 앱 영상을 받는 수신기. 인지(AI) 앱은 시작하지 않는다.
# 설치된 RGB 앱으로 전환 → 상태 확인(:8561/status) → RTSP(:8560/rgb) 수신 순서로 동작한다.
import json
import threading
import time
from urllib.request import ProxyHandler, Request, build_opener
import oak_app_control
from console_log import event, register_secret, progress_task

IDENTIFIER = 'com.qsentech.oak4-rgb-only'


def rgb_status(ip, mxid='', timeout=2):
    """RGB 앱의 /status를 조회해 '순수 RGB·카메라 입력·AI 꺼짐·epoch 있음'인지 확인한다. mxid를 주면 장치 ID도 확인한다."""
    request = Request(f'http://{ip}:8561/status')
    with build_opener(ProxyHandler({})).open(request, timeout=timeout) as response:
        result = json.loads(response.read(65536))
    if (result.get('ok') is not True or result.get('mode') != 'rgb_only'
            or result.get('source') != 'camera' or result.get('ai_enabled') is not False
            or not result.get('source_epoch')):
        raise RuntimeError('CAM01 순수 RGB 전송 상태가 확인되지 않았습니다.')
    if mxid and result.get('mxid') != mxid:
        raise RuntimeError('CAM01 장치 ID가 설정과 다릅니다.')
    return result


def wait_rgb_ready(cli, ip, password, stop_event, update, build_timeout=120, warmup_timeout=60):
    """RGB 앱이 첫 프레임을 낼 때까지 기다린다.
    시작 중(preparing)은 build_timeout까지, 실행 중인데 프레임이 없으면 warmup_timeout까지 기다리고, 앱이 멈추면 오류를 낸다.
    """
    build_deadline = time.monotonic()+build_timeout
    warmup_deadline = None
    while not stop_event.is_set():
        try:
            rgb_status(ip)
            update('RGB 카메라 준비 완료')
            return
        except (OSError, ValueError, RuntimeError):
            pass
        current = oak_app_control.status(cli, ip, password, identifier=IDENTIFIER, allow_preparing=True, installed_only=True)
        now = time.monotonic()
        if current['state'] == 'preparing':
            warmup_deadline = None
            update('RGB 프로그램 시작 중 · 완료까지 기다리는 중')
        elif current['state'] == 'running':
            if warmup_deadline is None:
                warmup_deadline = now+warmup_timeout
            update('RGB 프로그램 실행 중 · 첫 카메라 프레임 준비 중')
            if now >= warmup_deadline:
                raise RuntimeError('RGB 프로그램은 실행됐지만 60초 안에 카메라 프레임이 나오지 않았습니다.')
        elif current.get('detail') in ('exited', 'stopped'):
            raise RuntimeError('RGB 프로그램이 실행 도중 중지됐습니다. 장치 실행 로그를 확인해 주세요.')
        else:
            update('RGB 프로그램 등록·시작 대기 중')
        if now >= build_deadline:
            raise RuntimeError('RGB 프로그램 시작 대기 시간이 초과되었습니다.')
        if stop_event.wait(1):
            return


@progress_task('CAM01 RGB 연결 준비')
def prepare_rgb(cli, ip, password, stop_event, progress=None):
    """CAM01을 RGB 전용 앱으로 전환하고 준비될 때까지 기다린다. 진행 상황은 콘솔과 progress 콜백으로 알린다."""
    register_secret(password)
    last_message = None
    def update(message):
        """같은 메시지는 한 번만 콘솔에 남기고, progress 콜백에는 매번 전달한다."""
        nonlocal last_message
        if message != last_message:
            event('CAM01 · '+message)
            last_message = message
        if progress:
            progress(message)
    update('설치된 RGB 앱 상태 확인 중')
    if stop_event.is_set():
        return
    app_id = oak_app_control.start_installed(cli, ip, IDENTIFIER, password, stop_event=stop_event)
    if app_id is None:
        return
    update('설치된 RGB 앱 시작 · 첫 카메라 프레임 준비 중')
    wait_rgb_ready(cli, ip, password, stop_event, update)


class Oak4RgbSource:
    """CAM01 RGB 수신기. 백그라운드에서 앱 전환·준비를 마친 뒤 공유 RTSP 수신기(receiver)를 시작한다."""
    def __init__(self, ip, mxid, receiver_factory, cli, password=''):
        self.ip, self.mxid, self.virtual = ip, mxid, False
        self.url = f'rtsp://{ip}:8560/rgb'
        self.receiver = receiver_factory('cam01-raw', 'CAM01 RGB', self.url)
        self.cli, self.password = cli, password
        self.stop_event = threading.Event()
        self.thread = None
        self.error = ''
        self.preparing = False
        self.started_at = 0.
        self.prepare_status = ''

    def start(self):
        """준비 스레드를 시작한다(UI를 막지 않음)."""
        self.preparing = True
        self.thread = threading.Thread(target=self._start, daemon=True, name='cam01-rgb-prepare')
        self.thread.start()

    def _start(self):
        """앱 준비 → 상태 확인 → 수신기 시작. 실패 시 비밀번호를 가린 오류 메시지를 남기고, 끝나면 비밀번호를 메모리에서 지운다."""
        try:
            prepare_rgb(self.cli, self.ip, self.password, self.stop_event,
                        progress=lambda message: setattr(self, 'prepare_status', message))
            if not self.stop_event.is_set():
                self.guard(self.ip)
                self.started_at = time.monotonic()
                self.receiver.start()
                self.prepare_status = ''
        except Exception as exc:
            # Do not expose a CLI command line containing the camera password.
            detail = str(exc) if isinstance(exc, RuntimeError) else type(exc).__name__
            if self.password:
                detail = detail.replace(self.password, '[비공개]')
            self.error = 'CAM01 RGB 연결 실패 · '+detail
            event(self.error, 'ERROR')
        finally:
            self.password = ''
            self.preparing = False

    def guard(self, _ip):
        """녹화 중 주기적으로 호출되어 RGB 앱 상태가 여전히 올바른지 확인한다."""
        return rgb_status(self.ip, self.mxid)

    def record_snapshot(self):
        """수신기의 최신 프레임을 반환한다. 3초 넘게 새 프레임이 없으면 연결 안 됨으로 본다."""
        ok, frame, sequence, stamp = self.receiver.record_snapshot()
        return ok and not self.stop_event.is_set() and time.monotonic()-stamp < 3, frame, sequence, stamp

    def snapshot(self):
        """(연결 여부, 프레임)만 반환하는 미리보기용 조회."""
        ok, frame, _, _ = self.record_snapshot()
        return ok, frame

    def info(self):
        """수신 상태 정보에 RGB 모드·준비 상태·수신 FPS를 더해 반환한다."""
        result = self.receiver.info()
        elapsed = time.monotonic()-self.started_at if self.started_at else 0.
        return dict(result, mode='rgb_only', ai_enabled=False, virtual=False,
                    mxid=self.mxid, error=self.error, preparing=self.preparing, prepare_status=self.prepare_status,
                    receive_fps=round(result['frames_received']/elapsed, 2) if elapsed else 0.)

    def close(self, timeout=6):
        """준비 스레드와 수신 스레드를 멈추고 timeout 안에 모두 끝났는지 반환한다."""
        self.stop_event.set()
        self.receiver.stop_event.set()
        deadline = time.monotonic()+timeout
        for thread in (self.thread, self.receiver.thread):
            if thread:
                thread.join(max(0., deadline-time.monotonic()))
        return not any(t and t.is_alive() for t in (self.thread, self.receiver.thread))
