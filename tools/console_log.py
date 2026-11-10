"""Compact console events; no frame logs, callback arguments or tracebacks."""
# 콘솔에 짧은 이벤트 로그를 남기는 공통 모듈.
# 프레임 단위 로그, 함수 인자, traceback은 출력하지 않고, 비밀번호 등 민감 정보는 [비공개]로 가린다.
from datetime import datetime
from functools import wraps
import re
import threading
import time

_lock = threading.RLock()
_secrets = set()
_progress_local = threading.local()


def progress_task(label, interval=10):
    """Report long commands without blocking the UI or duplicating nested tasks."""
    # 오래 걸리는 작업에 '처리 중 · N초 경과' 로그를 주기적으로 남기는 데코레이터.
    # 중첩 호출은 바깥 작업만 보고하고, 진행 로그가 한 번이라도 나갔으면 종료/오류 로그도 남긴다.
    def decorate(function):
        @wraps(function)
        def call(*args, **kwargs):
            if getattr(_progress_local, 'active', False):
                return function(*args, **kwargs)
            message = label(*args, **kwargs) if callable(label) else label
            if message is None:
                return function(*args, **kwargs)
            done = threading.Event()
            began = time.monotonic()
            reported = [False]
            failed = False
            def report():
                """작업이 끝날 때까지 interval초마다 진행 로그를 남긴다."""
                while not done.wait(interval):
                    if done.is_set():
                        return
                    reported[0] = True
                    event(f'{message} · 처리 중 · {int(time.monotonic()-began)}초 경과')
            reporter = threading.Thread(target=report, daemon=True, name='command-progress')
            _progress_local.active = True
            reporter.start()
            try:
                return function(*args, **kwargs)
            except BaseException:
                failed = True
                raise
            finally:
                done.set()
                reporter.join()
                _progress_local.active = False
                if reported[0]:
                    event(f'{message} · {"오류로 종료" if failed else "처리 종료"} · '
                          f'{int(time.monotonic()-began)}초 경과', 'ERROR' if failed else 'INFO')
        return call
    return decorate


def register_secret(value):
    """로그에서 가릴 비밀값을 등록한다."""
    if value:
        with _lock:
            _secrets.add(str(value))


def event(message, level='INFO'):
    """한 줄 로그를 출력한다. 비밀값·URL 계정·password 인자를 가리고, 줄바꿈을 합쳐 최대 400자로 자른다."""
    with _lock:
        text = str(message)
        for secret in sorted(_secrets, key=len, reverse=True):
            text = text.replace(secret, '[비공개]')
        text = re.sub(r'(://)[^/\s:@]+:[^/\s@]+@', r'\1[비공개]@', text)
        text = re.sub(r'(?i)((?:--password|password|비밀번호)\s*(?:=|:)?:?\s*)("[^"]*"|\x27[^\x27]*\x27|\S+)',
                      r'\1[비공개]', text)
        text = ' '.join(text.split())[:400]
        try:
            print(f'[{datetime.now():%H:%M:%S}] [{level}] {text}', flush=True)
        except (OSError, UnicodeError):
            pass  # A closed console must never interrupt recording.


def state_event(owner, key, value, message, level='INFO'):
    """owner별 key 상태가 바뀔 때만 로그를 남긴다(반복 폴링 시 같은 로그가 쌓이지 않게)."""
    with _lock:
        states = getattr(owner, '_console_states', None)
        if not isinstance(states, dict):
            states = {}
            owner._console_states = states
        if key in states and states[key] == value:
            return
        states[key] = value
        event(message, level)


def action(label):
    """사용자 동작(버튼 등) 시작을 ACTION 로그로 남기고, 예외가 나면 예외 종류만 ERROR로 남기는 데코레이터."""
    def decorate(function):
        @wraps(function)
        def call(*args, **kwargs):
            message = label(*args, **kwargs) if callable(label) else label
            if message:
                event(message, 'ACTION')
            try:
                return function(*args, **kwargs)
            except Exception as exc:
                event(f'{message or function.__name__} · 예외 {type(exc).__name__}', 'ERROR')
                raise
        return call
    return decorate


def install_dialog_logging(messagebox):
    # Keep dialog behavior unchanged and avoid nesting wrappers across windows.
    """messagebox의 오류/경고 대화상자를 띄울 때 같은 내용을 로그로도 남기게 감싼다. 이미 감싼 경우는 다시 감싸지 않는다."""
    for name, level in (('showerror', 'ERROR'), ('showwarning', 'WARN')):
        original = getattr(messagebox, name)
        if getattr(original, '_console_logged', False) is True:
            continue
        def logged(title=None, message=None, *args, _original=original, _level=level, **kwargs):
            event(f'{title or "알림"} · {message or ""}', _level)
            return _original(title, message, *args, **kwargs)
        logged._console_logged = True
        setattr(messagebox, name, logged)
