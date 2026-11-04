"""Compact console events; no frame logs, callback arguments or tracebacks."""
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
    if value:
        with _lock:
            _secrets.add(str(value))


def event(message, level='INFO'):
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
    for name, level in (('showerror', 'ERROR'), ('showwarning', 'WARN')):
        original = getattr(messagebox, name)
        if getattr(original, '_console_logged', False) is True:
            continue
        def logged(title=None, message=None, *args, _original=original, _level=level, **kwargs):
            event(f'{title or "알림"} · {message or ""}', _level)
            return _original(title, message, *args, **kwargs)
        logged._console_logged = True
        setattr(messagebox, name, logged)
