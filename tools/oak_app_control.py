"""Query/control explicitly identified project camera applications."""
import json
import subprocess

IDENTIFIER = 'com.qsentech.oak4-yolopv2-standalone'
RGB_IDENTIFIER = 'com.qsentech.oak4-rgb-only'
DEVELOPMENT_ID = '00000000-0000-0000-0000-000000000000'


def command(cli, ip, args, password='', timeout=30):
    argv = [str(cli)]
    if password:
        argv += ['--password', password]
    argv += ['app', *args, '-d', ip]
    try:
        result = subprocess.run(argv, stdin=subprocess.DEVNULL, capture_output=True,
                                text=True, encoding='utf-8', errors='replace',
                                timeout=timeout,
                                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    except (OSError, subprocess.SubprocessError):
        raise RuntimeError('카메라 앱 명령에 응답이 없습니다. IP·인증·네트워크를 확인해 주세요.') from None
    if result.returncode:
        raise RuntimeError('카메라 앱 명령에 실패했습니다. IP·인증·네트워크를 확인해 주세요.')
    return result.stdout


def list_apps(cli, ip, password=''):
    text = command(cli, ip, ['list', '--format', 'json'], password)
    # CLI update notices may precede the JSON array.
    for offset, char in enumerate(text):
        if char != '[':
            continue
        try:
            items, _ = json.JSONDecoder().raw_decode(text[offset:])
        except ValueError:
            continue
        if isinstance(items, list):
            break
    else:
        raise RuntimeError('카메라 앱 상태 응답을 읽을 수 없습니다.')
    return [item for item in items if isinstance(item, dict)]


def status(cli, ip, password='', identifier=IDENTIFIER, allow_preparing=False, installed_only=False):
    matches = [item for item in list_apps(cli, ip, password) if item.get('identifier') == identifier]
    installed = [item for item in matches if item.get('app_id') != DEVELOPMENT_ID]
    if installed_only or installed:
        matches = installed
    if len(matches) > 1:
        running = [item for item in matches if str(item.get('status', '')).lower() == 'running']
        if len(running) != 1:
            raise RuntimeError('동일한 카메라 앱이 여러 개입니다. 장치 앱 구성을 확인해 주세요.')
        matches = running
    if not matches:
        return dict(state='stopped', app_id=None, detail='앱 미설치')
    item = matches[0]
    state = str(item.get('status', '')).lower()
    if allow_preparing and state in ('building', 'starting', 'initializing', 'pulling'):
        return dict(state='preparing', app_id=item.get('app_id'), detail=state)
    if state not in ('running', 'stopped', 'exited', 'created', 'ready'):
        raise RuntimeError('카메라 앱 상태를 확정할 수 없습니다: ' + state)
    return dict(state='running' if state == 'running' else 'stopped',
                app_id=item.get('app_id'), detail=state)


def stop(cli, ip, app_id, password=''):
    if not app_id:
        return
    command(cli, ip, ['stop', app_id], password, timeout=60)


def start_installed(cli, ip, identifier, password='', stop_event=None):
    """Switch known camera apps using installed packages only; never build/run."""
    items = list_apps(cli, ip, password)
    targets = [item for item in items if item.get('identifier') == identifier
               and item.get('app_id') and item['app_id'] != DEVELOPMENT_ID]
    if len(targets) > 1:
        running = [item for item in targets if str(item.get('status', '')).lower() == 'running']
        if len(running) == 1:
            targets = running
    if len(targets) != 1:
        raise RuntimeError('설치된 카메라 앱이 없거나 중복입니다. AI/RGB 패키지를 먼저 설치해 주세요.')
    target = targets[0]
    for item in items:
        if stop_event and stop_event.is_set():
            return None
        if (item.get('identifier') in (IDENTIFIER, RGB_IDENTIFIER)
                and item.get('app_id') != target['app_id'] and item.get('status', '').lower() == 'running'):
            stop(cli, ip, item['app_id'], password)
    if stop_event and stop_event.is_set():
        return None
    if target.get('status', '').lower() not in ('running', 'starting', 'initializing'):
        command(cli, ip, ['start', target['app_id'], '--enable', 'false'], password, timeout=60)
    elif target.get('enabled'):
        command(cli, ip, ['disable', target['app_id']], password)
    return target['app_id']
