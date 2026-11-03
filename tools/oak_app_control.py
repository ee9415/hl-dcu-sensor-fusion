"""Control only the project's YOLOPv2 app, without starting other camera apps."""
import json
import subprocess

IDENTIFIER = 'com.qsentech.oak4-yolopv2-standalone'


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


def status(cli, ip, password=''):
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
    matches = [item for item in items if isinstance(item, dict)
               and item.get('identifier') == IDENTIFIER]
    if len(matches) > 1:
        raise RuntimeError('동일한 카메라 앱이 여러 개입니다. 장치 앱 구성을 확인해 주세요.')
    if not matches:
        return dict(state='stopped', app_id=None, detail='앱 미설치')
    item = matches[0]
    state = str(item.get('status', '')).lower()
    if state not in ('running', 'stopped', 'exited', 'created'):
        raise RuntimeError('카메라 앱 상태를 확정할 수 없습니다: ' + state)
    return dict(state='running' if state == 'running' else 'stopped',
                app_id=item.get('app_id'), detail=state)


def stop(cli, ip, app_id, password=''):
    if not app_id:
        return
    command(cli, ip, ['stop', app_id], password, timeout=60)
