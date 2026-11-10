"""Query/control explicitly identified project camera applications."""
# oakctl로 카메라 안의 프로젝트 앱(AI 앱, RGB 전용 앱) 상태를 조회하고 시작/정지한다.
# 앱은 identifier로 명확히 구분하며, 다른 앱은 건드리지 않는다.
import json
import subprocess

# AI(인지) 앱, RGB 전용 앱, 개발용 실행(oakctl run) 앱의 식별자
IDENTIFIER = 'com.qsentech.oak4-yolopv2-standalone'
RGB_IDENTIFIER = 'com.qsentech.oak4-rgb-only'
DEVELOPMENT_ID = '00000000-0000-0000-0000-000000000000'


def command(cli, ip, args, password='', timeout=30):
    """oakctl app 명령을 실행하고 stdout을 반환한다. 오류 메시지에 비밀번호가 드러나지 않게 한다."""
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
    """카메라에 설치된 앱 목록을 읽는다. 앞에 업데이트 안내가 붙어도 JSON 배열만 찾아 파싱한다."""
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
    """identifier 앱의 상태를 running/stopped(/preparing)로 정리해 반환한다.
    설치본을 개발용(oakctl run) 앱보다 우선하고, 후보가 모호하면 오류를 낸다.
    """
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
    """지정한 app_id만 정지한다."""
    if not app_id:
        return
    command(cli, ip, ['stop', app_id], password, timeout=60)


def start_installed(cli, ip, identifier, password='', stop_event=None):
    """Switch known camera apps using installed packages only; never build/run."""
    # 설치된 대상 앱으로 전환한다: 실행 중인 다른 프로젝트 앱을 멈추고, 대상 앱을 자동 시작 해제 상태로 시작한다.
    # stop_event가 설정되면 중간에 멈추고 None을 반환한다.
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
