"""Temporarily install resolution-aware code, compare, then restore device files."""
import argparse
import base64
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip', required=True)
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=int, default=25)
    p.add_argument('--fps', type=float, default=10.5)
    p.add_argument('--capture-only', action='store_true')
    p.add_argument('--without-video', action='store_true', help='Temporarily suppress debug video')
    p.add_argument('--starts', type=int, nargs='+', default=[60, 303])
    p.add_argument('--order', type=int, nargs='+', choices=(320, 192), default=[320, 192, 192, 320])
    a = p.parse_args()
    if a.capture_only and (a.device_ip != '192.168.201.2' or
                           a.video.resolve() != Path(r'D:\project\halla_yolopv2\KakaoTalk_20261016_170026655.mp4').resolve()):
        p.error('capture helper is fixed to device 192.168.201.2 and the original KakaoTalk video')
    a.output.mkdir(parents=True, exist_ok=True)
    deployment = ROOT/'src/hl_camera_bringup/scripts/deploy_app_files.py'
    compare = ROOT/'src/hl_camera_bringup/scripts/compare_device_model_resolutions.py'
    result = subprocess.run([sys.executable, str(deployment), 'deploy', '--device-ip', a.device_ip,
                             '--files', 'main.py', 'yolopv2_node.py', 'postprocess.py'],
                            capture_output=True, text=True, encoding='utf-8', check=True)
    info = json.loads(result.stdout.strip().splitlines()[-1])
    state = dict(temporary_deployment=info, without_video=a.without_video, comparison_complete=False, restored=False)
    manifest = a.output/'outer_manifest.json'
    def save():
        manifest.write_text(json.dumps(state, indent=2), encoding='utf-8')
    save()
    print('Temporary resolution-aware files deployed; backup: '+info['backup'], flush=True)
    try:
        if a.without_video:
            if a.capture_only:
                raise ValueError('Video suppression is not compatible with capture-only diagnostics')
            patch = (ROOT/'src/hl_camera_bringup/scripts/temporary_no_video_patch.py').read_bytes()
            payload = base64.b64encode(patch).decode()
            code = f"""
import base64
from pathlib import Path
scope={{}};exec(base64.b64decode({payload!r}),scope)
main=Path('/app/main.py');node=Path('/app/yolopv2_node.py')
changed_main=scope['patch_main'](main.read_text());changed_node=scope['patch_node'](node.read_text())
for path,source in ((main,changed_main),(node,changed_node)):
    temp=path.with_name(path.name+'.no_video');temp.write_text(source);temp.replace(path)
print('NO_VIDEO_PATCHED')
"""
            encoded = base64.b64encode(code.encode()).decode()
            cmd = ['C:/Program Files/oakctl/bin/oakctl.exe','app','exec','-d',a.device_ip,
                   '00000000-0000-0000-0000-000000000000','--','python3.12','-c',
                   f"import base64;exec(base64.b64decode('{encoded}'))"]
            result = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', timeout=120)
            if result.returncode or 'Traceback (most recent call last)' in result.stdout+result.stderr or 'NO_VIDEO_PATCHED' not in result.stdout:
                raise RuntimeError(result.stdout+result.stderr)
            print('Temporary debug video suppression applied', flush=True)
        cmd = [sys.executable, str(compare), '--device-ip', a.device_ip,
               '--video', str(a.video), '--output', str(a.output/'comparison'),
               '--seconds', str(a.seconds), '--fps', str(a.fps),
               '--starts', *[str(x) for x in a.starts], '--order', *[str(x) for x in a.order]]
        if a.capture_only:
            cmd.append('--capture-only')
        subprocess.run(cmd, check=True)
        state['comparison_complete'] = True
    finally:
        subprocess.run([sys.executable, str(deployment), 'rollback', '--device-ip', a.device_ip,
                        '--backup', info['backup']], check=True)
        state['restored'] = True
        save()
        print('Original device files restored and backend restarted', flush=True)


if __name__ == '__main__':
    main()
