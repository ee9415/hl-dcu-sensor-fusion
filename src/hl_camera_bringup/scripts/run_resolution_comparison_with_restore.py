"""Temporarily install resolution-aware code, compare, then restore device files."""
import argparse
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
    state = dict(temporary_deployment=info, comparison_complete=False, restored=False)
    manifest = a.output/'outer_manifest.json'
    def save():
        manifest.write_text(json.dumps(state, indent=2), encoding='utf-8')
    save()
    print('Temporary resolution-aware files deployed; backup: '+info['backup'], flush=True)
    try:
        cmd = [sys.executable, str(compare), '--device-ip', a.device_ip,
               '--video', str(a.video), '--output', str(a.output/'comparison'),
               '--seconds', str(a.seconds), '--fps', str(a.fps)]
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
