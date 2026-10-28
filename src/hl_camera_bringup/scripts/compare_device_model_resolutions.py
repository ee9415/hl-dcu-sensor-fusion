"""Temporarily compare two installed OAK models, restoring main.py in finally.

Runs full-processing PC road replay in ABBA order across two segments. The
verification pipeline generates debug streams, so rates are not headless FPS.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import uuid

OAK = 'C:/Program Files/oakctl/bin/oakctl.exe'
APP = '00000000-0000-0000-0000-000000000000'
ROOT = Path(__file__).resolve().parents[3]
RECT_SHA = 'eb41e41492381f29210b6ae194941ed8f24fb68eabfe3583bfa8cfc97009a4b7'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip', required=True)
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=int, default=25)
    p.add_argument('--fps', type=float, default=10.5)
    p.add_argument('--capture-only', action='store_true', help='Capture instrumented masks, not FPS')
    p.add_argument('--starts', type=int, nargs='+', default=[60, 303])
    p.add_argument('--order', type=int, nargs='+', choices=(320, 192), default=[320, 192, 192, 320])
    args = p.parse_args()
    if args.capture_only and (args.device_ip != '192.168.201.2' or
                              args.video.resolve() != Path(r'D:\project\halla_yolopv2\KakaoTalk_20261016_170026655.mp4').resolve()):
        p.error('capture helper is fixed to device 192.168.201.2 and the original KakaoTalk video')
    args.output.mkdir(parents=True, exist_ok=True)
    command = [OAK, 'app', 'exec', '-d', args.device_ip, APP, '--']
    backup = '/app/resolution_compare_backup_' + uuid.uuid4().hex[:10]
    manifest = dict(device=args.device_ip, backup=backup, restored=False, runs=[])
    manifest_path = args.output/'manifest.json'

    def save():
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')

    def remote(code):
        payload = base64.b64encode(code.encode()).decode()
        r = subprocess.run(command+['python3.12', '-c', f"import base64;exec(base64.b64decode('{payload}'))"],
                           capture_output=True, text=True, encoding='utf-8', timeout=120)
        if r.returncode or 'Traceback (most recent call last)' in r.stdout+r.stderr:
            raise RuntimeError(r.stdout+r.stderr)
        return r.stdout

    def restart():
        subprocess.run(command+['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True)

    def wait_ready():
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            try:
                for port in (8557, 8558):
                    with socket.create_connection((args.device_ip, port), timeout=2):
                        pass
                return
            except OSError:
                time.sleep(1)
        raise RuntimeError('Device ports did not become ready')

    def wslpath(path):
        return subprocess.check_output(['wsl','-d','Ubuntu-22.04','--','wslpath','-a',path.resolve().as_posix()], text=True).strip()

    def health(run_dir, expected):
        r = subprocess.run([OAK, 'app', 'logs', APP, '-d', args.device_ip, '--no-follow', '--tail', '60'],
                           check=True, capture_output=True, text=True, encoding='utf-8', timeout=30)
        (run_dir/'health.log').write_text(r.stdout, encoding='utf-8')
        shapes = re.findall(r'"input_shape"\s*:\s*\[\s*1\s*,\s*3\s*,\s*(\d+)\s*,\s*(\d+)\s*\]', r.stdout)
        if not shapes or tuple(map(int, shapes[-1])) != (expected, 320):
            raise RuntimeError(f'Unexpected device input shape: {shapes[-1:]}, expected {expected}x320')

    verifier = wslpath(ROOT/'src/hl_camera_bringup/scripts/verify_pc_video_device.py')
    video = wslpath(args.video)
    remote(f"""
from pathlib import Path
import hashlib,json
b=Path({backup!r});b.mkdir()
main=Path('/app/main.py').read_bytes();(b/'main.py').write_bytes(main)
model=Path('/app/models/yolopv2_person_320x192.rvc4.tar.xz')
assert hashlib.sha256(model.read_bytes()).hexdigest()=={RECT_SHA!r}
(b/'manifest.json').write_text(json.dumps(dict(main_sha256=hashlib.sha256(main).hexdigest())))
""")
    save()
    try:
        for block, shape in enumerate((320, 192) if args.capture_only else args.order, 1):
            remote(f"""
from pathlib import Path
import hashlib
b=Path({backup!r});source=(b/'main.py').read_text()
if {shape}==192:
    old="int(os.environ.get('YOLOPV2_INPUT_HEIGHT', '320'))"
    new="int(os.environ.get('YOLOPV2_INPUT_HEIGHT', '192'))"
    assert source.count(old)==1
    source=source.replace(old,new,1)
    old="'models' / 'yolopv2_person_320.rvc4.tar.xz'"
    new="'models' / 'yolopv2_person_320x192.rvc4.tar.xz'"
    assert source.count(old)==1
    source=source.replace(old,new,1)
compile(source,'main.py','exec')
tmp=Path('/app/main.py.resolution_compare');tmp.write_text(source);tmp.replace('/app/main.py')
""")
            restart()
            wait_ready()
            if args.capture_only:
                if args.device_ip != '192.168.201.2':
                    raise ValueError('Capture helper currently supports only 192.168.201.2')
                capture_dir = args.output/f'capture_height{shape}'
                capture = ROOT/'src/hl_camera_bringup/scripts/capture_pc_boundary_candidates.py'
                subprocess.run([sys.executable, str(capture), '--start', '60', '--end', '76',
                                '--output', str(capture_dir)], check=True)
                manifest['runs'].append(dict(name=capture_dir.name, shape=shape, instrumented=True))
                save()
                continue
            for start in args.starts:
                name = f'block{block}_height{shape}_sec{start}'
                run_dir = args.output/name
                run_dir.mkdir(parents=True, exist_ok=True)
                cmd = ['wsl','-d','Ubuntu-22.04','--',
                       '/mnt/d/git_hub/hl-dcu-sensor-fusion/artifacts/model_conversion_env/bin/python',
                       verifier, '--device-ip',args.device_ip,'--video',video,'--output',wslpath(run_dir),
                       '--seconds',str(args.seconds),'--start',str(start),'--end',str(start+16),
                       '--fps',str(args.fps),'--processing','full']
                result = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', timeout=args.seconds+100)
                (run_dir/'console.txt').write_text(result.stdout+result.stderr, encoding='utf-8')
                result.check_returncode()
                health(run_dir, shape)
                report = json.loads((run_dir/'report.json').read_text())
                if not report['passed'] or not report['camera_recovered']:
                    raise RuntimeError(f'Run did not pass: {name}')
                manifest['runs'].append(dict(name=name, shape=shape, start=start,
                                             output_fps=report['output_fps']))
                save()
                print(json.dumps(manifest['runs'][-1]), flush=True)
    finally:
        remote(f"""
from pathlib import Path
import hashlib,json
b=Path({backup!r});data=(b/'main.py').read_bytes()
assert hashlib.sha256(data).hexdigest()==json.loads((b/'manifest.json').read_text())['main_sha256']
p=Path('/app/main.py.restore');p.write_bytes(data);p.replace('/app/main.py')
assert hashlib.sha256(Path('/app/main.py').read_bytes()).hexdigest()==hashlib.sha256(data).hexdigest()
""")
        restart()
        manifest['restored']=True
        save()
        print('Original main.py restored and backend restarted', flush=True)


if __name__ == '__main__':
    main()
