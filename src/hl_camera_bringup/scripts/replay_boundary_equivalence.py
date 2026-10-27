"""Bit-exact A/B replay of boundary/stop-line/render code on the OAK4's own NumPy.

Uploads a baseline tree (git ref, default HEAD) and the working tree to a
temporary directory on the device, replays identical inputs through both in
separate python3.12 processes, and compares every record with zero tolerance.

Inputs: deterministic synthetic sequences and test fixtures always; optionally
real device inputs captured with --capture-video. Capture temporarily patches
/app/yolopv2_node.py to save each PC frame's tracker inputs (180x320 RGB sample,
drivable/lane masks, NN timestamp, boxes, source epoch), backs it up first and
restores/restarts it in finally, like profile_oak_pipeline.py. Algorithms are
not changed by this tool. Requires Windows oakctl (and WSL OpenCV for capture).
"""
import argparse
import base64
import io
import json
from pathlib import Path
import subprocess
import tarfile
import time
import uuid

OAKCTL = 'C:/Program Files/oakctl/bin/oakctl.exe'
APP = '00000000-0000-0000-0000-000000000000'
TREE = 'apps/yolopv2_standalone'
CHUNK = 18000  # base64 characters per exec; stays below the Windows command limit


def wanted(name):
    name = name.replace('\\', '/')
    return ((name.endswith('.py') or name.endswith('.json')) and '/tests/' not in name
            and '/models/' not in name or '/tests/fixtures/' in name and name.endswith('.npz'))


def bundle(root, ref):
    archive = subprocess.run(['git', '-C', str(root), 'archive', '--format=tar', ref, TREE],
                             check=True, capture_output=True).stdout
    out = io.BytesIO()
    with tarfile.open(fileobj=out, mode='w:gz') as target:
        with tarfile.open(fileobj=io.BytesIO(archive)) as source:
            for member in source.getmembers():
                if member.isfile() and wanted(member.name):
                    member.name = 'base/'+member.name[len(TREE)+1:]
                    target.addfile(member, source.extractfile(member))
        for path in sorted((root/TREE).rglob('*')):
            relative = path.relative_to(root/TREE).as_posix()
            if path.is_file() and '__pycache__' not in relative and wanted('/'+relative):
                target.add(path, 'candidate/'+relative)
        target.add(root/'src/hl_camera_bringup/scripts/boundary_replay_runner.py', 'runner.py')
    return out.getvalue()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--device-ip', required=True)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--baseline-ref', default='HEAD')
    p.add_argument('--capture-video', type=Path, help='PC video for real device input capture')
    p.add_argument('--capture-seconds', type=int, default=20)
    p.add_argument('--capture-frames', type=int, default=200)
    p.add_argument('--start', type=float, default=2.)
    p.add_argument('--end', type=float, default=18.)
    p.add_argument('--keep', action='store_true', help='keep the device work directory for inspection')
    args = p.parse_args()
    root = Path(__file__).resolve().parents[3]
    args.output.mkdir(parents=True, exist_ok=True)
    work = '/tmp/boundary_replay_'+uuid.uuid4().hex[:10]
    command = [OAKCTL, 'app', 'exec', '-d', args.device_ip, APP, '--']

    def run(code, timeout=120):
        encoded = base64.b64encode(code.encode()).decode()
        r = subprocess.run(command+['python3.12', '-c', f"import base64;exec(base64.b64decode('{encoded}'))"],
                           capture_output=True, text=True, encoding='utf-8', timeout=timeout)
        if r.returncode or 'Traceback (most recent call last)' in r.stdout+r.stderr:
            raise RuntimeError(r.stdout+r.stderr)
        return r.stdout

    def marker(text, name):
        return json.loads(text.split(name+'=', 1)[1].splitlines()[0])

    data = base64.b64encode(bundle(root, args.baseline_ref)).decode()
    run(f"from pathlib import Path;Path({work!r}).mkdir()")
    for i in range(0, len(data), CHUNK):
        run(f"open({work!r}+'/bundle.b64','a').write({data[i:i+CHUNK]!r})")
    run(f"""
import base64,io,tarfile
from pathlib import Path
w=Path({work!r});raw=base64.b64decode((w/'bundle.b64').read_text())
tarfile.open(fileobj=io.BytesIO(raw)).extractall(w)
assert (w/'base/image_boundaries.py').exists() and (w/'candidate/image_boundaries.py').exists()
""")
    report = dict(device_ip=args.device_ip, work=work, baseline_ref=args.baseline_ref,
                  baseline_commit=subprocess.check_output(['git', '-C', str(root), 'rev-parse', args.baseline_ref],
                                                          text=True).strip(), captured=None)
    captured = None
    if args.capture_video:
        captured = work+'/captured'
        report['captured'] = capture(args, root, run, captured, args.output)
    try:
        flag = f" --captured {captured}" if captured else ''
        for name in ('base', 'candidate'):
            out = run(f"import subprocess,sys;r=subprocess.run(['python3.12','{work}/runner.py','run','--tree',"
                      f"'{work}/{name}','--output','{work}/{name}.pkl']+{flag.split()!r},capture_output=True,text=True);"
                      "print(r.stdout+r.stderr);sys.exit(r.returncode)", timeout=1800)
            report[name+'_frames'] = json.loads(out.strip().splitlines()[-1])
        out = run(f"import subprocess;r=subprocess.run(['python3.12','{work}/runner.py','compare','{work}/base.pkl',"
                  f"'{work}/candidate.pkl'],capture_output=True,text=True);print(r.stdout+r.stderr)", timeout=600)
        report['replay'] = marker(out, 'REPLAY_REPORT')
        report['identical'] = report['replay']['identical']
    finally:
        if not args.keep:
            run(f"import shutil;shutil.rmtree({work!r})")
        (args.output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(dict(identical=report.get('identical'), environment=report['replay']['environment'],
                          sequences={k: v['frames'] for k, v in report['replay']['sequences'].items()})))


def capture(args, root, run, captured, output):
    backup = captured+'_app_backup'
    anchor = '        boundary_started = time.monotonic()'
    hook = f"""        if source.get('input_source') == 'pc' and getattr(self, '_replay_count', 0) < {args.capture_frames}:
            self._replay_count = getattr(self, '_replay_count', 0) + 1
            _rgb = preview.getCvFrame()
            _image = _rgb[np.linspace(0, _rgb.shape[0]-1, 180).astype(int)[:, None],
                          np.linspace(0, _rgb.shape[1]-1, 320).astype(int)[None, :]]
            np.savez('{captured}/%06d.npz' % self._replay_count, image=_image, road=drivable, lane=lane,
                     timestamp=nn_data.getTimestamp().total_seconds(),
                     boxes=np.array([[d.xmin, d.ymin, d.xmax, d.ymax] for d in detection_messages], float).reshape(-1, 4),
                     epoch=source.get('source_epoch'), frame_index=source.get('source_frame_index'))
"""
    old, new = '\n'+anchor+'\n', '\n'+hook+anchor+'\n'
    run(f"""
from pathlib import Path
path=Path('/app/yolopv2_node.py');s=path.read_text()
assert s.count({old!r})==1 and '_replay_count' not in s
Path({captured!r}).mkdir();Path({backup!r}).mkdir()
(Path({backup!r})/'yolopv2_node.py').write_bytes(path.read_bytes())
s=s.replace({old!r},{new!r},1)
compile(s,'yolopv2_node.py','exec');path.write_text(s)
""")
    (output/'capture_manifest.json').write_text(json.dumps(dict(backup=backup, restored=False), indent=2))
    command = [OAKCTL, 'app', 'exec', '-d', args.device_ip, APP, '--']
    try:
        subprocess.run(command+['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True)
        time.sleep(18)  # model startup; the verifier waits for the telemetry port itself

        def wslpath(path):
            return subprocess.check_output(['wsl', '-d', 'Ubuntu-22.04', '--', 'wslpath', '-a',
                                            path.resolve().as_posix()], text=True).strip()
        r = subprocess.run(['wsl', '-d', 'Ubuntu-22.04', '--', 'python3',
                            wslpath(root/'src/hl_camera_bringup/scripts/verify_pc_video_device.py'),
                            '--device-ip', args.device_ip, '--video', wslpath(args.capture_video),
                            '--output', wslpath(output/'capture'), '--seconds', str(args.capture_seconds),
                            '--start', str(args.start), '--end', str(args.end)],
                           capture_output=True, text=True, encoding='utf-8', timeout=args.capture_seconds+120)
        (output/'capture_console.txt').write_text(r.stdout+r.stderr, encoding='utf-8')
    finally:
        run(f"from pathlib import Path;b=Path({backup!r})/'yolopv2_node.py';a=Path('/app/yolopv2_node.py');"
            "a.write_bytes(b.read_bytes());assert a.read_bytes()==b.read_bytes();b.unlink();b.parent.rmdir();print('RESTORED')")
        subprocess.run(command+['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True)
        (output/'capture_manifest.json').write_text(json.dumps(dict(backup=backup, restored=True), indent=2))
    count = run(f"from pathlib import Path;print('COUNT='+str(len(list(Path({captured!r}).glob('*.npz')))))")
    frames = int(count.split('COUNT=', 1)[1].split()[0])
    if frames < 20:
        raise RuntimeError(f'Only {frames} device frames were captured')
    return dict(frames=frames, directory=captured)


if __name__ == '__main__':
    main()
