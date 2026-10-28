"""Temporarily capture PC-frame boundary candidates on the device and restore."""
import base64
import argparse
import json
import subprocess
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'artifacts/rectangular_model_20261027/device_candidate_trace'
CMD = [r'C:\Program Files\oakctl\bin\oakctl.exe', 'app', 'exec', '-d',
       '192.168.201.2', '00000000-0000-0000-0000-000000000000', '--']


def remote(code, timeout=120):
    encoded = base64.b64encode(code.encode()).decode()
    run = subprocess.run(CMD + ['python3.12', '-c',
                         f"import base64;exec(base64.b64decode('{encoded}'))"],
                         capture_output=True, text=True, timeout=timeout)
    if run.returncode or 'Traceback (most recent call last)' in run.stdout + run.stderr:
        raise RuntimeError(run.stdout + run.stderr)
    return run.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', type=float, default=60.)
    parser.add_argument('--end', type=float, default=76.)
    parser.add_argument('--output', type=Path, default=OUT)
    parser.add_argument('--fit-evidence', action='store_true',
                        help='실제 피팅 점과 곡선 행을 진단 정보에 기록합니다')
    args = parser.parse_args()
    if args.start < 0 or args.end <= args.start:
        parser.error('영상 구간이 올바르지 않습니다')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    name = 'boundary_trace_' + uuid.uuid4().hex[:10]
    backup = '/app/' + name + '_backup.py'
    capture = '/app/' + name + '.npz'
    module = '/app/' + name + '.py'
    probe = (ROOT/'src/hl_camera_bringup/scripts/boundary_debug_capture.py').read_text()
    probe = probe.replace("COUNT = min(100, max(0, int(os.environ.get('BOUNDARY_DEBUG_FRAMES', '0'))))", 'COUNT = 25')
    probe = probe.replace("WARMUP = max(0, int(os.environ.get('BOUNDARY_DEBUG_WARMUP', '60')))", 'WARMUP = 0')
    probe = probe.replace("DEST = Path('/app/boundary_debug.npz')", f'DEST = Path({capture!r})')
    probe = probe.replace('    seen += 1\n', "    source = kwargs.pop('_capture_source', {})\n    if source.get('input_source') != 'pc':\n        return tracker.update(*args, **kwargs)\n    seen += 1\n")
    probe = probe.replace("    record = {'history_before':", "    import cv2\n    arrays[prefix+'_rgb'] = cv2.resize(kwargs['rgb'], (320,180))\n    record = {'source': source, 'rgb': {'array': prefix+'_rgb'}, 'history_before':")
    probe = probe.replace("('update','_detect','_complete','_warp','_fuse')", "('update','_detect','_complete','_warp','_fuse','_detect_full','_detect_legacy','_detect_regions')")
    if args.fit_evidence:
        marker = "    prefix = 'f%03d' % len(records)\n"
        assert probe.count(marker) == 1
        probe = probe.replace(marker, '    tracker.trace_fit_evidence = True\n' + marker, 1)
    assert 'COUNT = 25' in probe and "kwargs.pop('_capture_source'" in probe
    manifest = {'backup': backup, 'capture': capture,
                'fit_evidence': args.fit_evidence, 'restored': False}
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
    installed = False
    try:
        remote(f"from pathlib import Path;p=Path('/app/yolopv2_node.py');Path({backup!r}).write_bytes(p.read_bytes());print('BACKUP_OK')")
        installed = True
        payload = base64.b64encode(probe.encode()).decode()
        replacement = f'boundaries = __import__({name!r}).update(self._boundary_tracker,'
        remote(f'''from pathlib import Path
import base64
Path({module!r}).write_bytes(base64.b64decode({payload!r}))
p=Path('/app/yolopv2_node.py');s=p.read_text()
a='boundaries = self._boundary_tracker.update('
b='detections=detection_messages)'
assert s.count(a)==1 and s.count(b)==1
s=s.replace(a,{replacement!r},1)
s=s.replace(b,'detections=detection_messages, _capture_source=source)',1)
compile(s,str(p),'exec');p.write_text(s)
print('PATCH_OK')''')
        subprocess.run(CMD + ['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True, timeout=30)
        time.sleep(18)
        output = subprocess.check_output(['wsl', '-d', 'Ubuntu-22.04', '--',
                                          'wslpath', '-a', (out/'replay').as_posix()], text=True).strip()
        p = subprocess.run(['wsl','-d','Ubuntu-22.04','--','python3',
             '/mnt/d/git_hub/hl-dcu-sensor-fusion/src/hl_camera_bringup/scripts/verify_pc_video_device.py',
             '--device-ip','192.168.201.2','--video','/mnt/d/project/halla_yolopv2/KakaoTalk_20261016_170026655.mp4',
             '--output',output,'--seconds','20','--start',str(args.start),'--end',str(args.end),'--fps','10','--processing','full'],
             capture_output=True, text=True, timeout=120)
        (out/'replay_console.txt').write_text(p.stdout+'\n'+p.stderr, encoding='utf8')
        if p.returncode:
            raise RuntimeError('PC replay failed: '+p.stderr)
        data = remote(f"from pathlib import Path;import base64;print('PAYLOAD='+base64.b64encode(Path({capture!r}).read_bytes()).decode())", timeout=180)
        (out/'boundary_trace.npz').write_bytes(base64.b64decode(data.split('PAYLOAD=',1)[1].splitlines()[0].strip(), validate=True))
        print('Captured device trace', flush=True)
    finally:
        if installed:
            remote(f"from pathlib import Path;b=Path({backup!r});p=Path('/app/yolopv2_node.py');p.write_bytes(b.read_bytes());assert p.read_bytes()==b.read_bytes();print('RESTORED')")
            subprocess.run(CMD + ['sv','restart','/etc/service/backend'], check=True, capture_output=True, timeout=30)
            manifest['restored'] = True
            (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
            print('Original device source restored', flush=True)


if __name__ == '__main__':
    main()
