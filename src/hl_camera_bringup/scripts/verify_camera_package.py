"""Record package/source identity and boot mode without changing the camera."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import socket
import subprocess

ROOT = Path(__file__).resolve().parents[3]
SOURCES = ('skeleton_response.py', 'image_boundaries.py', 'lane_overlay.py', 'yolopv2_node.py',
           'video_transport.py', 'ego_lane_quadratic.py', 'main.py', 'rtsp_server.py',
           'lane_boundaries.py', 'lane_boundaries.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-ip', default='192.168.201.2')
    parser.add_argument('--app-id', required=True)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    expected = {name:hashlib.sha256((ROOT/'apps/yolopv2_standalone'/name)
                                  .read_bytes().replace(b'\r\n', b'\n')).hexdigest()
                for name in SOURCES}
    with args.package.open('rb') as package_file:
        package_hash = hashlib.file_digest(package_file, 'sha256').hexdigest()
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    manifest = dict(package=args.package.name, bytes=args.package.stat().st_size,
                    sha256=package_hash, source_commit=commit, source_sha256=expected)
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    code = f"""
import hashlib,json
from pathlib import Path
print('IDENTITY='+json.dumps(dict(source_sha256={{name:hashlib.sha256(Path('/app',name).read_bytes()).hexdigest() for name in {SOURCES!r}}},temporary_backup_present=bool(list(Path('/app').glob('deploy_backup_*'))))))
"""
    encoded = base64.b64encode(code.encode()).decode()
    cli = 'C:/Program Files/oakctl/bin/oakctl.exe'
    result = subprocess.run([cli, 'app', 'exec', '-d', args.device_ip, args.app_id, '--',
                             'python3.12', '-c', f"import base64;exec(base64.b64decode('{encoded}'))"],
                            input='', capture_output=True, text=True, encoding='utf-8', timeout=30)
    if result.returncode or 'IDENTITY=' not in result.stdout:
        raise RuntimeError('Camera source identity query failed: '+result.stdout+result.stderr)
    identity = json.loads(result.stdout.split('IDENTITY=', 1)[1].splitlines()[0])
    with socket.create_connection((args.device_ip, 8557), timeout=8) as connection:
        connection.settimeout(8)
        connection.sendall(b'HELLO\n')
        with connection.makefile('rb') as stream:
            for _ in range(20):
                payload = json.loads(stream.readline())
                if 'frame_seq' in payload:
                    frame = payload
                    break
            else:
                raise RuntimeError('No telemetry frame received')
    identity.update(app_id=args.app_id, device_ip=args.device_ip,
                    hashes_match=identity['source_sha256'] == expected)
    (args.output/'runtime_identity.json').write_text(json.dumps(identity, indent=2), encoding='utf-8')
    (args.output/'boot_frame.json').write_text(json.dumps(frame, indent=2), encoding='utf-8')
    assert identity['hashes_match'], 'Installed source does not match the build source'
    assert not identity['temporary_backup_present'], 'Runtime includes staged hotfix backups'
    assert frame.get('processing_mode') == 'full', 'Boot processing mode is not full'
    assert frame.get('boundary_model') == 'persistent_skeleton', 'Wrong boundary tracker'
    print(json.dumps(dict(identity_ok=True, app_id=args.app_id, package_sha256=package_hash,
                          processing_mode=frame['processing_mode'], model=frame['boundary_model'])))


if __name__ == '__main__':
    main()
