"""Snapshot/deploy/rollback the optional guidance without replacing model/main.

The local snapshot includes device originals (including absent-file markers),
SHA256, app configuration, health logs and repository commit. Rollback refuses
to overwrite files changed after this deployment. Uses Windows oakctl only.
"""
import argparse
import ast
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import uuid

OAKCTL = 'C:/Program Files/oakctl/bin/oakctl.exe'
APP = '00000000-0000-0000-0000-000000000000'
NAMES = ('yolopv2_node.py', 'virtual_guidance.py', 'virtual_guidance.json')
CONTEXT = ('main.py', 'postprocess.py', 'image_boundaries.py', 'lane_boundaries.json')
ROOT = Path(__file__).resolve().parents[3]


def adapt_node_to_installed_postprocess(node, postprocess):
    """Keep the installed 320-only postprocess API; never upgrade it implicitly."""
    node = node.replace('\r\n', '\n')
    definitions = {n.name:len(n.args.args) for n in ast.parse(postprocess).body
                   if isinstance(n, ast.FunctionDef)}
    if definitions.get('map_letterboxed_box_to_source') == 2:
        old = 'np.asarray((x1, y1, x2, y2)), self._source_size, self.model_size'
        assert node.count(old) == 1
        node = node.replace(old, 'np.asarray((x1, y1, x2, y2)), self._source_size')
    if definitions.get('segmentation_masks') == 3:
        for old in ('\n                    self.model_size,\n', '\n            self.model_size,\n'):
            assert node.count(old) == 1
            node = node.replace(old, '\n')
    compile(node, 'yolopv2_node.py', 'exec')
    return node


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=('snapshot', 'deploy', 'update', 'rollback'))
    p.add_argument('--device-ip', default='192.168.201.2')
    p.add_argument('--backup', type=Path, required=True)
    p.add_argument('--with-transport', action='store_true',help='Snapshot and deploy display control transport too')
    p.add_argument('--with-lane-overlay', action='store_true',help='Include preview-only lane score renderer')
    args = p.parse_args()
    b = args.backup.resolve()
    names = NAMES+('video_transport.py',) if args.with_transport else NAMES
    if args.with_lane_overlay:
        names += ('lane_overlay.py',)
    command = [OAKCTL, 'app', 'exec', '-d', args.device_ip, APP, '--']

    def run(code):
        encoded = base64.b64encode(code.encode()).decode()
        result = subprocess.run(command+['python3.12', '-c',
            f"import base64;exec(base64.b64decode('{encoded}'))"],
            capture_output=True, text=True, encoding='utf8', timeout=60)
        if result.returncode or 'Traceback (most recent call last)' in result.stdout+result.stderr:
            raise RuntimeError(result.stdout+result.stderr)
        return result.stdout

    def hashes():
        out = run(f"""
import hashlib,json
from pathlib import Path
print('RESULT='+json.dumps({{n:hashlib.sha256(Path('/app',n).read_bytes()).hexdigest()
 if Path('/app',n).exists() else None for n in {names+CONTEXT!r}}}))
""")
        return json.loads(out.split('RESULT=', 1)[1].splitlines()[0])

    if args.command == 'snapshot':
        b.mkdir(parents=True, exist_ok=False)
        payload = run(f"""
import base64,json
from pathlib import Path
print('RESULT='+json.dumps({{n:base64.b64encode(Path('/app',n).read_bytes()).decode()
 if Path('/app',n).exists() else None for n in {names+CONTEXT!r}}}))
""")
        originals = json.loads(payload.split('RESULT=', 1)[1].splitlines()[0])
        digests = {}
        for name, encoded in originals.items():
            data = None if encoded is None else base64.b64decode(encoded)
            digests[name] = None if data is None else hashlib.sha256(data).hexdigest()
            if data is not None:
                (b/name).write_bytes(data)
        environment = run("""
import json,os
from pathlib import Path
keys=('OAK_INPUT_FPS','OAK_VIRTUAL_GUIDANCE','OAK_WEB_VISUALIZER')
result={k:os.environ.get(k) for k in keys}
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        cmd=(p/'cmdline').read_bytes()
        if b'python' not in cmd or b'main.py' not in cmd:continue
        env=dict(v.split(b'=',1) for v in (p/'environ').read_bytes().split(b'\\0') if b'=' in v)
        result={k:env.get(k.encode(),b'').decode(errors='replace') for k in keys}
        break
    except (OSError,ValueError):pass
print('RESULT='+json.dumps(result))
""")
        (b/'app_config.txt').write_text(environment,encoding='utf8')
        for name, sub in (('health_before.txt', ['logs', APP, '--no-follow', '--tail', '5']),):
            r = subprocess.run([OAKCTL, 'app']+sub+['-d', args.device_ip],
                capture_output=True, text=True, encoding='utf8', timeout=30)
            (b/name).write_text(r.stdout+r.stderr, encoding='utf8')
            if r.returncode:
                raise RuntimeError(f'Unable to record {name}')
        manifest = dict(device_ip=args.device_ip, originals=digests,files=list(names),
                        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
        (b/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
        print(json.dumps(dict(snapshot=str(b), originals=digests)))
        return

    manifest = json.loads((b/'manifest.json').read_text(encoding='utf8'))
    names=tuple(manifest.get('files',NAMES))
    if args.device_ip != manifest['device_ip']:
        raise RuntimeError('Snapshot belongs to another device')
    current = hashes()
    if args.command in ('deploy', 'update'):
        if args.command == 'deploy' and (b/'deployment.json').exists():
            raise RuntimeError('This snapshot was already deployed; create a new one')
        before = dict(manifest['originals'])
        if args.command == 'update':
            before.update(json.loads((b/'deployment.json').read_text(encoding='utf8'))['deployed'])
        if current != before:
            raise RuntimeError('Device changed since snapshot; refusing deployment')
        files = {n:(ROOT/'apps/yolopv2_standalone'/n).read_bytes() for n in names}
        source = adapt_node_to_installed_postprocess(files['yolopv2_node.py'].decode('utf8'),
                                                     (b/'postprocess.py').read_text(encoding='utf8'))
        files['yolopv2_node.py'] = source.encode('utf8')
        (b/'deployed_yolopv2_node.py').write_bytes(files['yolopv2_node.py'])
    else:
        deployment = json.loads((b/'deployment.json').read_text(encoding='utf8'))
        if any(current[n] != digest for n, digest in deployment['deployed'].items()):
            raise RuntimeError('Device files changed after deployment; refusing rollback overwrite')
        files = {n:None if manifest['originals'][n] is None else (b/n).read_bytes() for n in names}
        for n, data in files.items():
            if data is not None and hashlib.sha256(data).hexdigest() != manifest['originals'][n]:
                raise RuntimeError(f'Corrupt rollback original: {n}')
    expected = {n:None if data is None else hashlib.sha256(data).hexdigest() for n,data in files.items()}
    stage = '/app/guidance_staging_'+uuid.uuid4().hex[:10]
    run(f"from pathlib import Path;Path({stage!r}).mkdir()")
    for n, data in files.items():
        if data is None:
            continue
        encoded = base64.b64encode(data).decode()
        for i in range(0, len(encoded), 18000):
            run(f"open({stage!r}+'/{n}.b64','a').write({encoded[i:i+18000]!r})")
    # Compile all sources before replacing anything. Keep remote originals too.
    remote_backup = stage+'/originals'
    run(f"""
import base64,hashlib,json,shutil
from pathlib import Path
s=Path({stage!r});expected={expected!r};staged={{}}
for n,digest in expected.items():
    if digest is None:continue
    data=base64.b64decode((s/(n+'.b64')).read_text())
    assert hashlib.sha256(data).hexdigest()==digest,n
    if n.endswith('.py'):compile(data,n,'exec')
    if n.endswith('.json'):json.loads(data)
    staged[n]=data
b=Path({remote_backup!r});b.mkdir()
old={{n:Path('/app',n).read_bytes() if Path('/app',n).exists() else None for n in expected}}
for n,data in old.items():
    if data is not None:(b/n).write_bytes(data)
(b/'manifest.json').write_text(json.dumps({{n:None if d is None else hashlib.sha256(d).hexdigest() for n,d in old.items()}}))
try:
    for n,digest in expected.items():
        dest=Path('/app',n)
        if digest is None:
            if dest.exists():dest.unlink()
        else:
            tmp=s/n;tmp.write_bytes(staged[n]);tmp.replace(dest)
    for n,digest in expected.items():
        assert (hashlib.sha256(Path('/app',n).read_bytes()).hexdigest() if Path('/app',n).exists() else None)==digest,n
except BaseException:
    for n,data in old.items():
        dest=Path('/app',n)
        if data is None:
            if dest.exists():dest.unlink()
        else:dest.write_bytes(data)
    raise
print('FILES_VERIFIED')
""")
    record = dict(deployed=expected, remote_backup=remote_backup)
    # Persist before restarting: rollback remains possible if startup fails.
    record_name = 'deployment.json' if args.command in ('deploy','update') else 'rollback.json'
    if args.command == 'update':
        (b/('deployment_previous_'+uuid.uuid4().hex[:10]+'.json')).write_bytes((b/'deployment.json').read_bytes())
    (b/record_name).write_text(json.dumps(record,indent=2),encoding='utf8')
    subprocess.run(command+['sv','restart','/etc/service/backend'],check=True,capture_output=True,timeout=30)
    after = hashes()
    assert all(after[n] == expected[n] for n in names)
    assert all(after[n] == manifest['originals'][n] for n in CONTEXT)
    print(json.dumps(dict(action=args.command, backup=str(b), verified=expected,
                          unchanged_context=True, remote_backup=remote_backup)))


if __name__ == '__main__':
    main()
