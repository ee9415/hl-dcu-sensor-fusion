"""Back up and replace individual /app source files on the OAK4, or roll back.

  deploy   --files a.py b.py   copy apps/yolopv2_standalone/<file> byte-for-byte
  rollback --backup DIR        restore every file saved in a backup directory

Each deploy first saves the device's current files under /app/deploy_backup_<id>
with SHA-256 sums, compiles the new sources on the device, verifies the written
bytes, then restarts the backend service. Requires Windows oakctl.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import uuid

OAKCTL = 'C:/Program Files/oakctl/bin/oakctl.exe'
APP = '00000000-0000-0000-0000-000000000000'
CHUNK = 18000  # base64 characters per exec; stays below the Windows command limit


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('command', choices=('deploy', 'rollback'))
    p.add_argument('--device-ip', required=True)
    p.add_argument('--files', nargs='+')
    p.add_argument('--backup')
    args = p.parse_args()
    root = Path(__file__).resolve().parents[3]/'apps/yolopv2_standalone'
    exec_command = [OAKCTL, 'app', 'exec', '-d', args.device_ip, APP, '--']

    def run(code):
        encoded = base64.b64encode(code.encode()).decode()
        r = subprocess.run(exec_command+['python3.12', '-c', f"import base64;exec(base64.b64decode('{encoded}'))"],
                           capture_output=True, text=True, encoding='utf-8', timeout=120)
        if r.returncode or 'Traceback (most recent call last)' in r.stdout+r.stderr:
            raise RuntimeError(r.stdout+r.stderr)
        return r.stdout

    if args.command == 'rollback':
        out = run(f"""
import hashlib,json
from pathlib import Path
b=Path({args.backup!r});sums=json.loads((b/'sha256.json').read_text())
for name,digest in sums.items():
    data=(b/name).read_bytes();assert hashlib.sha256(data).hexdigest()==digest
    Path('/app',name).write_bytes(data);assert Path('/app',name).read_bytes()==data
print('ROLLED_BACK='+json.dumps(sorted(sums)))
""")
        subprocess.run(exec_command+['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True)
        print(out.split('ROLLED_BACK=', 1)[1].splitlines()[0])
        return
    if not args.files:
        p.error('deploy needs --files')
    backup = '/app/deploy_backup_'+uuid.uuid4().hex[:10]
    staging = backup+'/incoming'
    names = [Path(name).name for name in args.files]
    digests = {}
    run(f"""
import hashlib,json
from pathlib import Path
b=Path({backup!r});Path({staging!r}).mkdir(parents=True)
sums={{}}
for name in {names!r}:
    data=Path('/app',name).read_bytes();(b/name).write_bytes(data);sums[name]=hashlib.sha256(data).hexdigest()
(b/'sha256.json').write_text(json.dumps(sums,indent=2))
print('BACKUP='+json.dumps(sums))
""")
    for name in names:
        data = (root/name).read_bytes()
        digests[name] = hashlib.sha256(data).hexdigest()
        encoded = base64.b64encode(data).decode()
        for i in range(0, len(encoded), CHUNK):
            run(f"open({staging!r}+'/{name}.b64','a').write({encoded[i:i+CHUNK]!r})")
    out = run(f"""
import base64,hashlib,json
from pathlib import Path
s=Path({staging!r});expected={digests!r};staged={{}}
for name,digest in expected.items():
    data=base64.b64decode((s/(name+'.b64')).read_text())
    assert hashlib.sha256(data).hexdigest()==digest,name
    compile(data,name,'exec');staged[name]=data
for name,data in staged.items():
    Path('/app',name).write_bytes(data);assert hashlib.sha256(Path('/app',name).read_bytes()).hexdigest()==expected[name]
import shutil;shutil.rmtree(s)
print('DEPLOYED='+json.dumps(expected))
""")
    subprocess.run(exec_command+['sv', 'restart', '/etc/service/backend'], check=True, capture_output=True)
    print(json.dumps(dict(backup=backup, deployed=json.loads(out.split('DEPLOYED=', 1)[1].splitlines()[0]))))


if __name__ == '__main__':
    main()
