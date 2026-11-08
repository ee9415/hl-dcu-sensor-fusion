"""Build/deploy small code bundles to an installed OAK runtime; no image rebuild."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid
import zipfile

ROOT=Path(__file__).resolve().parents[3]
APP=ROOT/'apps/yolopv2_standalone'
sys.path.insert(0,str(APP))
from release_runtime import manifest_for, validate_manifest


def build_bundle(app, version, output):
    manifest=manifest_for(app,version)
    validate_manifest(manifest,manifest['runtime'])
    output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output,'w',compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('manifest.json',json.dumps(manifest,sort_keys=True))
        for name in manifest['files']:
            data=(app/name).read_bytes().replace(b'\r\n',b'\n')
            if name.endswith('.py'):compile(data,name,'exec')
            else:json.loads(data)
            archive.writestr(name,data)
    return manifest


class Remote:
    def __init__(self,cli,ip,app_id):
        self.cli,self.ip,self.app_id=cli,ip,app_id

    def run(self,code):
        encoded=base64.b64encode(code.encode()).decode()
        args=[self.cli,'app','exec','-d',self.ip,self.app_id,'--','python3.12','-c',
              "import base64;exec(base64.b64decode("+repr(encoded)+"))"]
        result=subprocess.run(args,input='',capture_output=True,text=True,encoding='utf-8',timeout=35)
        if result.returncode or 'Traceback (most recent call last)' in result.stdout+result.stderr:
            raise RuntimeError(result.stdout+result.stderr)
        return result.stdout

    def control(self,command,value=None):
        arguments=['release_runtime.py',command]+([value] if value is not None else [])
        output=self.run("import sys;sys.path.insert(0,'/app');import release_runtime;"
                        f"sys.argv={arguments!r};release_runtime.main()")
        if 'RELEASE_RESULT=' not in output:raise RuntimeError('Runtime launcher is missing; install runtime container first')
        return json.loads(output.split('RELEASE_RESULT=',1)[1].splitlines()[0])

    def restart(self):
        subprocess.run([self.cli,'app','restart','-d',self.ip,self.app_id],check=True,
                       input='',capture_output=True,text=True,timeout=90)

    def wait(self,release,timeout=120):
        deadline=time.monotonic()+timeout
        next_status=0
        while time.monotonic()<deadline:
            if time.monotonic()>=next_status:
                state=self.control('status')
                failure=state.get('last_failure') or ''
                if not state.get('pending') and release in failure:
                    raise RuntimeError('Automatic rollback completed: '+json.dumps(state))
                next_status=time.monotonic()+10
            try:
                with socket.create_connection((self.ip,8557),timeout=3) as connection:
                    connection.settimeout(3);connection.sendall(b'HELLO\n')
                    with connection.makefile('rb') as stream:
                        for _ in range(8):
                            frame=json.loads(stream.readline())
                            if 'frame_seq' in frame and frame.get('code_release')==release:
                                state=self.control('status')
                                if not state.get('pending') and (state['active'] or 'embedded')==release:
                                    return dict(state=state,frame=frame)
            except (OSError,ValueError):pass
            time.sleep(1)
        state=self.control('status')
        raise RuntimeError('Release not healthy; runtime may have rolled back: '+json.dumps(state))

    def deploy(self,bundle):
        state=self.control('status')
        data=bundle.read_bytes()
        with zipfile.ZipFile(bundle) as archive:manifest=json.loads(archive.read('manifest.json'))
        validate_manifest(manifest,state['runtime'])
        if state.get('pending'):raise ValueError('Another activation is pending')
        # Upload to app-specific persistent storage, never overwrite live sources.
        name='upload-'+uuid.uuid4().hex+'.zip'
        remote=str(Path(state['storage'])/name).replace('\\','/')
        self.run(f"from pathlib import Path;p=Path({remote!r});p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b'')")
        try:
            for offset in range(0,len(data),9000):
                chunk=base64.b64encode(data[offset:offset+9000]).decode()
                self.run(f"import base64;f=open({remote!r},'r+b');f.seek({offset});f.write(base64.b64decode({chunk!r}));f.close()")
            sha=hashlib.sha256(data).hexdigest()
            self.run(f"import hashlib;from pathlib import Path;assert hashlib.sha256(Path({remote!r}).read_bytes()).hexdigest()=={sha!r}")
            staged=self.control('stage',remote)
            if staged!=manifest:raise RuntimeError('Staged release identity mismatch')
        finally:
            self.run(f"from pathlib import Path;Path({remote!r}).unlink(missing_ok=True)")
        self.control('activate',manifest['release'])
        self.restart()
        return dict(bundle_bytes=len(data),bundle_sha256=sha,manifest=manifest,
                    **self.wait(manifest['release']))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('build','deploy','status','rollback'))
    parser.add_argument('--bundle',type=Path)
    parser.add_argument('--version')
    parser.add_argument('--device-ip',default='192.168.201.2')
    parser.add_argument('--app-id')
    parser.add_argument('--oakctl',default='C:/Program Files/oakctl/bin/oakctl.exe')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.command=='build':
        if not args.bundle or not args.version:parser.error('build requires --bundle and --version')
        result=dict(manifest=build_bundle(APP,args.version,args.bundle),bundle_bytes=args.bundle.stat().st_size)
    else:
        if not args.app_id:parser.error('An explicit installed runtime --app-id is required')
        remote=Remote(args.oakctl,args.device_ip,args.app_id)
        if args.command=='deploy':
            if not args.bundle:parser.error('deploy requires --bundle')
            result=remote.deploy(args.bundle)
        elif args.command=='rollback':
            state=remote.control('status')
            remote.control('rollback');remote.restart()
            result=remote.wait(state.get('previous') or 'embedded')
        else:result=remote.control('status')
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({key:value for key,value in result.items() if key not in ('frame','manifest')}))


if __name__=='__main__':main()
