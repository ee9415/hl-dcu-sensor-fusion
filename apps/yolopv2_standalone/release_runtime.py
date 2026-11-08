"""Immutable runtime launcher and transactional code releases (stdlib only)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import zipfile
from functools import wraps

SCHEMA = 1
MAX_BYTES = 8*1024*1024
RUNTIME_FILES = ('runtime-base.toml', 'requirements.txt', 'install-runtime.sh', 'backend-run.sh', 'release_runtime.py')
EXCLUDED_CODE = {'release_runtime.py', 'inspect_lane_contours.py'}


def serialized(method):
    @wraps(method)
    def wrapped(self,*args,**kwargs):
        import fcntl
        self.root.mkdir(parents=True,exist_ok=True)
        with (self.root/'update.lock').open('a+b') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            return method(self,*args,**kwargs)
    return wrapped


def digest(data):
    return hashlib.sha256(data).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':')).encode()


def contract(app):
    # Container version may change independently; runtime requirements may not.
    base = (app/'runtime-base.toml').read_text().split('[base_image]',1)[1].strip()
    # oakctl omits oakapp.toml from runtime containers. Check agreement on the PC.
    configuration=app/'oakapp.toml'
    if configuration.exists() and configuration.read_text().split('[base_image]',1)[1].strip()!=base:
        raise ValueError('Synchronize runtime-base.toml with oakapp.toml before building')
    return digest(canonical(dict(base_image=base, files={name:digest(
        (app/name).read_bytes().replace(b'\r\n', b'\n')) for name in RUNTIME_FILES})))


def code_files(app):
    return sorted(p.name for p in app.iterdir() if p.is_file() and
                  p.suffix in ('.py', '.json') and p.name not in EXCLUDED_CODE)


def manifest_for(app, version):
    files={name:digest((app/name).read_bytes().replace(b'\r\n', b'\n')) for name in code_files(app)}
    identity=dict(schema=SCHEMA, version=version, runtime=contract(app), files=files)
    return dict(identity, release=digest(canonical(identity)))


def validate_manifest(manifest, runtime):
    identity={key:manifest[key] for key in ('schema','version','runtime','files')}
    if manifest['schema']!=SCHEMA or manifest['runtime']!=runtime:
        raise ValueError('Runtime mismatch: install a matching runtime container first')
    if digest(canonical(identity))!=manifest['release']:
        raise ValueError('Invalid release identity')
    files=manifest['files']
    if not isinstance(files,dict) or not {'main.py','telemetry_server.py'}<=files.keys():
        raise ValueError('Incomplete code release')
    for name, sha in files.items():
        if (not re.fullmatch(r'[a-zA-Z0-9_]+\.(py|json)',name) or
                name in EXCLUDED_CODE or not re.fullmatch('[0-9a-f]{64}',sha)):
            raise ValueError('Invalid code file')


def validate_directory(folder, runtime):
    manifest=json.loads((folder/'manifest.json').read_text())
    validate_manifest(manifest,runtime)
    for name,sha in manifest['files'].items():
        data=(folder/name).read_bytes()
        if digest(data)!=sha:raise ValueError('Code hash mismatch: '+name)
        if name.endswith('.py'):compile(data,name,'exec')
        else:json.loads(data)
    return manifest


def atomic_json(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.new')
    with temp.open('wb') as stream:
        stream.write(canonical(value));stream.flush();os.fsync(stream.fileno())
    os.replace(temp,path)
    fd=os.open(path.parent,os.O_RDONLY)
    try:os.fsync(fd)
    finally:os.close(fd)


class ReleaseStore:
    def __init__(self, root, app):
        self.root,self.app=Path(root),Path(app)
        self.runtime=contract(self.app)

    def read_state(self):
        path=self.root/'state.json'
        return json.loads(path.read_text()) if path.exists() else dict(active=None,previous=None,pending=None)

    def write_state(self,state):
        atomic_json(self.root/'state.json',state)

    def folder(self, release):
        if not isinstance(release,str) or not re.fullmatch('[0-9a-f]{64}',release):
            raise ValueError('Invalid release ID')
        return self.root/'releases'/release

    def stage(self,archive):
        if Path(archive).stat().st_size>MAX_BYTES:raise ValueError('Code bundle too large')
        self.root.mkdir(parents=True,exist_ok=True)
        temp=Path(tempfile.mkdtemp(prefix='staging-',dir=self.root))
        try:
            with zipfile.ZipFile(archive) as bundle:
                infos=bundle.infolist()
                if sum(i.file_size for i in infos)>MAX_BYTES:raise ValueError('Expanded bundle too large')
                names=[i.filename for i in infos]
                manifest=json.loads(bundle.read('manifest.json'))
                validate_manifest(manifest,self.runtime)
                if len(names)!=len(set(names)) or set(names)!=set(manifest['files'])|{'manifest.json'}:
                    raise ValueError('Unexpected or duplicate archive members')
                for name in names:
                    data=bundle.read(name)
                    with (temp/name).open('wb') as stream:
                        stream.write(data);stream.flush();os.fsync(stream.fileno())
            validate_directory(temp,self.runtime)
            target=self.folder(manifest['release'])
            target.parent.mkdir(exist_ok=True)
            if target.exists():validate_directory(target,self.runtime)
            else:
                os.replace(temp,target)
                fd=os.open(target.parent,os.O_RDONLY)
                try:os.fsync(fd)
                finally:os.close(fd)
            return manifest
        finally:
            if temp.exists():shutil.rmtree(temp)

    @serialized
    def activate(self,release):
        validate_directory(self.folder(release),self.runtime)
        state=self.read_state()
        if state.get('pending'):raise ValueError('Another activation is pending')
        if state['active']==release:return
        state['pending']=dict(release=release,attempted=False)
        self.write_state(state)

    @serialized
    def rollback(self):
        state=self.read_state()
        if state.get('pending'):raise ValueError('Wait for pending activation before rollback')
        # None is the embedded recovery code, and is a valid previous release.
        target=state.get('previous')
        if target:validate_directory(self.folder(target),self.runtime)
        state.update(active=target,previous=state['active'],pending=None)
        self.write_state(state)

    def status(self):
        state=self.read_state()
        return dict(runtime=self.runtime,storage=str(self.root),**state)


def healthy(release, timeout=2):
    try:
        with socket.create_connection(('127.0.0.1',8557),timeout=timeout) as connection:
            connection.settimeout(timeout);connection.sendall(b'HELLO\n')
            with connection.makefile('rb') as stream:
                for _ in range(8):
                    frame=json.loads(stream.readline())
                    if 'frame_seq' in frame:
                        return frame.get('code_release')==release and frame.get('processing_mode')=='full'
    except (OSError,ValueError):pass
    return False


def stop_child(child):
    if child.poll() is not None:return
    os.killpg(child.pid,signal.SIGTERM)
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid,signal.SIGKILL);child.wait()


def launch(store):
    state=store.read_state()
    pending=state.get('pending')
    if pending and pending['attempted']:
        state['pending']=None
        state['last_failure']='Unconfirmed release interrupted; reverted to previous code'
        store.write_state(state);pending=None
    candidate=pending['release'] if pending else state['active']
    try:
        if candidate:validate_directory(store.folder(candidate),store.runtime)
    except (OSError,ValueError,KeyError) as exc:
        state.update(pending=None,last_failure=str(exc))
        if pending:
            store.write_state(state)
            return launch(store)
        state['active']=None
        store.write_state(state);candidate=None
    if pending:
        state['pending']['attempted']=True;store.write_state(state)
    folder=store.folder(candidate) if candidate else store.app
    env=dict(os.environ,OAK_CODE_RELEASE=candidate or 'embedded',OAK_RUNTIME_FINGERPRINT=store.runtime)
    env.setdefault('YOLOPV2_MODEL_ARCHIVE',str(store.app/'models/yolopv2_person_320.rvc4.tar.xz'))
    child=subprocess.Popen([sys.executable,'-u',str(folder/'main.py')],cwd=folder,env=env,start_new_session=True)
    def terminate(signum,frame):
        stop_child(child)
        raise SystemExit(128+signum)
    signal.signal(signal.SIGTERM,terminate)
    signal.signal(signal.SIGINT,terminate)
    try:
        if pending:
            deadline=time.monotonic()+90
            while child.poll() is None and time.monotonic()<deadline:
                if healthy(candidate):
                    state.update(previous=state['active'],active=candidate,pending=None,last_failure=None)
                    store.write_state(state)
                    break
                time.sleep(1)
            else:
                stop_child(child)
                state.update(pending=None,last_failure='Candidate failed startup health check: '+candidate)
                store.write_state(state)
                return launch(store)
        return child.wait()
    finally:stop_child(child)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('launch','stage','activate','rollback','status'))
    parser.add_argument('value',nargs='?')
    args=parser.parse_args()
    app=Path(__file__).resolve().parent
    storage=os.environ.get('OAKAGENT_STORAGE_APP')
    if not storage:raise RuntimeError('OAK app persistent storage is required')
    store=ReleaseStore(Path(storage)/'code-deploy',app)
    if args.command=='launch':raise SystemExit(launch(store))
    if args.command=='stage':result=store.stage(args.value)
    elif args.command=='activate':store.activate(args.value);result=store.status()
    elif args.command=='rollback':store.rollback();result=store.status()
    else:result=store.status()
    print('RELEASE_RESULT='+json.dumps(result))


if __name__=='__main__':main()
