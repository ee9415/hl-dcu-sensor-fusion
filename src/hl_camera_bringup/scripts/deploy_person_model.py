"""Upload a validated local RVC4 archive to the existing OAK development app.

Uses a temporary, token-protected device upload endpoint, verifies SHA256 before
replacement, backs up main.py and any previous person archive, then restarts only
the backend. Existing baseline archive and other app sources stay in place.
"""
import argparse
import base64
import hashlib
import http.client
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import time
import uuid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('archive',type=Path)
    p.add_argument('--device-ip',required=True)
    p.add_argument('--app-id',default='00000000-0000-0000-0000-000000000000')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    root=Path(__file__).resolve().parents[3]
    data=args.archive.read_bytes();digest=hashlib.sha256(data).hexdigest()
    with tarfile.open(args.archive,'r:xz') as archive:
        config=json.load(archive.extractfile('config.json'))
    model=config['model']
    assert model['metadata']['name']=='yolopv2_person_320'
    assert [(v['name'],v['shape']) for v in model['inputs']]==[('images',[1,320,320,3])]
    expected={'det_stride_8':[1,40,40,255],'det_stride_16':[1,20,20,255],
              'det_stride_32':[1,10,10,255],'drivable_area':[1,320,320,2],
              'lane_line':[1,320,320,1]}
    assert {v['name']:v['shape'] for v in model['outputs']}==expected
    local=root/'apps/yolopv2_standalone/models/yolopv2_person_320.rvc4.tar.xz'
    shutil.copy2(args.archive,local)
    command=['C:/Program Files/oakctl/bin/oakctl.exe','app','exec','-d',args.device_ip,args.app_id,'--']
    def run(code):
        encoded=base64.b64encode(code.encode()).decode()
        r=subprocess.run(command+['python3.12','-c',f"import base64;exec(base64.b64decode('{encoded}'))"],
                         capture_output=True,text=True,encoding='utf-8',timeout=180)
        r.check_returncode()
        if 'Traceback (most recent call last)' in r.stdout+r.stderr:
            raise RuntimeError(r.stdout+r.stderr)
        return r.stdout
    token=uuid.uuid4().hex
    backup='/app/person_model_backup_'+token[:10]
    remote='/app/models/yolopv2_person_320.rvc4.tar.xz'
    run(f"from pathlib import Path;import shutil,json,hashlib;p=Path('{backup}');p.mkdir();"
        "shutil.copy2('/app/main.py',p/'main.py');"
        f"shutil.copy2('{remote}',p/'model.tar.xz') if Path('{remote}').exists() else None;"
        "print('BACKUP_READY')")
    server=f'''
from http.server import HTTPServer,BaseHTTPRequestHandler
from pathlib import Path
import hashlib,json,time
target=Path({remote!r}); temporary=target.with_suffix('.upload')
class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path!='/'+{token!r} or self.headers.get('Content-Length')!={str(len(data))!r}:
            self.send_error(403);return
        self.connection.settimeout(15)
        try:
            remaining={len(data)}; h=hashlib.sha256()
            with temporary.open('wb') as output:
                while remaining:
                    chunk=self.rfile.read(min(1024*1024,remaining))
                    if not chunk:raise ValueError('Truncated model')
                    output.write(chunk);h.update(chunk);remaining-=len(chunk)
            if h.hexdigest()!={digest!r}:raise ValueError('Model checksum mismatch')
            temporary.replace(target)
            Path({backup!r}+'/upload.json').write_text(json.dumps(dict(sha256=h.hexdigest(),bytes={len(data)})))
            self.send_response(200);self.end_headers();self.wfile.write(b'OK')
            self.server.finished=True
        except Exception as exc:
            temporary.unlink(missing_ok=True);self.send_error(400,str(exc))
    def log_message(self,*args):pass
server=HTTPServer(('0.0.0.0',8559),Handler);server.timeout=1;server.finished=False
deadline=time.monotonic()+120
while not server.finished and time.monotonic()<deadline:server.handle_request()
server.server_close()
'''
    payload=base64.b64encode(server.encode()).decode()
    run("import subprocess,base64;"
        f"subprocess.Popen(['python3.12','-c',base64.b64decode('{payload}').decode()],"
        "stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)")
    for attempt in range(20):
        connection=http.client.HTTPConnection(args.device_ip,8559,timeout=120)
        try:
            connection.connect();break
        except OSError:
            connection.close()
            if attempt==19:raise
            time.sleep(.2)
    try:
        connection.request('POST','/'+token,body=data,headers={'Content-Type':'application/octet-stream'})
        response=connection.getresponse();body=response.read()
        if response.status!=200:raise RuntimeError(body.decode(errors='replace'))
    finally:connection.close()
    main_source=(root/'apps/yolopv2_standalone/main.py').read_bytes()
    encoded=base64.b64encode(main_source).decode()
    verified=run(f"from pathlib import Path;import hashlib,base64,json;"
                 f"assert hashlib.sha256(Path('{remote}').read_bytes()).hexdigest()=='{digest}';"
                 f"data=base64.b64decode('{encoded}');compile(data,'main.py','exec');"
                 "Path('/app/main.py.person_new').write_bytes(data);"
                 "Path('/app/main.py.person_new').replace('/app/main.py');"
                 f"print('VERIFIED='+json.dumps(dict(archive='{remote}',sha256='{digest}',backup='{backup}')))")
    report=dict(archive=str(args.archive),sha256=digest,bytes=len(data),device=args.device_ip,
                backup=backup,remote=remote,main_sha256=hashlib.sha256(main_source).hexdigest(),verified=verified)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'deployment.json').write_text(json.dumps(report,indent=2))
    subprocess.run(command+['sv','restart','/etc/service/backend'],check=True)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
