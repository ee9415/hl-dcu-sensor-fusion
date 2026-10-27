"""Install the PC input path into a running development app with source backups."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-ip',required=True)
    parser.add_argument('--app-id',default='00000000-0000-0000-0000-000000000000')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    command=['C:/Program Files/oakctl/bin/oakctl.exe','app','exec','-d',args.device_ip,args.app_id,'--']
    def run(code):
        payload=base64.b64encode(code.encode()).decode()
        result=subprocess.run(command+['python3.12','-c',f"import base64;exec(base64.b64decode('{payload}'))"],
                              capture_output=True,text=True,encoding='utf-8')
        result.check_returncode()
        if 'Traceback (most recent call last)' in result.stdout:
            raise RuntimeError(result.stdout)
        return result.stdout
    backup='/app/video_input_backup_'+uuid.uuid4().hex[:10]
    files=['main.py','yolopv2_node.py','video_transport.py','video_input_node.py']
    run(f"from pathlib import Path;Path('{backup}').mkdir()")
    report=dict(device=args.device_ip,app_id=args.app_id,backup=backup,files={})
    for name in files:
        data=(root/'apps/yolopv2_standalone'/name).read_bytes()
        digest=hashlib.sha256(data).hexdigest()
        result=run(f"from pathlib import Path;import hashlib;p=Path('/app/{name}');b=Path('{backup}/{name}');"
                   "b.write_bytes(p.read_bytes()) if p.exists() else None;"
                   f"Path('/app/{name}.video_new').write_bytes(b'');"
                   "print('PREVIOUS='+ (hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else 'absent'))")
        previous=result.split('PREVIOUS=',1)[1].splitlines()[0]
        for start in range(0,len(data),7000):
            chunk=base64.b64encode(data[start:start+7000]).decode()
            run(f"import base64;f=open('/app/{name}.video_new','ab');f.write(base64.b64decode('{chunk}'));f.close()")
        run(f"from pathlib import Path;import hashlib;p=Path('/app/{name}.video_new');"
            f"assert hashlib.sha256(p.read_bytes()).hexdigest()=='{digest}';compile(p.read_bytes(),str(p),'exec')")
        report['files'][name]=dict(previous=previous,sha256=digest)
    # Replace only after all uploaded modules compile and match their hashes.
    run("from pathlib import Path\n"+''.join(f"Path('/app/{n}.video_new').replace('/app/{n}')\n" for n in files))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'deployment.json').write_text(json.dumps(report,indent=2))
    subprocess.run(command+['sv','restart','/etc/service/backend'],check=True)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
