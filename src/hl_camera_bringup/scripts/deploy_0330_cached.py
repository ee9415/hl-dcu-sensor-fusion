from pathlib import Path
import subprocess,base64,hashlib,json
root=Path.cwd();cmd=['C:/Program Files/oakctl/bin/oakctl.exe','app','exec','-d','192.168.201.2','00000000-0000-0000-0000-000000000000','--']
def run(code):
 encoded=base64.b64encode(code.encode()).decode()
 r=subprocess.run(cmd+['python3.12','-c',"import base64;exec(base64.b64decode('"+encoded+"'))"],capture_output=True,text=True,encoding='utf-8');r.check_returncode();return r.stdout
for name in ['image_boundaries.py','lane_boundaries.py']:
 data=(root/'apps/yolopv2_standalone'/name).read_bytes();digest=hashlib.sha256(data).hexdigest()
 run("from pathlib import Path;Path('/app/"+name+".new').write_bytes(b'')")
 for i in range(0,len(data),7000):
  chunk=base64.b64encode(data[i:i+7000]).decode()
  run("import base64;f=open('/app/"+name+".new','ab');f.write(base64.b64decode('"+chunk+"'));f.close()")
 code="from pathlib import Path;import hashlib;p=Path('/app/"+name+"');n=Path(str(p)+'.new');assert hashlib.sha256(n.read_bytes()).hexdigest()=='"+digest+"';compile(n.read_bytes(),str(p),'exec');p.with_suffix('.pre0330').write_bytes(p.read_bytes());n.replace(p);print('verified "+name+" "+digest+"')"
 print(run(code))
subprocess.run(cmd+['sv','restart','/etc/service/backend'],check=True)
