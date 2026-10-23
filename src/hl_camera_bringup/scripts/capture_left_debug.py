import subprocess,base64,sys,json
from pathlib import Path
root=Path(__file__).resolve().parents[3]
out=root/(sys.argv[2] if len(sys.argv)>2 else 'artifacts/left_debug_0322');out.mkdir(parents=True,exist_ok=True)
cmd=['C:/Program Files/oakctl/bin/oakctl.exe','app','exec','-d','192.168.201.2','00000000-0000-0000-0000-000000000000','--']
def run(code):
    encoded=base64.b64encode(code.encode()).decode()
    r=subprocess.run(cmd+['python3.12','-c',"import base64;exec(base64.b64decode('"+encoded+"'))"],capture_output=True,text=True,encoding='utf-8')
    if r.returncode:raise RuntimeError(r.stderr+r.stdout)
    return r.stdout
mode=sys.argv[1]
if mode in ('install','timing-install'):
    source='boundary_timing_capture.py' if mode=='timing-install' else 'boundary_debug_capture.py'
    probe=(root/'src/hl_camera_bringup/scripts'/source).read_text()
    count=min(100,max(1,int(sys.argv[3]))) if len(sys.argv)>3 else 10
    probe=probe.replace("'0'))))", "'"+str(count)+"'))))").replace("'60')))","'30')))")
    probe=probe.replace("'_fuse')", "'_fuse','_detect_full','_detect_legacy','_detect_regions')")
    code="from pathlib import Path; import base64\n"
    code+="Path('/app/boundary_debug.npz').unlink(missing_ok=True)\n"
    code+="p=Path('/app/yolopv2_node.py'); b=Path('/app/yolopv2_node.debug_backup'); assert not b.exists(); b.write_bytes(p.read_bytes())\n"
    code+="Path('/app/boundary_debug_capture.py').write_bytes(base64.b64decode('"+base64.b64encode(probe.encode()).decode()+"'))\n"
    code+="s=p.read_text(); s=s.replace('boundaries = self._boundary_tracker.update(', 'boundaries = __import__(\"boundary_debug_capture\").update(self._boundary_tracker,'); p.write_text(s)\n"
    run(code)
    subprocess.run(cmd+['sv','restart','/etc/service/backend'],check=True)
elif mode in ('export','timing-export'):
    names=['boundary_timing.json'] if mode=='timing-export' else ['boundary_debug.npz','image_boundaries.py','lane_boundaries.py','lane_boundaries.json']
    for name in names:
        data=run("from pathlib import Path;import base64; print('PAYLOAD='+base64.b64encode(Path('/app/"+name+"').read_bytes()).decode())")
        payload=data.split('PAYLOAD=',1)[1].splitlines()[0].strip()
        (out/name).write_bytes(base64.b64decode(payload))
    print('captured')
elif mode=='restore':
    run("from pathlib import Path;p=Path('/app/yolopv2_node.py');b=Path('/app/yolopv2_node.debug_backup');p.write_bytes(b.read_bytes());b.unlink();Path('/app/boundary_debug_capture.py').unlink()")
    subprocess.run(cmd+['sv','restart','/etc/service/backend'],check=True)
