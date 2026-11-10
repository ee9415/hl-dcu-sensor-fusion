"""Real camera replay diagnosis; explicitly disconnect prior publisher, preserve display options."""
import sys,socket,json,time,threading,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'tools'))
import cv2
from video_file_sender import VideoFileSender,select_camera
from oak_recorder import CaptureSource
from frame_source_clock import SourceClock
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--start',type=float,default=55);p.add_argument('--end',type=float,default=64);p.add_argument('--name',default='device55');p.add_argument('--record-output',action='store_true');p.add_argument('--output-base',type=Path,default=ROOT/'artifacts/lane_597_20261110');a=p.parse_args()
OUT=a.output_base/a.name;OUT.mkdir(parents=True,exist_ok=True)
targets=[45.,45.3,46.,46.6,55.,58.,58.5,59.7,60.,378.,428.]
rows=[];errors=[];done=threading.Event()
before=select_camera('192.168.201.2');(OUT/'selected_camera.json').write_text(json.dumps(before,indent=2))
def receive():
 try:
  with socket.create_connection(('192.168.201.2',8557),timeout=6) as sock:
   sock.settimeout(1);sock.sendall(b'HELLO\n');pending=b''
   with (OUT/'frames.jsonl').open('w') as f:
    while not done.is_set():
     try:data=sock.recv(1048576)
     except TimeoutError:continue
     if not data:break
     pending+=data
     while b'\n' in pending:
      line,pending=pending.split(b'\n',1);r=json.loads(line)
      if 'frame_seq' in r:rows.append(r);f.write(json.dumps(r)+'\n');f.flush()
 except Exception as e:errors.append(str(e))
thread=threading.Thread(target=receive,daemon=True);thread.start()
sources={k:CaptureSource(k,k,f'rtsp://192.168.201.2:{port}/{path}') for k,port,path in [('rgb',8555,'rgb'),('result',8554,'yolopv2'),('top',8553,'bev')]}
sender=None;images=[];writers={};next_write=0.;clock=SourceClock();lossless={}
try:
 for source in sources.values():source.start()
 deadline=time.monotonic()+35
 while time.monotonic()<deadline and not all(s.snapshot()[0] for s in sources.values()):time.sleep(.2)
 print('streams warm', {k:s.info() for k,s in sources.items()},flush=True)
 video=ROOT/'artifacts/recordings/six_camera/six_camera_20261104_073117_05e93c/cam01/cam_01_rgb_20261104_073117.mp4'
 sender=VideoFileSender('192.168.201.2',video,loop=False,fps=10,start_s=a.start,end_s=a.end);sender.start();began=time.monotonic();saved=set()
 while time.monotonic()-began<a.end-a.start+15:
  time.sleep(.03)
  if a.record_output and time.monotonic()>=next_write:
   for key in ('result','top'):
    ok,img=sources[key].snapshot()
    if ok and img is not None:
     decoded=clock.read(img,12 if key=='result' else 10)
     if decoded is not None:
      actual,score=decoded
      known=any(abs(float(r.get('source_media_time_s') or -100)-actual)<1e-6 for r in rows)
      if known:
       for target in targets:
        ident=key+'_'+str(target);rank=(abs(actual-target),score)
        if rank[0]<=.3 and (ident not in lossless or rank<tuple(lossless[ident]['rank'])):
         name=f'lossless_{key}_{target:g}.png';cv2.imwrite(str(OUT/name),img)
         lossless[ident]={'target':target,'source_time':actual,'rank':rank,'path':name}
     if key not in writers:
      writers[key]=cv2.VideoWriter(str(OUT/(key+'_actual.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),10.,(img.shape[1],img.shape[0]))
     writers[key].write(img)
   next_write=time.monotonic()+.1
  if rows:
   r=rows[-1];t=r.get('source_media_time_s')
   if t is not None:
    for target in targets:
     if target not in saved and target<=t<target+.3:
      saved.add(target);paths={}
      for k,s in sources.items():
       ok,img=s.snapshot()
       if ok and img is not None:
        name=f'{k}_{target:g}.jpg';cv2.imwrite(str(OUT/name),img);paths[k]=name
      images.append({'target':target,'telemetry':r,'paths':paths})
  if sender.thread and not sender.thread.is_alive():
   if not a.record_output:break
   if time.monotonic()-began>a.end-a.start+8:break
 time.sleep(.5)
finally:
 if sender:sender.close()
 done.set();thread.join(3)
 for s in sources.values():s.close()
 for writer in writers.values():writer.release()
 report={'start':a.start,'end':a.end,'sender':sender.snapshot() if sender else None,'errors':errors,'frames':len(rows),'lossless':lossless,'images':images,'streams':{k:s.info() for k,s in sources.items()}}
 (OUT/'report.json').write_text(json.dumps(report,indent=2,default=str))
 print(json.dumps({k:v for k,v in report.items() if k not in ('images','streams')},ensure_ascii=False),flush=True)
pc=[r for r in rows if r.get('source_media_time_s') is not None]
for target in [58,59,59.7,60,61,62]:
 if not pc:continue
 r=min(pc,key=lambda r:abs(r['source_media_time_s']-target))
 print(json.dumps({k:r.get(k) for k in ['source_media_time_s','code_release','skeleton_response_speed','left_boundary_state','right_boundary_state','boundary_detection_diagnostics']},ensure_ascii=False),flush=True)
if not pc or (sender and sender.snapshot().get('error')):raise RuntimeError('Device replay failed; inspect report')
