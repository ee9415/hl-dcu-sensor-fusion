"""Render source-time matched device boundary coordinates (not RTSP frame matching)."""
from pathlib import Path
import sys,json,argparse
import cv2,numpy as np
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from lane_overlay import overlay_tracked_skeleton
p=argparse.ArgumentParser();p.add_argument('--name',default='device55');p.add_argument('--folder',type=Path);a=p.parse_args();folder=a.folder if a.folder else ROOT/'artifacts/lane_597_20261110'/a.name
rows=[json.loads(l) for l in (folder/'frames.jsonl').read_text().splitlines()]
rows=[r for r in rows if r.get('source_media_time_s') is not None]
cap=cv2.VideoCapture(str(ROOT/'artifacts/recordings/six_camera/six_camera_20261104_073117_05e93c/cam01/cam_01_rgb_20261104_073117.mp4'))
for target in [55,57,57.6,58,58.1,58.3,58.5,58.6,59,59.3,59.5,59.6,59.7,59.8,60,60.1,61,62]:
 r=min(rows,key=lambda r:abs(r['source_media_time_s']-target));t=r['source_media_time_s']
 cap.set(cv2.CAP_PROP_POS_FRAMES,round(t*10));ok,frame=cap.read();assert ok
 img=cv2.resize(frame,(960,540))
 if 'left_boundary_supported_image' in r:
  yrows=ImageBoundaryTracker().rows
  values={}
  for side in ['left','right']:
   coordinates={round(y,6):x for x,y in r[side+'_boundary_image']}
   supported={round(y,6) for x,y in r[side+'_boundary_supported_image']}
   values['image_'+side]=np.column_stack(([coordinates.get(round(y,6),np.nan) for y in yrows],yrows))
   values[side+'_support']=np.array([round(y,6) in supported for y in yrows])
   values[side+'_state']=r[side+'_boundary_state']
  overlay_tracked_skeleton(img,SimpleNamespace(**values),(180,320))
 else:
  for side in ['left','right']:
   pts=np.asarray(r[side+'_boundary_image'],dtype=float)*3
   for j,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
    if np.isfinite([a,b]).all():cv2.line(img,tuple(np.rint(a).astype(int)),tuple(np.rint(b).astype(int)),(255,0,255),3)
 cv2.putText(img,f'DEVICE XY t={t:.1f}s {r["left_boundary_state"]}/{r["right_boundary_state"]}',(10,25),0,.65,(0,255,255),2)
 cv2.imwrite(str(folder/f'xy_{t:g}.jpg'),img)
cap.release()
