"""Inspect cached masks and selected source frames without device IO."""
from pathlib import Path
import sys,json,argparse
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from lane_contours import extract_lane_contours
from lane_overlay import overlay_tracked_skeleton
from skeleton_response import response_config
p=argparse.ArgumentParser();p.add_argument('--name',default='before');a=p.parse_args()
out=ROOT/'artifacts/curvature_20261110'/a.name;out.mkdir(parents=True,exist_ok=True)
video=ROOT/'artifacts/recordings/six_camera/six_camera_20261104_073117_05e93c/cam01/cam_01_rgb_20261104_073117.mp4'
cap=cv2.VideoCapture(str(video));tracker=ImageBoundaryTracker();tracker.config=response_config(tracker.config,3.)
records=[]
for i in range(501):
 z=np.load(ROOT/f'artifacts/lane_597_20261110/cache/{i:04d}.npz');lane=z['lane'];r=extract_lane_contours(lane)
 est=tracker.update_contours(r,lane.shape,i/10)
 if i>=410:records.append(dict(time=i/10,**est.telemetry()))
 if i in [420,440,450,453,460,466,480]:
  cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,frame=cap.read();assert ok
  base=cv2.resize(frame,(960,540));mask=base.copy();mask[cv2.resize(lane,(960,540),interpolation=cv2.INTER_NEAREST)>0]=(0,255,255)
  overlay_tracked_skeleton(base,est,lane.shape)
  cv2.putText(base,f'{i/10:.1f} {est.left_state}/{est.right_state}',(10,25),0,.7,(0,255,255),2)
  cv2.imwrite(str(out/f'frame_{i}.jpg'),np.hstack((mask,base)))
  (out/f'curves_{i}.json').write_text(json.dumps([c.tolist() for c in r.group_curves]))
(out/'records.json').write_text(json.dumps(records))
print(json.dumps({'frames':501,'output':str(out)}))
