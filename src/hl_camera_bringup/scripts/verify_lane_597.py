"""Offline FP32 model and current contour tracker around requested 59.7s; no device IO."""
from pathlib import Path
import sys,json,time,argparse
from collections import Counter
import cv2,numpy as np,onnxruntime as ort
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from lane_contours import extract_lane_contours
from lane_overlay import overlay_tracked_skeleton
from postprocess import segmentation_masks
from skeleton_response import response_config
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--speed',type=float,default=1.);parser.add_argument('--output',type=Path);args=parser.parse_args()
OUT=ROOT/'artifacts/lane_597_20261110'
if args.speed != 1.:OUT=OUT/f'speed{args.speed:g}'
if args.output is not None:OUT=args.output
OUT.mkdir(parents=True,exist_ok=True)
VIDEO=ROOT/'artifacts/recordings/six_camera/six_camera_20261104_073117_05e93c/cam01/cam_01_rgb_20261104_073117.mp4'
MODEL=Path('/mnt/d/git_hub/OAK_4D_PRO/apps/yolopv2_standalone/models/yolopv2_person/yolopv2_person_320.onnx')
options=ort.SessionOptions();options.intra_op_num_threads=4
session=ort.InferenceSession(str(MODEL),sess_options=options,providers=['CPUExecutionProvider'])
cap=cv2.VideoCapture(str(VIDEO)); assert cap.isOpened()
trackers={0:ImageBoundaryTracker(),50:ImageBoundaryTracker(),58:ImageBoundaryTracker()}
for tracker in trackers.values():tracker.config=response_config(tracker.config,args.speed)
records=[]; counts={k:Counter() for k in trackers}; began=time.monotonic()
targets={580,590,597,600,610,620};cache=ROOT/'artifacts/lane_597_20261110/cache';cache.mkdir(exist_ok=True)
for i in range(631):
    ok,frame=cap.read();assert ok
    t=i/10
    file=cache/f'{i:04d}.npz'
    if file.exists():
        z=np.load(file);lane=z['lane'];road=z['road']
    else:
        small=cv2.resize(frame,(320,180))
        inp=np.zeros((320,320,3),np.uint8);inp[70:250]=small[:,:,::-1]
        arr=np.ascontiguousarray(inp.transpose(2,0,1)[None],dtype=np.float32)/255
        rd,ll=session.run(['drivable_area','lane_line'],{'images':arr})
        road,lane=segmentation_masks(rd,ll,(1920,1080))
        np.savez_compressed(file,road=road,lane=lane)
    contour=extract_lane_contours(lane)
    for start,tracker in trackers.items():
        if t<start:continue
        est=tracker.update_contours(contour,lane.shape,t,rgb=frame)
        if t>=57:
            rec={'time':t,'history_start':start,'lane_pixels':int(np.count_nonzero(lane)),
                 'pieces':len(contour.pieces),'groups':len(contour.group_curves),
                 'states':[est.left_state,est.right_state], 'diagnostics':est.detection_diagnostics,
                 'left':est.image_left.tolist(),'right':est.image_right.tolist()}
            records.append(rec);counts[start].update(['/'.join(rec['states'])])
        if i in targets:
            base=cv2.resize(frame,(960,540));mask=base.copy()
            enlarged=cv2.resize(lane,(960,540),interpolation=cv2.INTER_NEAREST)>0
            mask[enlarged]=(0,255,255)
            tracked=base.copy();overlay_tracked_skeleton(tracked,est,lane.shape)
            cv2.putText(mask,f'RAW MASK t={t:.1f}s',(12,24),0,.65,(0,255,255),2)
            cv2.putText(tracked,f'TRACK start={start}s t={t:.1f}s {est.left_state}/{est.right_state}',(12,24),0,.6,(0,255,255),2)
            cv2.imwrite(str(OUT/f't{i}_start{start}.jpg'),np.hstack((mask,tracked)))
            if i==597 and start==0:
                cv2.imwrite(str(OUT/'source_597.jpg'),base)
                np.savez_compressed(OUT/'frame_597.npz',rgb=cv2.resize(frame,(320,180)),lane=lane,road=road)
    if i%50==0:print(f'processed {t:.1f}s elapsed={time.monotonic()-began:.1f}s',flush=True)
cap.release()
(OUT/'records.json').write_text(json.dumps(records,allow_nan=True,indent=2))
summary={'video':str(VIDEO),'model':str(MODEL),'kind':'offline ONNX FP32, black letterbox, current contour tracker; speed recorded separately; not device FP16 or ground-truth accuracy','speed':args.speed,'counts':{k:dict(v) for k,v in counts.items()},'frames':631,'elapsed':time.monotonic()-began}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
