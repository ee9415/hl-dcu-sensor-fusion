"""Render actual recorded estimates beside their synchronized tracker RGB input."""
import json,sys
from pathlib import Path
import cv2
import numpy as np
folder=Path(sys.argv[1]);z=np.load(folder/'boundary_debug.npz');records=json.loads(str(z['records_json']))
tiles=[];frames=[];previous=None
for i,r in enumerate(records):
    u=next(c for c in r['calls'] if c['name']=='update');rgb=z[u['image']['array']]
    t=r['telemetry'];t=dict(t,frame=i,timestamp=u['timestamp_s'],rgb_change=float(np.mean(abs(rgb.astype(float)-previous))) if previous is not None else None)
    previous=rgb.astype(float);frames.append(t)
    if i not in np.linspace(0,len(records)-1,6).astype(int):continue
    original=cv2.resize(rgb,(640,360));overlay=original.copy()
    for key,color in [('left_boundary_image',(0,255,255)),('right_boundary_image',(255,180,0))]:
        p=np.array(t[key])
        if len(p)>1:cv2.polylines(overlay,[np.rint(p*2).astype(np.int32)],False,color,2)
    cv2.putText(overlay,f'f{i}: {t["left_boundary_state"]}/{t["right_boundary_state"]}',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
    tiles.append(np.hstack((original,overlay)))
cv2.imwrite(str(folder/'actual_frames.jpg'),np.vstack(tiles))
(folder/'actual_frames.jsonl').write_text('\n'.join(json.dumps(t) for t in frames))
print(json.dumps(dict(frames=len(frames),mean_image_change=float(np.mean([t['rgb_change'] for t in frames[1:]])))))
