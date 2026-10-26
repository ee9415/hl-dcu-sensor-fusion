"""Diagnose paint rejection on captured monitor input, without changing device state."""
import json,sys
from pathlib import Path
import numpy as np
import cv2
root=Path(__file__).resolve().parents[3];sys.path.insert(0,str(root/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from lane_boundaries import white_lane_evidence
folder=Path(sys.argv[1]);z=np.load(folder/'boundary_debug.npz');records=json.loads(str(z['records_json']))
reports=[];tiles=[]
for i in (0,20,40,60,80,99):
    r=records[i];u=next(c for c in r['calls'] if c['name']=='update')
    def get(k):return z[u[k]['array']]
    image=get('image');lane=get('lane');paint=get('paint');allowed=get('allowed');brightness=image.mean(2)
    raw=white_lane_evidence(image,lane)>0
    report=dict(frame=i,threshold=u['threshold'],diagnostics=r['telemetry'].get('boundary_detection_diagnostics'),ablations={})
    panels=[]
    for name,mask in [('final',paint),('without_brightness_and_pole',raw&allowed),('raw_evidence',raw)]:
        t=ImageBoundaryTracker();t.nn_rank_mask=lane>0
        o=t._detect(mask,1,None)
        report['ablations'][name]=dict(pixels=int(mask[:,160:].sum()),quality=float(o[1]) if o else None,
            points=o[0][np.isfinite(o[0][:,0])].tolist() if o else [])
        im=image.copy();im[mask]=(0,0,255)
        if o:
            p=o[0][np.isfinite(o[0][:,0])];cv2.polylines(im,[np.rint(p).astype(int)],False,(255,255,0),1)
        im=cv2.resize(im,(640,360));cv2.putText(im,f'f{i} {name} threshold={u["threshold"]:.1f}',(5,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
        panels.append(im)
    if i==0:
        report['row_pixels']=[]
        for y in (100,120,140,160):
            xs=np.flatnonzero(raw[y]&(np.arange(320)>190))
            report['row_pixels'].append(dict(y=y,pixels=[dict(x=int(x),brightness=float(brightness[y,x]),nn=bool(lane[y,max(0,x-3):min(320,x+4)].any()),allowed=bool(allowed[y,x]),final=bool(paint[y,x])) for x in xs]))
    reports.append(report);tiles.append(np.hstack(panels))
(folder/'paint_audit.json').write_text(json.dumps(reports,indent=2))
cv2.imwrite(str(folder/'paint_audit.jpg'),np.vstack(tiles))
print(json.dumps([{**{k:v for k,v in r.items() if k!='ablations'},'ablations':{k:{a:b for a,b in v.items() if a!='points'} for k,v in r['ablations'].items()}} for r in reports]))
