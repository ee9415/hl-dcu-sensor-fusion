"""Reconstruct mask stages from an unmodified tracker variable capture."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from lane_boundaries import white_lane_evidence

root=Path(sys.argv[1]);z=np.load(root/'variables.npz')
records=json.loads(str(z['records_json']))
def get(v):
    if isinstance(v,dict) and 'array' in v:return z[v['array']]
    if isinstance(v,list):return [get(x) for x in v]
    return v
reports=[]
for i,r in enumerate(records):
    u=next(c for c in r['calls'] if c['name']=='update')
    image=get(u['image']);lane=get(u['lane']);road=get(u['road'])
    raw=white_lane_evidence(image,lane)>0
    bright=raw&(image.mean(2)>=u['threshold'])
    padded=np.pad(road,((2,2),(5,5)));dilated=np.zeros_like(road)
    for dy in (0,2,4):
        for dx in (0,2,5,8,10):dilated|=padded[dy:dy+180,dx:dx+320]
    stages=[raw,bright,bright&dilated,get(u['paint'])]
    t=ImageBoundaryTracker();stats=[]
    for mask in stages:
        o=t._detect(mask,1,None)
        stats.append(dict(t.detection_diagnostics[1],found=o is not None))
    calls=[c for c in r['calls'] if c['name']=='_detect' and c.get('side')==1]
    seeds=get(calls[0]['seeds']);traces=[]
    for start,seed in seeds[:28]:
        ys=[];xs=[];value=seed;slope=0.;last=start
        for y in range(start,76,-2):
            options=np.flatnonzero(stages[-1][y]);pred=value+slope*(y-last)
            options=options[abs(options-pred)<min(16.,5.+.25*(last-y))]
            if not len(options):continue
            value=float(options[np.argmin(abs(options-pred))]);ys.append(y);xs.append(value);last=y
            if len(ys)>=4:slope=float(np.clip((xs[-1]-xs[-4])/(ys[-1]-ys[-4]),-2,2))
        traces.append(dict(seed=[start,seed],points=list(zip(xs,ys)),count=len(ys),span=max(ys)-min(ys) if ys else 0))
    samples=[]
    for y in range(172,149,-2):
        lo=194+(y-164);hi=lo+8
        samples.append(dict(y=y,x_range=[lo,hi],
            stage_pixels=[(np.flatnonzero(m[y,lo:hi])+lo).tolist() for m in stages],
            brightness=image[y,lo:hi].mean(1).round().tolist()))
    reports.append(dict(frame=i,threshold=u['threshold'],stages=stats,traces=traces,samples=samples))
    if i==0:
        tiles=[]
        for name,mask in zip(('RGB evidence','Brightness','Road mask','Object exclusion'),stages):
            a=image.copy();a[mask]=(0,0,255)
            a=cv2.resize(a,(960,540),interpolation=cv2.INTER_NEAREST)
            cv2.putText(a,name,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2);tiles.append(a)
        cv2.imwrite(str(root/'stages.jpg'),np.vstack(tiles))
        cv2.imwrite(str(root/'rgb.jpg'),cv2.resize(image,(960,540)))
(root/'analysis.json').write_text(json.dumps(reports,indent=2))
print(json.dumps(reports[0]))
