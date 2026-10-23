"""Compare line and quadratic residuals on captured left detector points."""
import inspect,json,sys
from pathlib import Path
import numpy as np
import cv2
folder=Path(sys.argv[1]);sys.path.insert(0,str(folder.resolve()))
from image_boundaries import ImageBoundaryTracker
z=np.load(folder/'boundary_debug.npz');records=json.loads(str(z['records_json']))
reports=[]
def unpack(v):
    if isinstance(v,dict) and 'array' in v:return z[v['array']]
    return [unpack(x) for x in v] if isinstance(v,list) else v
for i,r in enumerate(records):
    u=next(c for c in r['calls'] if c['name']=='update')
    paint=unpack(u['paint']);rgb=unpack(u['image'])
    c=next(c for c in r['calls'] if c['name']=='_detect_full' and c.get('side')==0)
    prior=unpack(c.get('prior'));t=ImageBoundaryTracker();t.nn_rank_mask=unpack(u['lane'])>0
    source,start=inspect.getsourcelines(t._detect_full)
    line=start+next(j for j,s in enumerate(source) if 'if best is None or rank>best[0]' in s)
    candidates=[]
    def trace(frame,event,arg):
        if event=='line' and frame.f_code is t._detect_full.__func__.__code__ and frame.f_lineno==line:
            v=frame.f_locals
            candidates.append(dict(rank=v['rank'],quality=v['score'],points=np.column_stack((v['xs'],v['ys'])).copy(),good=v['good'].copy(),start=v['start'],seed=v['seed']))
        return trace
    sys.settrace(trace)
    try:out=t._detect_full(paint,0,prior)
    finally:sys.settrace(None)
    best=max(candidates,key=lambda x:x['rank']);q=best['points'][best['good']]
    def fit(q,degree):
        coef=np.polynomial.polynomial.polyfit(q[:,1],q[:,0],degree)
        err=np.polynomial.polynomial.polyval(q[:,1],coef)-q[:,0]
        return dict(coeff=coef.tolist(),mae=float(abs(err).mean()),max=float(abs(err).max()))
    linear=fit(q,1);quadratic=fit(q,2)
    groups=[];centers=[]
    for x,y in q:
        row=np.flatnonzero(paint[int(y)])
        parts=np.split(row,np.flatnonzero(np.diff(row)>1)+1)
        group=next(g for g in parts if int(x) in g)
        center=float(group.mean());centers.append((center,y))
        groups.append(dict(y=int(y),chosen=float(x),minimum=int(group.min()),maximum=int(group.max()),center=center,offset=float(x-center)))
    f=next(c for c in r['calls'] if c['name']=='_fuse')
    final=unpack(f['output'])[0];final=final[np.isfinite(final).all(1)]
    delta=np.polynomial.polynomial.polyval(t.rows,quadratic['coeff'])-np.polynomial.polynomial.polyval(t.rows,linear['coeff'])
    reports.append(dict(frame=i,quality=best['quality'],seed=[best['seed'],best['start']],points=q.tolist(),pixel_groups=groups,group_center_linear=fit(np.array(centers),1),group_center_quadratic=fit(np.array(centers),2),linear=linear,quadratic=quadratic,final_fit=fit(final,2),near_linear=fit(q[q[:,1]>=122],1),near_quadratic=fit(q[q[:,1]>=122],2),max_line_curve_difference=float(abs(delta[(t.rows>=q[:,1].min())&(t.rows<=q[:,1].max())]).max())))
    if i==0:
        im=cv2.resize(rgb,(960,540),interpolation=cv2.INTER_NEAREST)
        for x,y in q:cv2.circle(im,(round(x*3),round(y*3)),3,(0,0,255),-1)
        for coef,color in ((linear['coeff'],(0,255,0)),(quadratic['coeff'],(0,220,255))):
            yy=t.rows[(t.rows>=q[:,1].min())&(t.rows<=q[:,1].max())]
            pts=np.column_stack((np.polynomial.polynomial.polyval(yy,coef),yy))
            cv2.polylines(im,[np.rint(pts*3).astype(int)],False,color,2)
        cv2.putText(im,'Green: line fit; yellow: quadratic; red: selected pixels',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),2)
        cv2.imwrite(str(folder/'left_line_vs_curve.jpg'),im)
(folder/'left_straight_report.json').write_text(json.dumps(reports,indent=2))
print(json.dumps({k:v for k,v in reports[0].items() if k in ('pixel_groups','group_center_linear','group_center_quadratic')}))
