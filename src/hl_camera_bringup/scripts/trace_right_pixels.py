"""Offline line-level trace of the exact captured right full detector."""
import json,sys,inspect
from pathlib import Path
import numpy as np
import cv2
folder=Path(sys.argv[1]);sys.path.insert(0,str(folder.resolve()))
from image_boundaries import ImageBoundaryTracker
z=np.load(folder/'boundary_debug.npz');records=json.loads(str(z['records_json']))
r=records[0];u=next(c for c in r['calls'] if c['name']=='update')
def a(k):return z[u[k]['array']]
paint=a('paint');rgb=a('image');lane=a('lane')
t=ImageBoundaryTracker();t.nn_rank_mask=lane>0
call=next(c for c in r['calls'] if c['name']=='_detect_full' and c['side']==1)
prior=z[call['prior']['array']] if call.get('prior') else None
source,start=inspect.getsourcelines(t._detect_full)
rankline=start+next(i for i,s in enumerate(source) if 'if best is None or rank>best[0]' in s)
optionline=start+next(i for i,s in enumerate(source) if 'if not len(options)' in s)
choices=[];candidates=[]
def trace(frame,event,arg):
    if frame.f_code is t._detect_full.__func__.__code__ and event=='line':
        v=frame.f_locals
        if frame.f_lineno==optionline:
            choices.append(dict(start=v['start'],seed=v['seed'],y=v['y'],predicted=v['predicted'],gate=v['gate'],options=v['options'].tolist(),last=v['last']))
        if frame.f_lineno==rankline:
            points=np.column_stack((v['xs'],v['ys']))
            candidates.append(dict(start=v['start'],seed=v['seed'],quality=v['score'],rank=v['rank'],temporal=v['temporal'],nn_support=t._nn_support(points[v['good']]),points=points.tolist(),good=v['good'].tolist(),coeff=v['coeff'].tolist()))
    return trace
sys.settrace(trace)
try: selected=t._detect_full(paint,1,prior)
finally:sys.settrace(None)
best=max(candidates,key=lambda c:c['rank'])
path=[c for c in choices if c['start']==best['start'] and c['seed']==best['seed']]
report={'threshold':u['threshold'],'candidates':candidates,'winning_steps':path}
table=[]
for y in (100,102,104,106,108,110,112,114,116,118,120,122,124,126,128):
    table.append(dict(y=y,paint_x=np.flatnonzero(paint[y,170:240]).__add__(170).tolist(),
                      selected_x=float(selected[0][np.argmin(abs(t.rows-y)),0])))
report['upper_rows']=table
report['visible_white_samples']=[dict(x=x,y=y,rgb=rgb[y,x].tolist(),brightness=float(rgb[y,x].mean()),nn=bool(lane[y,x]),paint=bool(paint[y,x])) for x,y in ((179,100),(182,102),(185,104),(188,106),(190,108))]
report['frame_comparison']=[]
for rec in records:
    found={}
    for call in rec['calls']:
        if call['name'] in ('_detect_regions','_detect_full','_detect') and call.get('side')==1 and call.get('return'):
            p=z[call['return'][0]['array']]
            found[call['name']]=dict(x100=float(p[np.argmin(abs(p[:,1]-100)),0]),quality=call['return'][1])
    report['frame_comparison'].append(found)
ablations=[]
for history in (True,False):
    for nn_weight in (.06,0.):
        from dataclasses import replace
        t.config=replace(t.config,image_nn_rank_weight=nn_weight)
        p=t._detect_full(paint,1,prior if history else None)
        at100=float(p[0][np.argmin(abs(t.rows-100)),0])
        ablations.append(dict(history=history,nn_weight=nn_weight,quality=p[1],x_at_y100=at100))
report['ablations']=ablations
im=cv2.resize(rgb,(960,540),interpolation=cv2.INTER_NEAREST)
for (x,y),good in zip(best['points'],best['good']):
    cv2.circle(im,(round(x*3),round(y*3)),3,(0,0,255) if good else (255,0,255),-1)
reg=next(c for c in r['calls'] if c['name']=='_detect_regions' and c['side']==1)['return']
regional=z[reg[0]['array']]
for curve,color in ((selected[0],(255,200,0)),(regional,(0,255,0))):
    p=curve[np.isfinite(curve).all(1)]
    cv2.polylines(im,[np.rint(p*3).astype(int)],False,color,2)
cv2.putText(im,'Cyan: selected full; green: regional; red: fit points',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),2)
cv2.imwrite(str(folder/'right_pixel_trace.jpg'),im)
(folder/'right_pixel_trace.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'visible_white_samples':report['visible_white_samples'],'frame_comparison':report['frame_comparison'],'winning_last_points':list(zip(best['points'][-4:],best['good'][-4:]))}))
