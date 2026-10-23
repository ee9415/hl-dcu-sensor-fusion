import sys,json,numpy as np
from pathlib import Path
p=Path('artifacts/left_debug_0322');sys.path.insert(0,str(p.resolve()));from image_boundaries import ImageBoundaryTracker
z=np.load(p/'boundary_debug.npz');r=json.loads(str(z['records_json']))[0]
for c in r['calls']:
 if c['name'] in ('_detect_full','_detect_legacy','_detect_regions') and c.get('side')==0:
  v=c['return'];print(c['name'],'None' if v is None else 'score='+str(v[1]));
  if v is not None:
   q=z[v[0]['array']];print(q[np.isfinite(q[:,0])][[0,-1]])
u=next(c for c in r['calls'] if c['name']=='update');paint=z[u['paint']['array']]
t=ImageBoundaryTracker()
for name,mask in [('original',paint),('exclude_right_for_diagnosis',paint.copy())]:
 if name!='original':mask[:,135:]=False
 o=t._detect(mask,0,None)
 print(name,'none' if o is None else o[1],t.detection_diagnostics[0])
 if o is not None:print(o[0][np.isfinite(o[0][:,0])][[0,-1]])
for y in range(164,145,-2):print('left paint row',y,np.flatnonzero(paint[y,:125]).tolist())
