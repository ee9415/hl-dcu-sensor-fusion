import sys,json,numpy as np
from pathlib import Path
p=Path('artifacts/left_debug_0322');sys.path.insert(0,str(p.resolve()));from image_boundaries import ImageBoundaryTracker
z=np.load(p/'boundary_debug.npz');r=json.loads(str(z['records_json']))[0];u=next(c for c in r['calls'] if c['name']=='update');mask=z[u['paint']['array']].copy();mask[:,135:]=False
def trace(f,e,a):
 if e=='return' and f.f_code.co_name=='_detect_regions':
  print('left-only band candidates',[[dict(points=len(q),xmin=q[:,0].min(),xmax=q[:,0].max(),ymin=q[:,1].min(),ymax=q[:,1].max()) for q in b] for b in f.f_locals['candidates']])
sys.setprofile(trace);ImageBoundaryTracker()._detect_regions(mask,0,None);sys.setprofile(None)
