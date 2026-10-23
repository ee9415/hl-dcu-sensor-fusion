import sys,json,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/"apps/yolopv2_standalone"))
from image_boundaries import ImageBoundaryTracker
z=np.load("artifacts/right_paint_debug_20261023/variables.npz");rs=json.loads(str(z["records_json"]))
t=ImageBoundaryTracker()
for i in range(100):
 r=rs[i%len(rs)];u=next(c for c in r["calls"] if c["name"]=="update");paint=z[u["paint"]["array"]]
 obs=[t._detect(paint,s,t.history[s]) for s in (0,1)]
 b=t._fuse(obs,[None if p is None else p.copy() for p in t.history],np.eye(2,3),0,.1)
 if i in (0,9,29,99):print(i,b.left_state,b.right_state,b.right_residual_px,b.restored_pair,b.image_right[:8,0].tolist())
