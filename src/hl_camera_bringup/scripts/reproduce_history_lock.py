"""Deterministic fusion audit; does not operate the camera."""
import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker

def make(t,lo,hi,x,slope):
    p=np.column_stack((x+slope*(t.rows-140),t.rows))
    p[(t.rows<lo)|(t.rows>hi),0]=np.nan
    return p

def run(warm):
    t=ImageBoundaryTracker()
    left=make(t,120,144,120,-.3);right=make(t,118,172,210,.4)
    if warm:
        t.history=[make(t,150,172,90,-.3),right.copy()];t.scores=[.5,.8]
    observations=[(left,.8,np.isfinite(left[:,0])),(right,.8,np.isfinite(right[:,0]))]
    result=[]
    for i in range(20):
        e=t._fuse(observations,t.history,np.array([[1.,0.,0.],[0.,1.,0.]]),0.,.1)
        p=e.image_left[np.isfinite(e.image_left[:,0])]
        result.append(dict(frame=i,left_state=e.left_state,confidence=e.left_confidence,
                           residual=e.left_residual_px,yrange=[p[:,1].min(),p[:,1].max()],
                           side_missing=float(t.side_missing[0]),pair_valid=e.pair_valid))
    return result

report={'fresh':run(False),'retained_disjoint_history':run(True),
        'scope':'Synthetic fusion reproducer, not proof that the attached frames used this branch.'}
dest=root/'artifacts/history_lock_reproduction';dest.mkdir(parents=True,exist_ok=True)
(dest/'report.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:[v[0],v[-1]] if isinstance(v,list) else v for k,v in report.items()}))
