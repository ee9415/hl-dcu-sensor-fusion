"""Cold-start A/B replay of synchronized tracker input at processing resolution."""
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np

root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from lane_boundaries import BoundaryConfig

capture=Path(sys.argv[1])
z=np.load(capture/'boundary_debug.npz')
records=json.loads(str(z['records_json']))
results={}
curves={}
for weight in (0,.06):
    tracker=ImageBoundaryTracker(BoundaryConfig(image_nn_rank_weight=weight))
    timings=[];estimates=[]
    for record in records:
        u=next(c for c in record['calls'] if c['name']=='update')
        def arr(key):return z[u[key]['array']]
        boxes=[SimpleNamespace(xmin=a,ymin=b,xmax=c,ymax=d) for a,b,c,d in record['boxes']]
        start=time.perf_counter()
        e=tracker.update(arr('road'),arr('lane'),u['timestamp_s'],rgb=arr('image'),detections=boxes)
        timings.append((time.perf_counter()-start)*1000)
        estimates.append(e.telemetry())
    curves[weight]=estimates
    results[str(weight)]={'frames':len(estimates),'valid_pairs':sum(e['boundary_pair_valid'] for e in estimates),
                          'mean_tracker_ms_host':float(np.mean(timings))}
differences=[]
for a,b in zip(curves[0],curves[.06]):
    for side in ('left','right'):
        x=np.array(a[side+'_boundary_image'],dtype=float)
        y=np.array(b[side+'_boundary_image'],dtype=float)
        if x.shape==y.shape and x.size:
            common=np.isfinite(x[:,0])&np.isfinite(y[:,0])
            if common.any():differences.extend(abs(x[common,0]-y[common,0]))
results['max_curve_difference_px']=float(max(differences,default=0))
(capture/'ranking_ab.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results))
