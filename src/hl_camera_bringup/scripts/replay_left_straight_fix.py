"""Replay captured paint with current detector; optionally retain a regression fixture."""
import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
folder=Path(sys.argv[1]);z=np.load(folder/'boundary_debug.npz')
records=json.loads(str(z['records_json']));report=[]
for i,r in enumerate(records):
    u=next(c for c in r['calls'] if c['name']=='update')
    paint=z[u['paint']['array']];lane=z[u['lane']['array']]
    c=next(c for c in r['calls'] if c['name']=='_detect_full' and c.get('side')==0)
    prior=z[c['prior']['array']] if c.get('prior') else None
    t=ImageBoundaryTracker();t.nn_rank_mask=lane>0
    result=t._detect(paint,0,prior)
    points=result[0];good=np.isfinite(points[:,0])
    coeff=np.polynomial.polynomial.polyfit(points[good,1],points[good,0],2)
    report.append(dict(frame=i,coeff=coeff.tolist(),quality=result[1]))
    if i==0 and '--fixture' in sys.argv:
        np.savez_compressed(root/'apps/yolopv2_standalone/tests/data/left_straight.npz',paint=paint,lane=lane,prior=prior)
(folder/'straight_fix_replay.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
