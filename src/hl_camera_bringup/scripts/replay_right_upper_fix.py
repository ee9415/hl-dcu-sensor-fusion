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
    c=next(c for c in r['calls'] if c['name']=='_detect_full' and c.get('side')==1)
    prior=z[c['prior']['array']] if c.get('prior') else None
    t=ImageBoundaryTracker();t.nn_rank_mask=lane>0
    result=t._detect(paint,1,prior)
    old=z[c['return'][0]['array']]
    index=np.argmin(abs(t.rows-100))
    report.append(dict(frame=i,old_x100=float(old[index,0]),new_x100=float(result[0][index,0]) if result else None,quality=result[1] if result else None))
    if i==0 and '--fixture' in sys.argv:
        dest=root/'apps/yolopv2_standalone/tests/data';dest.mkdir(exist_ok=True)
        np.savez_compressed(dest/'right_upper_gap.npz',paint=paint,lane=lane,prior=prior)
(folder/'fix_replay.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
