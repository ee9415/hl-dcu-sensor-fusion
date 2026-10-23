import json,sys
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);z=np.load(root/'boundary_debug.npz');records=json.loads(str(z['records_json']))
def unpack(v):
    if isinstance(v,dict) and 'array' in v:return z[v['array']]
    if isinstance(v,list):return [unpack(x) for x in v]
    return v
def extent(p):
    if p is None:return None
    p=np.asarray(p);p=p[np.isfinite(p).all(axis=1)]
    return dict(n=len(p),ends=p[[0,-1]].tolist(),yrange=[float(p[:,1].min()),float(p[:,1].max())]) if len(p) else None
r=records[0]
out={'diagnostics':r['detection_diagnostics'],'segments':[[extent(unpack(q)) for q in s] for s in r['region_segments']], 'calls':[]}
for c in r['calls']:
    if c['name'] in ('_detect_regions','_detect_full','_detect_legacy','_detect'):
        ret=unpack(c.get('return'))
        out['calls'].append(dict(name=c['name'],side=c.get('side'),curve=extent(ret[0]) if ret else None,quality=ret[1] if ret else None))
    if c['name']=='_fuse':
        out['fuse']={'predicted':[extent(p) for p in unpack(c['predicted'])], 'observations':[dict(curve=extent(o[0]),quality=o[1]) if o else None for o in unpack(c['observations'])], 'output':[extent(p) for p in unpack(c['output'])]}
(root/'candidate_report.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
