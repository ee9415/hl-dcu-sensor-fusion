"""Replay captured fusion inputs, reporting every pair rejection."""
import sys,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[3];sys.path.insert(0,str(root/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
folder=Path(sys.argv[1]);z=np.load(folder/'boundary_debug.npz');r=json.loads(str(z['records_json']))[0]
def unpack(v):
    if isinstance(v,dict) and 'array' in v:return z[v['array']]
    return [unpack(x) for x in v] if isinstance(v,list) else v
f=next(c for c in r['calls'] if c['name']=='_fuse')
reports=[]
for learned in ([None,'disable_learned_guard'] if 'width_samples_before' in r else [False,True]):
    t=ImageBoundaryTracker();t.history=unpack(r['history_before']);t.scores=r['scores_before']
    t.width=unpack(r['width_before'])
    exact='width_samples_before' in r
    t.width_samples=unpack(r['width_samples_before']).copy() if exact else np.full(48,5 if learned else 0)
    if learned=='disable_learned_guard':t.width_samples[:]=0
    if exact:t.last_valid_pair=unpack(r['last_valid_pair_before'])
    original=t._plausible_pair;checks=[]
    def check(pair):
        valid=original(pair)
        if not valid and all(p is not None for p in pair):
            common=np.isfinite(pair[0][:,0])&np.isfinite(pair[1][:,0])
            gap=pair[1][common,0]-pair[0][common,0]
            minimum=np.where(t.rows>=150,.25*np.clip(1.6*(t.rows-65),24,240),6.)
            if t.width is not None:minimum=np.where(t.width_samples>=5,np.maximum(minimum,.55*t.width),minimum)
            failed=common.copy();failed[common]=gap<minimum[common]
            checks.append(dict(rows=t.rows[failed].tolist(),gap=(pair[1][failed,0]-pair[0][failed,0]).tolist(),minimum=minimum[failed].tolist(),width_samples=t.width_samples[failed].tolist()))
        return valid
    t._plausible_pair=check
    e=t._fuse(unpack(f['observations']),unpack(f['predicted']),unpack(f.get('motion',np.array([[1.,0.,0.],[0.,1.,0.]]))),f.get('motion_score',0.),f.get('step',.1))
    reports.append(dict(learned_width_assumption=learned,width_present=t.width is not None,states=[e.left_state,e.right_state],restored=e.restored_pair,rejections=checks))
(folder/'pair_rejection.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports))
