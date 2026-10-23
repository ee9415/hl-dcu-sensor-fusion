"""Sequential identical-input replay, cold start, baseline versus current tracker."""
import json,sys,types,time
from collections import Counter
from pathlib import Path
import numpy as np
import cv2
root=Path(__file__).resolve().parents[3];sys.path.insert(0,str(root/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
folder=Path(sys.argv[1]);z=np.load(folder/'boundary_debug.npz')
records=json.loads(str(z['records_json']))
baseline=types.ModuleType('moving_baseline');sys.modules[baseline.__name__]=baseline
exec(compile((folder/'image_boundaries.py').read_text(),'baseline','exec'),baseline.__dict__)
def unpack(v):
    if isinstance(v,dict) and 'array' in v:return z[v['array']]
    return [unpack(a) for a in v] if isinstance(v,list) else v
reports={};pictures={}
for label,cls in [('baseline',baseline.ImageBoundaryTracker),('current',ImageBoundaryTracker)]:
    t=cls();frames=[];pictures[label]=[];previous_rgb=None
    check=t._plausible_pair;checks=[]
    def audited_pair(pair,learned=True):
        valid=check(pair,learned)
        if not valid and all(p is not None for p in pair):
            common=np.isfinite(pair[0][:,0])&np.isfinite(pair[1][:,0])
            gap=pair[1][:,0]-pair[0][:,0]
            minimum=np.where(t.rows>=150,t.config.image_min_width_ratio*np.clip(1.6*(t.rows-65),24,240),6.)
            if learned and t.width is not None:
                trusted=t.width_samples>=5
                minimum[trusted]=np.maximum(minimum[trusted],t.config.image_retained_width_ratio*t.width[trusted])
            bad=common&(gap<minimum)
            checks.append(dict(learned=learned,common_rows=int(common.sum()),
                rows=t.rows[bad].tolist(),gap=gap[bad].tolist(),minimum=minimum[bad].tolist(),
                smooth=[bool(t._smooth_shape(p)) for p in pair]))
        return valid
    t._plausible_pair=audited_pair
    for i,r in enumerate(records):
        u=next(c for c in r['calls'] if c['name']=='update')
        rgb=unpack(u['image']);boxes=[types.SimpleNamespace(xmin=a,ymin=b,xmax=c,ymax=d) for a,b,c,d in r['boxes']]
        checks.clear();start=time.perf_counter()
        e=t.update(unpack(u['road']),unpack(u['lane']),u['timestamp_s'],rgb=rgb,detections=boxes)
        elapsed=(time.perf_counter()-start)*1000
        row=dict(frame=i,timestamp=u['timestamp_s'],elapsed_ms=elapsed,
                 rgb_change=float(np.mean(abs(rgb.astype(float)-previous_rgb))) if previous_rgb is not None else None,
                 pair_failures=list(checks),**e.telemetry())
        previous_rgb=rgb.astype(float)
        frames.append(row)
        im=cv2.resize(rgb,(640,360))
        for p,color in [(e.image_left,(0,255,255)),(e.image_right,(255,180,0))]:
            p=p[np.isfinite(p).all(1)]
            if len(p)>1:cv2.polylines(im,[np.rint(p*2).astype(np.int32)],False,color,2)
        cv2.putText(im,f'{label} f{i} {e.left_state}/{e.right_state}',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
        pictures[label].append(im)
    reports[label]=dict(frames=len(frames),states=dict(Counter(f['boundary_state'] for f in frames)),
        restored=sum(f['boundary_restored_pair'] for f in frames),invalid=sum(not f['boundary_pair_valid'] for f in frames),
        left=dict(Counter(f['left_boundary_state'] for f in frames)),right=dict(Counter(f['right_boundary_state'] for f in frames)),
        rebase=sum(any(d.get('stale_prior_reacquired') for d in f['boundary_detection_diagnostics']) for f in frames))
    (folder/(label+'_replay.jsonl')).write_text('\n'.join(json.dumps(f) for f in frames))
indices=np.linspace(0,len(records)-1,6).astype(int)
cv2.imwrite(str(folder/'replay_comparison.jpg'),np.vstack([np.hstack([pictures[k][i] for k in ('baseline','current')]) for i in indices]))
(folder/'moving_replay_summary.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports))
