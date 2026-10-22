"""Replay recorded real paint/prior stages; compare old output with corrected fusion."""
import argparse
import csv
import json
from pathlib import Path
import sys
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from lane_boundaries import draw_boundaries


def main():
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    z=np.load(args.source/'variables.npz');records=json.loads(str(z['records_json']))
    def get(v):
        if isinstance(v,dict) and 'array' in v:return z[v['array']].copy()
        if isinstance(v,list):return [get(i) for i in v]
        return v
    tracker=ImageBoundaryTracker();stats=[];tiles=[];last=None
    for i,r in enumerate(records):
        u=next(c for c in r['calls'] if c['name']=='update')
        raw=get(u['paint']);motion=get(u['motion'])
        predicted=[tracker._warp(p,motion) for p in tracker.history]
        observations=[tracker._detect(raw,s,predicted[s]) for s in (0,1)]
        b=tracker._fuse(observations,predicted,motion,u['motion_score'],.1)
        old=[np.array(r['telemetry'][s+'_boundary_image'],float).reshape(-1,2) for s in ('left','right')]
        row={'frame':i,'old_valid':r['telemetry']['boundary_pair_valid'],'new_valid':b.pair_valid,
             'left_state':b.left_state,'right_state':b.right_state,'restored':b.restored_pair}
        for side,label in enumerate(('left','right')):
            p=getattr(b,'image_'+label);good=np.isfinite(p[:,0]);support=getattr(b,label+'_support')
            row[label+'_min_y']=float(p[good,1].min()) if good.any() else None
            row[label+'_observed_rows']=int(support.sum())
            row[label+'_residual']=getattr(b,label+'_residual_px')
        stats.append(row);last=b
        if i in (6,7,8,15,len(records)-1):
            image=get(u['image']).astype(np.uint8);a=cv2.resize(image,(640,360));n=a.copy()
            for side,p in enumerate(old):
                if len(p):cv2.polylines(a,[np.rint(p*2).astype(int)],False,((0,220,255),(255,180,0))[side],2)
            draw_boundaries(n,b,(320,180))
            tile=np.hstack([a,n]);cv2.putText(tile,f'frame {i}: OLD | FIXED (same captured paint)',(8,18),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1);tiles.append(tile)
    before=[p.copy() for p in tracker.history]
    missing_valid=[]
    for _ in range(300):
        b=tracker._fuse([None,None],[p.copy() for p in tracker.history],np.eye(2,3),0.,.1)
        missing_valid.append(b.pair_valid)
    hold_error=max(float(np.nanmax(abs(a-b))) for a,b in zip(before,tracker.history))
    return_moves=[]
    observations=[tracker._detect(raw,s,tracker.history[s]) for s in (0,1)]
    for o in observations:
        if o is not None:o[0][:,0]+=12
    for _ in range(20):
        prior=[p.copy() for p in tracker.history]
        b=tracker._fuse(observations,prior,np.eye(2,3),0.,.1)
        return_moves.append(max(float(np.nanmax(abs(p[:,0]-q[:,0]))) for p,q in zip(prior,tracker.history)))
    first=next((i for i,r in enumerate(stats) if r['new_valid']),len(stats))
    report={'frames':len(stats),'first_valid_frame':first,'old_invalid':sum(not r['old_valid'] for r in stats),
            'new_invalid_after_first_detection':sum(not r['new_valid'] for r in stats[first:]),
            'max_observed_residual_px':max((r[s+'_residual'] or 0 for r in stats for s in ('left','right')),default=0),
            'synthetic_30s_loss_valid':all(missing_valid),'synthetic_30s_hold_error_px':hold_error,
            'synthetic_12px_reacquisition_max_step':max(return_moves),'final_return_states':[b.left_state,b.right_state]}
    with (args.output/'frames.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,stats[0].keys());w.writeheader();w.writerows(stats)
    cv2.imwrite(str(args.output/'comparison.jpg'),np.vstack(tiles))
    (args.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':main()
