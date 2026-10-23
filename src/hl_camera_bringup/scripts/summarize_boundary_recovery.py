"""Summarize frame-level recovery without treating HELD as detection success."""
import json,sys
from collections import Counter
from pathlib import Path
import numpy as np
path=Path(sys.argv[1]);frames=[json.loads(s) for s in path.read_text().splitlines()]
def stamp(f):return f.get('nn_timestamp_s',f.get('timestamp',0.))
events=[];start=None;longest=0.;rejections=Counter();fixes=Counter()
for i,f in enumerate(frames):
    held=f['left_boundary_state']=='held' and f['right_boundary_state']=='held'
    if held and start is None:start=i
    if not held and start is not None:
        longest=max(longest,stamp(f)-stamp(frames[start]));start=None
    for d in f.get('boundary_detection_diagnostics',[]):
        if d.get('fusion_pair_rejection'):rejections[d['fusion_pair_rejection']]+=1
        for key in ('stale_prior_reacquired','confirmed_pair_replaced_invalid_fusion'):
            if d.get(key):fixes[key]+=1
    if i and f['left_boundary_state']==f['right_boundary_state']=='observed':
        old=frames[i-1]
        if old['left_boundary_state']!='observed' or old['right_boundary_state']!='observed':
            matrix=np.array(f['motion_matrix'])
            points=np.array([[80,100,1],[240,100,1],[80,170,1],[240,170,1]])
            shift=float(np.max(np.linalg.norm(points@matrix.T-points[:,:2],axis=1)))
            events.append(dict(frame=f.get('frame_seq',f.get('frame')),timestamp=stamp(f),
                previous=[old['left_boundary_state'],old['right_boundary_state']],motion_confidence=f['motion_confidence'],
                applied_motion_px=shift,rgb_change=f.get('rgb_change')))
if start is not None:longest=max(longest,stamp(frames[-1])-stamp(frames[start]))
summary=dict(frames=len(frames),duration_s=stamp(frames[-1])-stamp(frames[0]),
    left=dict(Counter(f['left_boundary_state'] for f in frames)),right=dict(Counter(f['right_boundary_state'] for f in frames)),
    restored=sum(f['boundary_restored_pair'] for f in frames),invalid=sum(not f['boundary_pair_valid'] for f in frames),
    longest_both_held_s=longest,rejection_side_counts=dict(rejections),recovery_side_counts=dict(fixes),
    recovery_events=events)
path.with_name(path.stem+'_recovery_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary))
