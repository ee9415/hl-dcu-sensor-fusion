"""Read-only telemetry analysis and isolated fusion reproductions (no device changes)."""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker


def main():
    root=Path(sys.argv[1])
    frames=[json.loads(line) for line in (root/'frames.jsonl').read_text().splitlines()]
    first,last=frames[0],frames[-1]
    tracker=ImageBoundaryTracker()
    def unpack(points):
        lookup={y:x for x,y in points}
        return np.column_stack(([lookup.get(y,np.nan) for y in tracker.rows],tracker.rows))
    pair=[unpack(first[s+'_boundary_image']) for s in ('left','right')]
    tracker.history=[p.copy() for p in pair]
    tracker.last_valid_pair=[p.copy() for p in pair]
    tracker.scores=[first[s+'_boundary_confidence'] for s in ('left','right')]
    tracker.width=pair[1][:,0]-pair[0][:,0]
    measured=pair[0].copy();measured[:,0]+=10
    result=tracker._fuse([(measured,.9),None],pair,np.eye(2,3),0.,.1)
    # A noncrossing but hooked line passes geometry validation.
    y=tracker.rows
    hooked=[np.column_stack((100+.25*(172-y),y)),
            np.column_stack((220-.4*(172-y)+25*np.sin((172-y)/8),y))]
    # Updating only near rows joins a new lower section to unchanged old rows.
    t=ImageBoundaryTracker();t.history=[p.copy() for p in hooked]
    t.last_valid_pair=[p.copy() for p in hooked];t.scores=[.7,.7]
    partial=hooked[0].copy();partial[:,0]+=40;partial[y<140,0]=np.nan
    for _ in range(20):
        estimate=t._fuse([(partial,.9),None],[p.copy() for p in t.history],np.eye(2,3),0.,.1)
    join=np.flatnonzero(y==140)[0]
    report={
        'frames':len(frames),'first_seq':first['frame_seq'],'last_seq':last['frame_seq'],
        'right_held_frames':sum(f['right_boundary_state']=='held' for f in frames),
        'restored_frames':sum(f['boundary_restored_pair'] for f in frames),
        'right_confidence_first':first['right_boundary_confidence'],
        'right_confidence_last':last['right_boundary_confidence'],
        'right_identical_all_frames':all(f['right_boundary_image']==first['right_boundary_image'] for f in frames),
        'motion_zero_frames':sum(f['motion_confidence']==0 for f in frames),
        'live_width_y172_px':pair[1][0,0]-pair[0][0,0],
        'isolated_left_10px_update_right_change_px':float(np.nanmax(abs(result.image_right-pair[1]))),
        'isolated_left_10px_update_left_change_px':float(np.nanmax(abs(result.image_left-pair[0]))),
        'synthetic_hooked_pair_accepted':t._valid_pair(hooked),
        'synthetic_partial_join_dx_px':float(abs(estimate.image_left[join,0]-estimate.image_left[join+1,0])),
        'synthetic_partial_join_dy_px':2.,
        'synthetic_partial_pair_valid':estimate.pair_valid,
    }
    (root/'analysis.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
