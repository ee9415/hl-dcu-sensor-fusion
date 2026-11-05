"""Reproduce a controlled dropout using a stored RGB/lane-mask .npz.

This injects missing left paint and 0..8 px motion into one real mask. It is
not a replay of genuine vehicle motion and does not measure ground-truth accuracy.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/yolopv2_standalone'))
from ego_lane_quadratic import (select_ego_quadratics, add_virtual_side,
                               complete_partial_lanes)
from image_boundaries import ImageBoundaryTracker
from lane_contours import extract_lane_contours
from lane_overlay import overlay_lane_quadratics, overlay_tracked_skeleton


def run(archive, output):
    output.mkdir(parents=True, exist_ok=True)
    with np.load(archive, allow_pickle=False) as data:
        rgb, source = data['rgb'].copy(), data['lane'].copy()
    lane = (source >= .5 if source.dtype.kind == 'f' else source > 0).astype(np.uint8)*255
    tracker = ImageBoundaryTracker()
    previous = [None, None]
    records, panels = [], []
    for frame in range(30):
        mask = lane.copy()
        if frame >= 10:
            mask[:, :mask.shape[1]//2] = 0
            shift = min(frame-10, 8)
            if shift:
                moved = np.zeros_like(mask)
                moved[:, shift:] = mask[:, :-shift]
                mask = moved
        contours = extract_lane_contours(mask)
        before = complete_partial_lanes(add_virtual_side(
            select_ego_quadratics(contours, mask.shape), contours, mask.shape), contours, mask.shape)
        started = time.perf_counter()
        estimate = tracker.update_contours(contours, mask.shape, frame*.05, rgb)
        elapsed = time.perf_counter()-started
        record = dict(frame=frame, time_s=frame*.05, states=[estimate.left_state, estimate.right_state],
                      old_reason=before.reason, old_jump_px=None, new_jump_px=None, tracking_ms=elapsed*1000)
        fit, prior = before.left, previous[0]
        if fit is not None and prior is not None:
            lo,hi = max(fit.points[0,1],prior.points[0,1]),min(fit.points[-1,1],prior.points[-1,1])
            if hi > lo:
                rows = np.linspace(lo,hi,32)
                record['old_jump_px'] = float(np.max(abs(
                    np.polyval(fit.coefficients,rows)-np.polyval(prior.coefficients,rows))))
        previous[0] = fit
        points = estimate.image_left
        if previous[1] is not None:
            common = np.isfinite(points[:,0]) & np.isfinite(previous[1][:,0])
            if common.any():
                record['new_jump_px'] = float(np.max(abs(points[common,0]-previous[1][common,0])))
        previous[1] = points.copy()
        records.append(record)
        if frame in (9, 10, 14, 20, 29):
            left = cv2.resize(rgb, (640,360))
            right = left.copy()
            overlay_lane_quadratics(left,before,mask.shape)
            overlay_tracked_skeleton(right,estimate,mask.shape)
            cv2.putText(left,f'BEFORE t={frame*.05:.2f}s',(8,24),0,.6,(0,255,255),2)
            cv2.putText(right,'AFTER '+str(record['states']),(8,24),0,.5,(0,255,255),1)
            panels.append(np.hstack((left,right)))
    if records[9]['states'] != ['observed', 'observed'] or records[10]['new_jump_px'] is None:
        raise ValueError('Fixture must establish an observed bilateral pair before dropout')
    if any(record['states'][0] not in ('held','inferred') for record in records[10:]):
        raise AssertionError('An unrelated contour impersonated the missing left boundary')
    if any((record['new_jump_px'] or 0.) > 20.*.05+1e-6 for record in records[10:]):
        raise AssertionError('Missing-side movement exceeded the 20 px/s contract')
    report = dict(source=str(archive), kind=__doc__, frames=len(records), records=records,
                  old_max_jump_px=max(r['old_jump_px'] or 0. for r in records),
                  new_max_jump_px=max(r['new_jump_px'] or 0. for r in records),
                  transition=records[10], tracking_max_ms=max(r['tracking_ms'] for r in records))
    (output/'real_mask_dropout.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    if not cv2.imwrite(str(output/'comparison.jpg'),np.vstack(panels)):
        raise OSError('Could not save comparison image')
    print(json.dumps({k:v for k,v in report.items() if k != 'records'},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    run(args.archive,args.output)
