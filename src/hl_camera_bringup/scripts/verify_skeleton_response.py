"""Compare the current tracker with a committed baseline on identical steps."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker


def scene(shift=0., left=True):
    rows = np.arange(82.,176.)
    return types.SimpleNamespace(group_curves=[np.column_stack((160+sign*.9*(rows-65)+shift,
        rows,np.ones(len(rows)))) for sign in (-1,1) if sign==1 or left])


def measure(cls, fps, missing, preanchor=False):
    tracker = cls()
    for i in range(fps):
        before = tracker.update_contours(scene(),(180,320),i/fps)
    initial = before.image_left.copy()
    good = np.isfinite(initial[:,0])
    base = fps-1
    if missing and preanchor:
        tracker.update_contours(scene(left=False),(180,320),1.)
        base += 1
    reached, samples = None, []
    for i in range(1,4*fps+1):
        result = tracker.update_contours(scene(12.,left=not missing),(180,320),(base+i)/fps)
        error = float(np.max(abs(result.image_left[good,0]-initial[good,0]-12.)))
        samples.append(dict(time_s=i/fps,error_px=error,left_state=result.left_state,
                            right_state=result.right_state))
        if reached is None and error<=1.2:
            reached = i/fps
    return dict(t90_s=reached,samples=samples)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline-ref',required=True)
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args()
    old = subprocess.check_output(['git','show',args.baseline_ref+':apps/yolopv2_standalone/image_boundaries.py'],cwd=ROOT)
    module = types.ModuleType('_baseline_image_boundaries')
    sys.modules[module.__name__] = module
    exec(compile(old,'baseline_image_boundaries.py','exec'),module.__dict__)
    comparisons = []
    for fps in (10,20):
        for missing, preanchor in ((False,False),(True,True),(True,False)):
            baseline = measure(module.ImageBoundaryTracker,fps,missing,preanchor)
            current = measure(ImageBoundaryTracker,fps,missing,preanchor)
            assert current['t90_s'] and (baseline['t90_s'] is None or current['t90_s'] < baseline['t90_s']), (
                fps,missing,baseline['t90_s'],current['t90_s'])
            comparisons.append(dict(fps=fps,missing=missing,preanchor=preanchor,
                                    baseline=baseline,current=current))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.json').write_text(json.dumps(dict(baseline_ref=args.baseline_ref,
        step_px=12.,comparisons=comparisons),indent=2),encoding='utf-8')
    print(json.dumps([dict(fps=c['fps'],missing=c['missing'],preanchor=c['preanchor'],before_s=c['baseline']['t90_s'],
                          after_s=c['current']['t90_s']) for c in comparisons],indent=2))


if __name__=='__main__':
    main()
