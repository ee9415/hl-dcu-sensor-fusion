"""Offline saved-mask replay and exact polynomial/corridor diagnostics."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from virtual_guidance import GuidanceConfig, VirtualGuidance, draw_guidance


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    tracker=ImageBoundaryTracker();guide=VirtualGuidance(GuidanceConfig(enabled=True))
    rows=[];samples=[]
    for i,path in enumerate(sorted(a.input.glob('*.npz'))):
        with np.load(path) as f:
            if not all(k in f for k in ('road','lane','image_bgr')):continue
            road,lane,image=(f[k] for k in ('road','lane','image_bgr'))
        boundaries=tracker.update(road,lane,i*.1,rgb=np.repeat(np.repeat(image,6,axis=0),6,axis=1))
        began=time.perf_counter();r=guide.update(road,boundaries,i*.1)
        elapsed=(time.perf_counter()-began)*1000
        row=dict(frame=i,reason=r.reason,source=r.source,valid=r.valid,ms=elapsed,
                 boundaries_available=r.boundaries_available,boundary_state=r.boundary_state,
                 path_display_available=r.visible_path is not None)
        if r.valid:
            row.update(target=r.target.tolist(),
                polynomial_error=float(np.max(abs(np.polynomial.polynomial.polyval(r.path[:,1],r.coefficients)-r.path[:,0]))),
                midpoint_error=float(abs(r.target[0]-.5*(r.left[0,0]+r.right[0,0]))))
        rows.append(row)
        if i%5==0 and len(samples)<6:
            display=cv2.resize(image,(960,540));draw_guidance(display,r)
            cv2.putText(display,f'{i} {r.source} {r.reason}',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),1)
            samples.append(display)
    report=dict(frames=len(rows),valid=sum(r['valid'] for r in rows),
        boundaries_available=sum(r['boundaries_available'] for r in rows),
        path_display_available=sum(r['path_display_available'] for r in rows),
        boundary_states=dict(Counter(r['boundary_state'] for r in rows)),
        reasons=dict(Counter(r['reason'] for r in rows)),sources=dict(Counter(r['source'] for r in rows)),
        guidance_ms_mean=float(np.mean([r['ms'] for r in rows])) if rows else None,
        max_polynomial_error=max((r.get('polynomial_error',0) for r in rows),default=0),
        max_midpoint_error=max((r.get('midpoint_error',0) for r in rows),default=0))
    (a.output/'report.json').write_text(json.dumps(report,indent=2))
    (a.output/'frames.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    if samples:
        while len(samples)%2:samples.append(np.zeros_like(samples[0]))
        cv2.imwrite(str(a.output/'comparison.jpg'),np.vstack([np.hstack(samples[i:i+2]) for i in range(0,len(samples),2)]))
    print(json.dumps(report))


if __name__=='__main__':main()
