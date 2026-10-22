"""Synthetic geometric verification, visualization and host timing; no device I/O."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/yolopv2_standalone'))
from lane_boundaries import BoundaryTracker, draw_boundaries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/fixed_bev_0200')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    forward, lateral = np.linspace(0,25,81), np.linspace(-6,6,241)
    panels, reports, durations = [], {}, []
    for name, bend, half_width in (('Straight',0,2.), ('Left',-.004,1.5),
                                    ('Right',.004,1.5), ('Too narrow',0,.6),
                                    ('Road widened',0,5.), ('No road evidence',0,0)):
        shift = bend*forward**2
        road = abs(lateral[None,:]-shift[:,None]) <= half_width
        if half_width == 0:
            road[:] = False
        lane = np.zeros_like(road)
        tracker = BoundaryTracker()
        for n in range(81):
            start = time.perf_counter()
            result = tracker.update_bev(forward,lateral,road,lane,n*.1)
            durations.append((time.perf_counter()-start)*1000)
        # Display only measured road rows and the two polynomial boundaries.
        frame = np.full((480,640,3),(14,22,30),np.uint8)
        xx, yy = np.meshgrid(np.linspace(-6,6,640),np.linspace(40,0,480))
        visible = (yy <= 25) & (abs(xx-bend*yy**2) <= half_width)
        if half_width:
            frame[visible] = (70,85,70)
        draw_boundaries(frame,result)
        panel = Image.fromarray(frame[:,:,::-1])
        text = ImageDraw.Draw(panel)
        text.text((12,12),f'{name}: {result.state}',fill='white')
        text.text((12,30),f'normal width error: {result.width_error:.5f} dev',fill='white')
        for anchor in (result.left[0],result.right[0]):
            x=(anchor/12+.5)*639
            text.ellipse((x-4,473,x+4,479),outline='white',width=2)
        panels.append(panel)
        reports[name] = result.telemetry(tracker.config)
    sheet = Image.new('RGB',(1920,960))
    for i,panel in enumerate(panels):
        sheet.paste(panel,((i%3)*640,(i//3)*480))
    sheet.save(args.output/'verification.png')
    reports['host_timing_ms'] = {'median':float(np.median(durations)),
                               'p95':float(np.percentile(durations,95))}
    (args.output/'report.json').write_text(json.dumps(reports,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({name:{k:v for k,v in value.items() if k in
          ('boundary_state','corridor_feasible','normal_width_error_dev','road_coverage','median','p95')}
          for name,value in reports.items()},indent=2))


if __name__ == '__main__':
    main()
