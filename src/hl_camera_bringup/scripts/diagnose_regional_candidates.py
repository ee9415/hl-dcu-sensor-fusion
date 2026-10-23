"""Read-only replay of regional candidate rejection on saved scene evidence."""
import sys
import json
from pathlib import Path
from types import SimpleNamespace
from collections import Counter
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker

def diagnose(paint, label):
    tracker = ImageBoundaryTracker()
    events = Counter()
    details = {}
    def trace(frame, event, arg):
        if frame.f_code.co_name != '_detect_regions':
            return trace
        if event == 'line':
            events[frame.f_lineno] += 1
        elif event == 'return':
            details['segments'] = [[{'points': len(q), 'y': [float(q[:,1].min()), float(q[:,1].max())],
                                     'x': [float(q[:,0].min()), float(q[:,0].max())]}
                                    for q in band] for band in frame.f_locals['candidates']]
        return trace
    sys.settrace(trace)
    try:
        result = tracker._detect_regions(paint, 1, None)
    finally:
        sys.settrace(None)
    details.update(label=label, found=result is not None, executed_lines=dict(events))
    return details

fixture = np.load(ROOT / 'apps/yolopv2_standalone/tests/fixtures/parked_dashed_right_20261023.npz')
tracker = ImageBoundaryTracker()
reports = []
original = tracker._detect_regions
def capture(paint, side, prior):
    if side == 1:
        reports.append(diagnose(paint, 'fixture_current_mask'))
    return original(paint, side, prior)
tracker._detect_regions = capture
boxes = [SimpleNamespace(xmin=a,ymin=b,xmax=c,ymax=d) for a,b,c,d in fixture['boxes']]
tracker.update(fixture['road'].astype(np.uint8)*255, np.zeros((180,320),np.uint8),
               0, rgb=fixture['rgb'], detections=boxes)
z = np.load(ROOT / 'artifacts/right_paint_debug_20261023/variables.npz')
for i, record in enumerate(json.loads(str(z['records_json']))):
    u = next(c for c in record['calls'] if c['name'] == 'update')
    reports.append(diagnose(z[u['paint']['array']], 'historical_mask_' + str(i)))
print(json.dumps(reports, indent=2))
