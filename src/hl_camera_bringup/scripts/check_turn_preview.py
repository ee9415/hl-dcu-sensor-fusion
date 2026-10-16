"""Offline synthetic turn QA; never labels these images as camera observations."""
import sys
from pathlib import Path
import json
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/yolopv2_standalone'))
from centerline import estimate_centerline
from dataclasses import replace
from turn_preview import TurnPreview, draw_turn

out = ROOT/'artifacts/turn_preview_20261016'
out.mkdir(parents=True, exist_ok=True)
mask = np.ones((180, 320), dtype=np.uint8)
base = replace(estimate_centerline(mask), coefficients=(0., .02, 0., 0.))
images = []
results = {}
for mode in ('left', 'right'):
    planner = TurnPreview()
    planner.request(mode, base)
    for i in range(1401):
        points = planner.update(base, i*.1)
        if i in (0, 300, 1400):
            frame = np.full((480, 640, 3), (14, 22, 30), np.uint8)
            draw_turn(frame, points)
            cv2.putText(frame, f'SYNTHETIC {mode} {i/10:.0f}s {planner.status}',
                        (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
            images.append(frame)
    tangent = points[-1]-points[-2]
    results[mode] = {'status': planner.status, 'end_heading_deg': float(np.degrees(np.arctan2(tangent[1], tangent[0])))}
cv2.imwrite(str(out/'synthetic_contact.jpg'), np.vstack((np.hstack(images[:3]), np.hstack(images[3:]))))
(out/'synthetic.json').write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
