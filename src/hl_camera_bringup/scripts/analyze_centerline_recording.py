"""Approximate overlay continuity from saved video; not a vehicle-motion measurement."""
import argparse
import json
import cv2
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('video')
parser.add_argument('--missing-image')
args = parser.parse_args()
cap = cv2.VideoCapture(args.video)
frames = missing = longest = run = 0
previous = None
steps = []
while True:
    ok, frame = cap.read()
    if not ok:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (85, 180, 180), (100, 255, 255))
    mask |= cv2.inRange(hsv, (14, 200, 200), (23, 255, 255))
    mask[:260] = 0
    mask[475:] = 0
    mask[:, :200] = 0
    mask[:, 800:] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 3), np.uint8))
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = [i for i in range(1, len(stats))
                  if stats[i, 3] >= 45 and stats[i, 2] < 200 and stats[i, 4] >= 100]
    frames += 1
    if not candidates:
        missing += 1
        run += 1
        longest = max(longest, run)
        if args.missing_image and run == 3:
            cv2.imwrite(args.missing_image, frame)
        previous = None
        continue
    run = 0
    chosen = max(candidates, key=lambda i: stats[i, 3])
    positions = []
    for row in (300, 350, 400, 450):
        y, x = np.where(labels[row-1:row+2] == chosen)
        positions.append(float(np.median(x)) if len(x) else None)
    if previous is not None:
        steps.extend(abs(a-b) for a, b in zip(previous, positions)
                     if a is not None and b is not None)
    previous = positions
cap.release()
print(json.dumps(dict(frames=frames, overlay_missing_frames=missing,
    longest_missing_frames=longest, step_px_p95=float(np.percentile(steps, 95)) if steps else None,
    max_step_px=max(steps) if steps else None), indent=2))
