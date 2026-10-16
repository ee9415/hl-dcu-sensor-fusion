"""Read-only 30 second RTSP observation; pixel metrics are not vehicle coordinates."""
import os
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np

os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
out = args.output
out.mkdir(parents=True, exist_ok=True)
cap = cv2.VideoCapture('rtsp://192.168.0.4:8554/yolopv2', cv2.CAP_FFMPEG)
if not cap.isOpened():
    raise SystemExit('RTSP open failed')
for _ in range(20):
    cap.read()
started = time.monotonic()
samples, pictures = [], []
writer = None
while time.monotonic() - started < 30:
    ok, frame = cap.read()
    if not ok:
        break
    if writer is None:
        writer = cv2.VideoWriter(str(out / 'result.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 10, (frame.shape[1], frame.shape[0]))
    writer.write(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cyan = cv2.inRange(hsv, (80, 140, 160), (100, 255, 255))
    orange = cv2.inRange(hsv, (12, 160, 160), (25, 255, 255))
    positions = []
    for row in (300, 350, 400, 450, 490):
        xs = np.flatnonzero((cyan[row-1:row+2] | orange[row-1:row+2]).any(axis=0))
        xs = xs[(xs > 100) & (xs < 860)]
        positions.append(float(np.median(xs)) if len(xs) >= 2 else None)
    elapsed = time.monotonic() - started
    samples.append({'t': elapsed, 'x': positions, 'cyan_pixels': int(np.count_nonzero(cyan)), 'orange_pixels': int(np.count_nonzero(orange))})
    if len(pictures) < 6 and elapsed >= len(pictures)*5:
        thumb = cv2.resize(frame, (640, 360))
        cv2.putText(thumb, f'{elapsed:.1f}s', (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        pictures.append(thumb)
cap.release()
if writer:
    writer.release()
(out / 'samples.json').write_text(json.dumps(samples), encoding='utf-8')
if len(pictures) == 6:
    cv2.imwrite(str(out / 'contact.jpg'), np.vstack([np.hstack(pictures[i:i+2]) for i in (0,2,4)]))
summary = {'frames': len(samples), 'duration_s': samples[-1]['t'] if samples else 0, 'rows': {}}
for i, row in enumerate((300,350,400,450,490)):
    diffs = [abs(b['x'][i]-a['x'][i]) for a,b in zip(samples,samples[1:]) if a['x'][i] is not None and b['x'][i] is not None]
    summary['rows'][row] = {'detected_frames':sum(s['x'][i] is not None for s in samples), 'step_px_p95':float(np.percentile(diffs,95)) if diffs else None, 'max_step_px':max(diffs) if diffs else None}
(out / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
print(json.dumps(summary, indent=2))
