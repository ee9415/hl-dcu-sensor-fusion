"""Offline old/new comparison; synthetic truth plus optional RGB-only diagnostic replay."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from trapezoid_boundaries import TrapezoidBoundaryTracker
from lane_boundaries import draw_boundaries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--capture-directory', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    trackers = [ImageBoundaryTracker(), TrapezoidBoundaryTracker()]
    times, errors = [[], []], [[], []]
    snapshots = []
    for i in range(120):
        curve = min(24., max(0., i-25)*.8)
        missing = 60 <= i < 90
        rgb = np.full((180, 320, 3), 60, np.uint8)
        road = np.zeros((180, 320), np.uint8)
        for y in range(90, 179):
            bend = curve*((174-y)/82)**2
            a, b = 160-.9*(y-65)+bend, 160+.9*(y-65)+bend
            road[y, max(0, int(a)):min(320, int(b)+1)] = 1
            if not missing:
                for x in (a, b):
                    x = int(round(x))
                    rgb[y, max(0, x-1):min(320, x+2)] = 235
        tiles = []
        for k, tracker in enumerate(trackers):
            started = time.perf_counter()
            estimate = tracker.update(road, road*0, i*.1, rgb=rgb)
            times[k].append((time.perf_counter()-started)*1000)
            frame_error = []
            for side, sign in (('left', -1), ('right', 1)):
                p = getattr(estimate, 'image_'+side)
                good = np.isfinite(p[:, 0]) & (p[:, 1] >= 92)
                truth = 160+sign*.9*(p[good, 1]-65)+curve*((174-p[good, 1])/82)**2
                frame_error.extend(abs(p[good, 0]-truth))
            errors[k].append(float(np.mean(frame_error)) if frame_error else None)
            if i in (20, 55, 85, 94, 119):
                frame = cv2.resize(rgb, (640, 360))
                draw_boundaries(frame, estimate, (320, 180))
                cv2.putText(frame, f'{"OLD" if k == 0 else "SEGMENTS"} frame={i} missing={missing}',
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                tiles.append(frame)
        if tiles:
            snapshots.append(np.hstack(tiles))
    cv2.imwrite(str(args.output/'synthetic_comparison.jpg'), np.vstack(snapshots))
    report = {'synthetic': {}, 'device_fps_verified': False}
    for k, name in enumerate(('old_image_motion', 'trapezoid_segments')):
        valid = [e for e in errors[k] if e is not None]
        report['synthetic'][name] = dict(mean_error_px=float(np.mean(valid)),
            max_frame_mean_error_px=max(valid), missing_output_frames=errors[k].count(None),
            update_mean_ms=float(np.mean(times[k][10:])), update_p95_ms=float(np.percentile(times[k][10:], 95)))
    if args.capture_directory:
        # This collection has NO synchronized NN masks/boxes. A lower-image ROI
        # is only a diagnostic substitute; do not label this a camera accuracy test.
        manifest = json.loads((args.capture_directory/'manifest.json').read_text())
        trackers = [ImageBoundaryTracker(), TrapezoidBoundaryTracker()]
        samples = manifest['frames']['rgb']
        diagnostic = []
        for i, item in enumerate(samples):
            rgb = cv2.imread(str(args.capture_directory/item['path']))
            if rgb is None:
                continue
            road = np.zeros((180, 320), np.uint8)
            road[92:] = 1
            tiles = []
            for k, tracker in enumerate(trackers):
                estimate = tracker.update(road, road*0, item['receive_s'], rgb=rgb)
                if i % 10 == 0 or i == len(samples)-1:
                    frame = cv2.resize(rgb, (640, 360))
                    draw_boundaries(frame, estimate, (320, 180))
                    cv2.putText(frame, f'{"OLD" if k == 0 else "SEGMENTS"} RGB-only ROI frame={i}',
                                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                    tiles.append(frame)
            if tiles:
                diagnostic.append(np.hstack(tiles))
        if diagnostic:
            cv2.imwrite(str(args.output/'rgb_only_diagnostic.jpg'), np.vstack(diagnostic))
        report['rgb_only_diagnostic'] = dict(frames=len(samples),
            limitation='No synchronized NN masks/boxes or ground truth; lower-image ROI only; corrupt input frames retained.')
    (args.output/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
