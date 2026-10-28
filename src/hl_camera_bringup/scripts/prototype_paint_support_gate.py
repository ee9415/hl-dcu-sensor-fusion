"""Offline prototype: require local paint support before pair reacquisition.

This is an experiment, not device code. Paint alone cannot distinguish a
parking marking from a lane boundary. It does not gate normal gradual fusion.
"""
import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker


def support_fraction(paint, observation):
    if paint is None or observation is None or np.shape(paint) != (180, 320):
        return 0., 0
    points = np.asarray(observation[0])
    support = np.asarray(observation[2], dtype=bool) if len(observation) > 2 else np.isfinite(points[:, 0])
    good = support & np.isfinite(points).all(axis=1)
    points = np.rint(points[good]).astype(int)
    points = points[(points[:, 0] >= 2) & (points[:, 0] < 318) &
                    (points[:, 1] >= 1) & (points[:, 1] < 179)]
    if len(points) < 8 or np.ptp(points[:, 1]) < 20:
        return 0., len(points)
    hit = np.zeros(len(points), dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-2, -1, 0, 1, 2):
            hit |= paint[points[:, 1]+dy, points[:, 0]+dx] > 0
    return float(hit.mean()), len(points)


def install_gate(tracker, current_paint, stats):
    original = tracker._confirm_current_pair
    def gated(observations, predicted):
        evidence = [support_fraction(current_paint[0], o) for o in observations]
        stats.append(evidence)
        if any(count < 8 or fraction < .4 for fraction, count in evidence):
            tracker.current_pair_hits = 0
            tracker.current_pair_previous = None
            return predicted
        return original(observations, predicted)
    tracker._confirm_current_pair = gated


def replay_capture(path, gated):
    archive = np.load(path)
    records = json.loads(str(archive['records_json']))
    tracker = ImageBoundaryTracker()
    paint, evidence = [None], []
    if gated:
        original_detect = tracker._detect
        def detect(mask, side, prior):
            paint[0] = mask
            return original_detect(mask, side, prior)
        tracker._detect = detect
        install_gate(tracker, paint, evidence)
    output = []
    for record in records:
        call = next(c for c in record['calls'] if c['name'] == 'update')
        def get(name):
            return archive[call[name]['array']]
        boxes = [SimpleNamespace(xmin=b[0], ymin=b[1], xmax=b[2], ymax=b[3])
                 for b in record['boxes']]
        result = tracker.update(get('road').astype(np.uint8)*255, get('lane'),
                                call['timestamp_s'], rgb=get('image'), detections=boxes)
        output.append({'time': record['source']['source_media_time_s'],
                       'frame': record['source']['source_frame_index'],
                       'state': result.right_state,
                       'right': result.telemetry()['right_boundary_image']})
    return output, evidence


def synthetic(gated, false_has_paint):
    tracker = ImageBoundaryTracker()
    def curve(x):
        return np.column_stack((np.full(48, float(x)), tracker.rows))
    tracker.history = [curve(80), curve(200)]
    tracker.scores = [.8, .05]
    tracker.last_valid_pair = [curve(80), curve(200)]
    mask = np.zeros((180, 320), dtype=np.uint8)
    for y in np.rint(tracker.rows).astype(int):
        mask[y, 80] = 1
        if false_has_paint:
            mask[y, 245] = 1
    evidence = []
    if gated:
        install_gate(tracker, [mask], evidence)
    values = []
    for _ in range(3):
        observations = [(curve(80), .8, np.ones(48, bool)),
                        (curve(245), .8, np.ones(48, bool))]
        result = tracker._fuse(observations, [p.copy() for p in tracker.history], np.eye(2, 3), 0., .1)
        values.append(float(result.image_right[20, 0]))
    return values, evidence


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    baseline, _ = replay_capture(args.capture, False)
    candidate, evidence = replay_capture(args.capture, True)
    result = {'capture_baseline': baseline, 'capture_gated': candidate,
              'capture_support': evidence,
              'synthetic_unpainted_baseline': synthetic(False, False)[0],
              'synthetic_unpainted_gated': synthetic(True, False)[0],
              'synthetic_painted_baseline': synthetic(False, True)[0],
              'synthetic_painted_gated': synthetic(True, True)[0]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps({k: v for k, v in result.items() if k.startswith('synthetic_')}, indent=2))


if __name__ == '__main__':
    main()
