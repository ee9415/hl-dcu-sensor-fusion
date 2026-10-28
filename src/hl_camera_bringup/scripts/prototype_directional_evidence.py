"""Offline directional-paint diagnostics; not used by the device tracker."""
import argparse
import json
from pathlib import Path

import numpy as np


def inspect_candidate(paint, points, support):
    """Measure local continuity along a candidate and sideways paint nearby.

    Geometry uses image coordinates only. Metrics do not identify lane semantics.
    """
    mask = np.asarray(paint) > 0
    points = np.asarray(points, dtype=float)
    support = np.asarray(support, dtype=bool)
    if mask.shape != (180, 320) or points.ndim != 2 or points.shape[1] != 2 or len(support) != len(points):
        raise ValueError('Invalid paint or candidate shape')
    finite = np.isfinite(points).all(axis=1)
    curve = points[finite]
    selected = points[finite & support]
    if len(curve) < 8 or len(selected) < 8:
        return {'rows': len(selected), 'longitudinal_fraction': None,
                'transverse_fraction': None}
    curve = curve[np.argsort(curve[:, 1])]
    along = []
    transverse = []
    for x, y in selected:
        yi = int(round(y))
        if not (5 <= yi < 175 and 10 <= x < 310):
            continue
        found = []
        for dy in (-4, 4):
            yy = yi + dy
            if yy < curve[0, 1] or yy > curve[-1, 1]:
                found.append(False)
                continue
            xx = int(round(np.interp(yy, curve[:, 1], curve[:, 0])))
            if not (2 <= xx < 318):
                found.append(False)
                continue
            found.append(bool(mask[yy-1:yy+2, xx-2:xx+3].any()))
        along.append(all(found))
        xx = int(round(x))
        transverse.append(bool(mask[yi-1:yi+2, xx-8:xx-3].any() and
                               mask[yi-1:yi+2, xx+4:xx+9].any()))
    return {'rows': len(along),
            'longitudinal_fraction': float(np.mean(along)) if along else None,
            'transverse_fraction': float(np.mean(transverse)) if transverse else None}


def synthetic():
    rows = np.linspace(80, 170, 46)
    candidate = np.column_stack((180 + .4*(rows-80), rows))
    support = np.ones(len(rows), dtype=bool)
    longitudinal = np.zeros((180, 320), np.uint8)
    for x, y in candidate:
        longitudinal[int(round(y)), int(round(x))] = 1
    horizontal = np.zeros_like(longitudinal)
    horizontal[125, 150:270] = 1
    # Identical image evidence may represent a true boundary or an unrelated
    # longitudinal parking stripe; semantic truth is deliberately not assumed.
    return {'longitudinal_paint': inspect_candidate(longitudinal, candidate, support),
            'transverse_paint': inspect_candidate(horizontal, candidate, support),
            'visually_identical_wrong_longitudinal_paint': inspect_candidate(longitudinal, candidate, support)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    archive = np.load(args.capture)
    records = json.loads(str(archive['records_json']))
    frames = []
    for record in records:
        update = next(c for c in record['calls'] if c['name'] == 'update')
        fuse = next(c for c in record['calls'] if c['name'] == '_fuse')
        observation = fuse['observations'][1]
        if observation is None:
            continue
        points = archive[observation[0]['array']]
        support = (archive[observation[2]['array']] if len(observation) > 2
                   else np.isfinite(points[:, 0]))
        metrics = inspect_candidate(archive[update['paint']['array']], points, support)
        frames.append({'time': record['source']['source_media_time_s'],
                       'frame': record['source']['source_frame_index'], **metrics})
    report = {'synthetic': synthetic(), 'device_capture': frames,
              'note': 'Exploratory pixel geometry only; no semantic labels or runtime gate.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
