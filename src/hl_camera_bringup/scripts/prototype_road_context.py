"""Offline study of road-mask context around candidate boundary samples."""
import argparse
import json
from pathlib import Path

import numpy as np


def road_context(road, points, support, side, distance=8):
    """Sample road membership inside/outside a curve in image coordinates.

    A detected lane boundary can lie inside a broader drivable mask. These
    statistics must not be interpreted as labels or used as a rejection gate.
    """
    mask = np.asarray(road) > 0
    points = np.asarray(points, dtype=float)
    support = np.asarray(support, dtype=bool)
    if mask.shape != (180, 320) or points.ndim != 2 or points.shape[1] != 2 or len(points) != len(support):
        raise ValueError('Invalid road mask or boundary shape')
    if side not in ('left', 'right') or not 1 <= distance <= 40:
        raise ValueError('Invalid side or sample distance')
    selected = np.rint(points[support & np.isfinite(points).all(axis=1)]).astype(int)
    sign = 1 if side == 'right' else -1
    valid = ((selected[:, 1] >= 0) & (selected[:, 1] < 180) &
             (selected[:, 0]-distance >= 0) & (selected[:, 0]+distance < 320))
    selected = selected[valid]
    if not len(selected):
        return {'rows': 0, 'inside_fraction': None, 'outside_fraction': None,
                'inside_to_outside_fraction': None, 'both_road_fraction': None}
    y, x = selected[:, 1], selected[:, 0]
    inside = mask[y, x-sign*distance]
    outside = mask[y, x+sign*distance]
    return {'rows': len(selected), 'inside_fraction': float(inside.mean()),
            'outside_fraction': float(outside.mean()),
            'inside_to_outside_fraction': float((inside & ~outside).mean()),
            'both_road_fraction': float((inside & outside).mean())}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
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
        road = archive[update['road']['array']]
        item = {'frame': record['source']['source_frame_index'],
                'time': record['source']['source_media_time_s'], 'samples': {}}
        for offset in (0, -20, 20):
            shifted = points.copy()
            shifted[:, 0] += offset
            item['samples'][str(offset)] = {
                str(distance): road_context(road, shifted, support, 'right', distance)
                for distance in (4, 8, 16)
            }
        frames.append(item)
    result = {'frames': frames,
              'note': 'Offsets are artificial perturbations, not verified false candidates. No runtime gate.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
