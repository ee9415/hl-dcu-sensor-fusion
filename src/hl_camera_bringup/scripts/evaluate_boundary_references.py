"""Compare sparse visible-paint reference points against recorded boundaries."""
import argparse
import json
from pathlib import Path

import numpy as np


def boundary_x(points, y, max_gap=6.0):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 2:
        return None
    p = p[np.isfinite(p).all(axis=1)]
    if len(p) < 2:
        return None
    p = p[np.argsort(p[:, 1])]
    i = int(np.searchsorted(p[:, 1], y))
    if i < len(p) and p[i, 1] == y:
        return float(p[i, 0])
    if i == 0 or i == len(p) or p[i, 1] - p[i-1, 1] > max_gap:
        return None
    return float(np.interp(y, p[i-1:i+1, 1], p[i-1:i+1, 0]))


def evaluate(labels, predictions):
    rows = []
    for case in labels['frames']:
        candidates = [p for p in predictions if p.get('input_source') == 'pc'
                      and p.get('source_frame_index') == case['source_frame_index']
                      and p.get('source_media_epoch') == case['source_media_epoch']]
        if len(candidates) != 1:
            raise ValueError(f"Expected one matching source frame, got {len(candidates)} for {case['source_frame_index']}")
        pred = candidates[0]
        for side in ('left', 'right'):
            for x, y in case.get(side, []):
                value = boundary_x(pred.get(side + '_boundary_image', []), y)
                rows.append({'frame': case['source_frame_index'], 'side': side, 'x': x,
                             'y': y, 'predicted_x': value,
                             'abs_error_px': None if value is None else abs(value-x)})
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--labels', type=Path, required=True)
    p.add_argument('--telemetry', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    labels = json.loads(args.labels.read_text(encoding='utf8'))
    predictions = [json.loads(line) for line in args.telemetry.read_text(encoding='utf8').splitlines() if line]
    rows = evaluate(labels, predictions)
    valid = [x['abs_error_px'] for x in rows if x['abs_error_px'] is not None]
    result = {'reference_status': labels.get('reference_status'), 'points': rows,
              'matched': len(valid), 'missing': len(rows)-len(valid),
              'median_abs_error_px': float(np.median(valid)) if valid else None,
              'max_abs_error_px': float(max(valid)) if valid else None}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
