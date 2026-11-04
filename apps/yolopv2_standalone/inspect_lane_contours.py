"""Offline inspection: python inspect_lane_contours.py masks.npz output_dir.

Input archive must contain original `rgb` and model `lane` mask arrays.
"""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from lane_contours import extract_lane_contours


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive')
    parser.add_argument('output')
    args = parser.parse_args()
    data = np.load(args.archive, allow_pickle=False)
    mask, rgb = data['lane'], data['rgb']
    if rgb.shape[:2] != mask.shape:
        raise ValueError('rgb and lane mask dimensions must match')
    started = time.perf_counter()
    result = extract_lane_contours(mask)
    elapsed = (time.perf_counter() - started) * 1000
    base = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    raw = base.copy()
    grouped = base.copy()
    cv2.drawContours(raw, result.rejected_contours, -1, (100, 100, 100), 1)
    for piece in result.pieces:
        cv2.drawContours(raw, [piece.contour], -1, (0, 255, 255), 1)
    palette = [(0,255,0), (255,180,0), (255,0,255), (0,180,255), (255,100,100)]
    for group_id, members in enumerate(result.groups):
        color = palette[group_id % len(palette)]
        for i in members:
            piece = result.pieces[i]
            cv2.drawContours(grouped, [piece.contour], -1, color, 1)
            cv2.putText(grouped, str(group_id),
                        (int(piece.centers[-1])+3, int(piece.rows[-1])),
                        cv2.FONT_HERSHEY_SIMPLEX, .3, color, 1)
    for group_id, curve in enumerate(result.group_curves):
        color = palette[group_id % len(palette)]
        for a,b,observed in zip(curve[:-1],curve[1:],result.curve_segment_observed[group_id]):
            cv2.line(grouped,(round(a[0]),round(a[1])),(round(b[0]),round(b[1])),
                     color if observed else (255,255,255),1)
    binary = (mask >= .5 if mask.dtype.kind == 'f' else mask > 0)
    panel = np.concatenate([base, cv2.cvtColor(binary.astype('uint8')*255,
                           cv2.COLOR_GRAY2BGR), raw, grouped], axis=1)
    panel = cv2.resize(panel, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    canvas = np.zeros((panel.shape[0]+36, panel.shape[1], 3), np.uint8)
    canvas[36:] = panel
    for i, label in enumerate(('RGB', 'MODEL MASK', 'CONTOURS (gray=rejected)',
                               'GROUPS (white=inferred segments)')):
        cv2.putText(canvas, label, (i*640+8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (255,255,255), 1)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output/'comparison.png'), canvas):
        raise OSError('failed to write comparison image')
    report = dict(source=args.archive, pieces=len(result.pieces),
                  rejected=len(result.rejected_contours), groups=result.groups,
                  links=result.links, processing_ms=elapsed,
                  group_curves=[curve.tolist() for curve in result.group_curves],
                  observed_curve_segments=[segments.tolist() for segments in result.curve_segment_observed],
                  note='No ego lane selection or guidance integration yet.')
    (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
