"""Replay identical sequential frames through two TorchScript input shapes.

Host FP32 comparison only: output differences are not labelled accuracy or
device FPS. Each segment starts independent tracker histories.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from postprocess import decode_heads, map_letterboxed_box_to_source, non_max_suppression


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--starts', type=float, nargs='+', required=True)
    p.add_argument('--frames', type=int, default=20)
    a = p.parse_args()
    torch.set_num_threads(4)
    model = torch.jit.load(str(a.weights), map_location='cpu').eval()
    video = cv2.VideoCapture(str(a.video))
    if not video.isOpened():
        raise RuntimeError('Video cannot be opened')
    records = []
    try:
        for start in a.starts:
            trackers = [ImageBoundaryTracker(), ImageBoundaryTracker()]
            for index in range(a.frames):
                second = start + index * .1
                video.set(cv2.CAP_PROP_POS_MSEC, second * 1000)
                ok, frame = video.read()
                if not ok or abs(frame.shape[1]/frame.shape[0] - 16/9) > .01:
                    raise ValueError(f'Missing frame or non-16:9 video at {second}')
                results, counts, mask_counts, box_records = [], [], [], []
                for (width, height), tracker in zip(((320, 320), (416, 256)), trackers):
                    h = round(width * 9/16)
                    top = (height-h)//2
                    rgb = np.zeros((height, width, 3), np.uint8)
                    rgb[top:top+h] = cv2.resize(frame, (width, h))[:, :, ::-1]
                    tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32)/255)
                    with torch.inference_mode():
                        detection, road, lane = model(tensor)
                    heads, _ = detection
                    boxes = non_max_suppression(decode_heads([v.numpy() for v in heads]))
                    objects = []
                    for box in boxes:
                        xy = map_letterboxed_box_to_source(box[:4], (320, 180), (width, height))
                        objects.append(SimpleNamespace(xmin=float(xy[0]), ymin=float(xy[1]),
                                                       xmax=float(xy[2]), ymax=float(xy[3]),
                                                       confidence=float(box[4]), label=int(box[5])))
                    drivable = road.argmax(dim=1)[0, top:top+h].numpy().astype(np.uint8)
                    lane = lane[0, 0, top:top+h].numpy()
                    drivable = cv2.resize(drivable, (320, 180), interpolation=cv2.INTER_NEAREST)*255
                    lane = (cv2.resize(lane, (320, 180), interpolation=cv2.INTER_LINEAR) >= .5).astype(np.uint8)*255
                    mask_counts.append(dict(drivable=int(np.count_nonzero(drivable)), lane=int(np.count_nonzero(lane))))
                    box_records.append([dict(xmin=o.xmin, ymin=o.ymin, xmax=o.xmax, ymax=o.ymax,
                                             confidence=o.confidence, label=o.label) for o in objects])
                    results.append(tracker.update(drivable, lane, index*.1, rgb=frame, detections=objects))
                    counts.append(len(objects))
                row = dict(segment=start, second=round(second, 3), objects=counts,
                           masks=mask_counts, boxes=box_records,
                           states=[[r.left_state, r.right_state] for r in results],
                           coefficients=[{key: r.telemetry().get(key) for key in
                                          ('left_image_coefficients', 'right_image_coefficients')} for r in results],
                           differences={})
                for side in ('left', 'right'):
                    x, y = [getattr(r, 'image_'+side) for r in results]
                    valid = np.isfinite(x[:, 0]) & np.isfinite(y[:, 0])
                    row['differences'][side] = float(np.mean(abs(x[valid,0]-y[valid,0]))) if valid.any() else None
                records.append(row)
    finally:
        video.release()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(records, indent=2), encoding='utf-8')
    for start in a.starts:
        segment = [r for r in records if r['segment'] == start]
        print(json.dumps(dict(segment=start, frames=len(segment), states=[dict(Counter('/'.join(r['states'][i]) for r in segment)) for i in (0, 1)],
                              differing_object_counts=sum(r['objects'][0] != r['objects'][1] for r in segment),
                              median_pixel_difference={side: float(np.median([r['differences'][side] for r in segment if r['differences'][side] is not None]))
                                                       if any(r['differences'][side] is not None for r in segment) else None for side in ('left', 'right')})))


if __name__ == '__main__':
    main()
