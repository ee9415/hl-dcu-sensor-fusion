"""Compare 320x320 and 416x256 TorchScript masks without labelled truth."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'apps/yolopv2_standalone'))
from postprocess import decode_heads, map_letterboxed_box_to_source, non_max_suppression


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--times', type=float, nargs='+', required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(4)
    model = torch.jit.load(str(a.weights), map_location='cpu').eval()
    video = cv2.VideoCapture(str(a.video))
    if not video.isOpened():
        raise RuntimeError('Video cannot be opened')
    records, panels = [], []
    try:
        for second in a.times:
            video.set(cv2.CAP_PROP_POS_MSEC, second * 1000)
            ok, frame = video.read()
            if not ok or abs(frame.shape[1] / frame.shape[0] - 16 / 9) > .01:
                raise ValueError(f'Missing frame or non-16:9 video at {second} s')
            image = cv2.resize(frame, (320, 180))
            outputs, detections = [], []
            for width, height in ((320, 320), (416, 256)):
                content_height = round(width * 9 / 16)
                top = (height - content_height) // 2
                rgb = np.zeros((height, width, 3), np.uint8)
                rgb[top:top+content_height] = cv2.resize(frame, (width, content_height))[:, :, ::-1]
                tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255)
                with torch.inference_mode():
                    detection, road, lane = model(tensor)
                heads, _ = detection
                boxes = non_max_suppression(decode_heads([head.numpy() for head in heads]))
                detections.append([
                    dict(box=map_letterboxed_box_to_source(box[:4], (320, 180), (width, height)).tolist(),
                         confidence=float(box[4]), label=int(box[5])) for box in boxes
                ])
                road = road.argmax(dim=1)[0, top:top+content_height].numpy().astype(np.uint8)
                lane = lane[0, 0, top:top+content_height].numpy()
                road = cv2.resize(road, (320, 180), interpolation=cv2.INTER_NEAREST) > 0
                lane = cv2.resize(lane, (320, 180), interpolation=cv2.INTER_LINEAR) >= .5
                outputs.append((road, lane))
            (road_a, lane_a), (road_b, lane_b) = outputs
            def iou(x, y):
                union = np.count_nonzero(x | y)
                return float(np.count_nonzero(x & y) / union) if union else None
            records.append(dict(second=second, road_iou=iou(road_a, road_b),
                                lane_iou=iou(lane_a, lane_b),
                                square_lane_pixels=int(lane_a.sum()),
                                larger_lane_pixels=int(lane_b.sum()),
                                square_detections=detections[0], larger_detections=detections[1]))
            row = []
            for label, lane in (('original', None), ('320x320', lane_a), ('416x256', lane_b)):
                overlay = image.copy()
                if lane is not None:
                    overlay[lane] = (0, 255, 255)
                overlay = cv2.resize(overlay, (640, 360))
                cv2.putText(overlay, f'{second:.1f}s {label}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                row.append(overlay)
            panels.append(np.hstack(row))
    finally:
        video.release()
    (a.output/'comparison.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    for start in range(0, len(panels), 4):
        cv2.imwrite(str(a.output/f'comparison_{start:03d}.jpg'), np.vstack(panels[start:start+4]))
    print(json.dumps([dict(second=r['second'], lane_iou=r['lane_iou'],
                           square_objects=len(r['square_detections']),
                           larger_objects=len(r['larger_detections'])) for r in records]))


if __name__ == '__main__':
    main()
