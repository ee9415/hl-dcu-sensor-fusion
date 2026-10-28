"""Compare ONNX lane/road outputs on identical video frames, not ground truth."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--square', type=Path, required=True)
    parser.add_argument('--rectangular', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--times', type=float, nargs='+', required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    sessions = [ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
                for path in (args.square, args.rectangular)]
    video = cv2.VideoCapture(str(args.video))
    if not video.isOpened():
        raise RuntimeError('Video cannot be opened')
    records, panels = [], []
    try:
        for second in args.times:
            video.set(cv2.CAP_PROP_POS_MSEC, second * 1000)
            ok, frame = video.read()
            if not ok:
                raise RuntimeError(f'Cannot decode frame at {second} s')
            if abs(frame.shape[1] / frame.shape[0] - 16 / 9) > .01:
                raise ValueError('Comparison requires a 16:9 source')
            image = cv2.resize(frame, (320, 180))
            outputs = []
            for session, height in zip(sessions, (320, 192)):
                top = (height - 180) // 2
                tensor_image = np.zeros((height, 320, 3), np.uint8)
                tensor_image[top:top + 180] = image[:, :, ::-1]
                tensor = np.ascontiguousarray(tensor_image.transpose(2, 0, 1)[None], dtype=np.float32) / 255
                prediction = session.run(None, {'images': tensor})
                road = np.argmax(prediction[3], axis=1)[0][top:top + 180] > 0
                lane = prediction[4][0, 0, top:top + 180] >= .5
                outputs.append((road, lane))
            def iou(a, b):
                union = np.count_nonzero(a | b)
                return float(np.count_nonzero(a & b) / union) if union else None
            (road_a, lane_a), (road_b, lane_b) = outputs
            records.append(dict(second=second, lane_iou=iou(lane_a, lane_b),
                                road_iou=iou(road_a, road_b),
                                square_lane_pixels=int(lane_a.sum()),
                                rectangular_lane_pixels=int(lane_b.sum())))
            row = []
            for label, lane in (('original', None), ('320x320', lane_a), ('320x192', lane_b)):
                overlay = image.copy()
                if lane is not None:
                    overlay[lane] = (0, 255, 255)
                overlay = cv2.resize(overlay, (640, 360))
                cv2.putText(overlay, f'{second:.1f}s {label}', (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                row.append(overlay)
            panels.append(np.hstack(row))
    finally:
        video.release()
    (args.output/'comparison.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
    for start in range(0, len(panels), 4):
        cv2.imwrite(str(args.output/f'comparison_{start:03d}.jpg'), np.vstack(panels[start:start+4]))
    print(json.dumps(records))


if __name__ == '__main__':
    main()
