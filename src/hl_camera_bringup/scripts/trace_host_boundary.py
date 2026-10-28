"""Offline FP32 diagnostic replay of candidate, prior and final boundaries."""
import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/yolopv2_standalone'))
from image_boundaries import ImageBoundaryTracker
from postprocess import decode_heads, non_max_suppression


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start', type=float, default=60)
    parser.add_argument('--seconds', type=float, default=5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 4
    session = ort.InferenceSession(str(args.model), opts, providers=['CPUExecutionProvider'])
    tracker = ImageBoundaryTracker()
    old_detect, old_fuse = tracker._detect, tracker._fuse
    state = {}

    def detect(paint, side, prior):
        result = old_detect(paint, side, prior)
        if side == 1:
            state['paint'] = paint.copy()
        return result

    def fuse(observations, predicted, motion, motion_score, step):
        state['candidate'] = None if observations[1] is None else observations[1][0].copy()
        state['support'] = None if observations[1] is None or len(observations[1]) < 3 else observations[1][2].copy()
        state['prior'] = None if predicted[1] is None else predicted[1].copy()
        return old_fuse(observations, predicted, motion, motion_score, step)

    tracker._detect, tracker._fuse = detect, fuse
    source = cv2.VideoCapture(str(args.video))
    fps = source.get(cv2.CAP_PROP_FPS)
    results = []
    for index in range(int(args.seconds * 10)):
        position = args.start + index / 10
        source.set(cv2.CAP_PROP_POS_MSEC, position * 1000)
        ok, frame = source.read()
        if not ok:
            raise RuntimeError(f'Cannot read video at {position}')
        image = np.zeros((192, 320, 3), np.uint8)
        small = cv2.resize(frame, (320, 180))
        image[6:186] = small[:, :, ::-1]
        tensor = np.ascontiguousarray(image.transpose(2, 0, 1)[None], dtype=np.float32) / 255
        out = session.run(None, {'images': tensor})
        road = np.argmax(out[3], axis=1)[0, 6:186].astype(np.uint8) * 255
        lane = (out[4][0, 0, 6:186] >= .5).astype(np.uint8) * 255
        boxes = []
        for x1, y1, x2, y2, confidence, label in non_max_suppression(decode_heads(out[:3])):
            x1, y1, x2, y2 = np.clip([x1/320, (y1-6)/180, x2/320, (y2-6)/180], 0, 1)
            boxes.append(SimpleNamespace(xmin=x1, ymin=y1, xmax=x2, ymax=y2,
                                         confidence=float(confidence), label=int(label)))
        state.clear()
        result = tracker.update(road, lane, index / 10, rgb=frame, detections=boxes)
        arrays = {'image_bgr': small, 'road': road, 'lane': lane,
                  'paint': state['paint'], 'result': result.image_right}
        for key in ('candidate', 'prior', 'support'):
            if state.get(key) is not None:
                arrays[key] = state[key]
        segment_list = tracker.region_segments[1]
        for n, segment in enumerate(segment_list):
            arrays[f'segment_{n}'] = np.asarray(segment)
        np.savez_compressed(args.output / f'{index:03}.npz', **arrays)
        results.append({'index': index, 'video_seconds': position,
                        'source_frame': int(round(position * fps)),
                        'state': result.right_state, 'confidence': result.right_confidence,
                        'residual': result.right_residual_px,
                        'diagnostics': result.detection_diagnostics[1],
                        'segments': len(segment_list)})
    source.release()
    (args.output / 'trace.json').write_text(json.dumps(results, indent=2))
    print(f'Captured {len(results)} frames to {args.output}')


if __name__ == '__main__':
    main()
