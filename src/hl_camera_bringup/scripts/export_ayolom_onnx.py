"""Export a trusted A-YOLOM checkpoint to static raw-head ONNX for OAK converters."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='Official A-YOLOM source checkout')
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--width', type=int, default=672)
    parser.add_argument('--height', type=int, default=384)
    args = parser.parse_args()
    if min(args.width, args.height) <= 0 or args.width % 32 or args.height % 32:
        parser.error('Input dimensions must be positive multiples of 32')
    sys.path.insert(0, str(args.source.resolve()))
    import cv2
    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch
    from ultralytics.nn.modules.head import Detect, Segment

    torch.set_num_threads(4)
    torch.manual_seed(27)
    checkpoint = torch.load(str(args.weights), map_location='cpu', weights_only=False)
    model = checkpoint['model'].float().eval()
    heads = [m for m in model.modules() if isinstance(m, (Detect, Segment))]
    if [type(h).__name__ for h in heads] != ['Detect', 'Segment', 'Segment', 'Detect']:
        raise ValueError('Expected base detection, road, lane and added detection heads')
    if [h.nc for h in heads] != [1, 1, 1, 4]:
        raise ValueError('Expected one base class and four added classes')
    for head in heads:
        head.export = False

    class RawHeads(torch.nn.Module):
        def __init__(self, network):
            super().__init__()
            self.network = network

        def forward(self, image):
            detection, road, lane, added = self.network(image)
            raw, extra = detection[1], added[1]
            return raw[0], raw[1], raw[2], road, lane, extra[0], extra[1], extra[2]

    wrapper = RawHeads(model).eval()
    bgr = cv2.imread(str(args.image))
    if bgr is None:
        raise ValueError(f'Cannot read {args.image}')
    h, w = bgr.shape[:2]
    ratio = min(args.width / w, args.height / h)
    resized = cv2.resize(bgr, (round(w * ratio), round(h * ratio)))
    image = np.full((args.height, args.width, 3), 114, np.uint8)
    top = (args.height - resized.shape[0]) // 2
    left = (args.width - resized.shape[1]) // 2
    image[top:top + resized.shape[0], left:left + resized.shape[1]] = resized[:, :, ::-1]
    sample = np.ascontiguousarray(image.transpose(2, 0, 1)[None], dtype=np.float32) / 255
    names = ['det_stride_8', 'det_stride_16', 'det_stride_32', 'drivable_area', 'lane_line',
             'person_tl_stride_8', 'person_tl_stride_16', 'person_tl_stride_32']
    # Raw regression distributions and class logits keep DFL/grid decoding and NMS on the host.
    # This avoids embedding platform-specific detection parsers in either compiled model.
    with torch.inference_mode():
        outputs = wrapper(torch.from_numpy(sample))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(wrapper, torch.from_numpy(sample), str(args.output), opset_version=12,
                      input_names=['images'], output_names=names, dynamic_axes=None,
                      do_constant_folding=True, dynamo=False)
    graph = onnx.load(str(args.output))
    onnx.checker.check_model(graph)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(args.output), sess_options=options,
                                   providers=['CPUExecutionProvider'])
    comparisons = []
    rng = np.random.default_rng(27)
    for kind, array in [('image', sample), ('random', rng.random(sample.shape, dtype=np.float32))]:
        with torch.inference_mode():
            reference = wrapper(torch.from_numpy(array))
        actual = session.run(names, {'images': array})
        for name, expected, result in zip(names, reference, actual):
            expected = expected.numpy()
            np.testing.assert_allclose(result, expected, atol=2e-4, rtol=1e-4)
            comparisons.append(dict(input=kind, name=name, shape=list(result.shape),
                                    max_abs_error=float(np.abs(result - expected).max())))
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    report = dict(source=str(args.weights), source_sha256=digest(args.weights),
                  source_repository='https://github.com/JiayuanWang-JW/YOLOv8-multi-task',
                  source_commit=subprocess.check_output(['git', '-C', str(args.source), 'rev-parse', 'HEAD'], text=True).strip(),
                  onnx_sha256=digest(args.output), input_shape=list(sample.shape),
                  input_order='RGB', normalization='pixel/255', letterbox_value=114,
                  opset=12, detection_classes=[heads[0].nc, heads[3].nc], reg_max=heads[0].reg_max,
                  checkpoint_note=checkpoint.get('person_note'),
                  class_mapping={'base': ['vehicle'], 'added': ['person', 'tl_red', 'tl_yellow', 'tl_green']},
                  names=model.names, outputs={n: list(o.shape) for n, o in zip(names, outputs)},
                  detection_postprocess='YOLOv8 DFL, stride grids, sigmoid class logits, NMS; no objectness',
                  segmentation_postprocess='Bilinear resize logits then channel argmax; preserve background channel',
                  comparisons=comparisons, torch_version=torch.__version__,
                  onnx_version=onnx.__version__, onnxruntime_version=ort.__version__,
                  rvc2_status='pending', rvc4_status='pending', device_validation='not performed')
    args.output.with_suffix('.validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
