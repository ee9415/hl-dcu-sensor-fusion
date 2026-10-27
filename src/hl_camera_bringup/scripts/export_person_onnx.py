"""Export the user-supplied YOLOPv2 TorchScript with verified static outputs."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import onnx
import onnxruntime as ort
import torch


class ExportHeads(torch.nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, image: torch.Tensor):
        detection, road, lane = self.model(image)
        heads, _ = detection
        return heads[0], heads[1], heads[2], road, lane


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--image', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(4)
    model = ExportHeads(torch.jit.load(str(a.weights), map_location='cpu').eval()).eval()
    frozen = torch.jit.freeze(torch.jit.script(model))
    bgr = cv2.imread(str(a.image))
    if bgr is None:
        raise ValueError(f'Cannot read {a.image}')
    h, w = bgr.shape[:2]
    scale = min(320 / h, 320 / w)
    resized = cv2.resize(bgr, (round(w * scale), round(h * scale)))
    rgb = np.full((320, 320, 3), 114, np.uint8)
    top, left = (320 - resized.shape[0]) // 2, (320 - resized.shape[1]) // 2
    rgb[top:top+resized.shape[0], left:left+resized.shape[1]] = resized[:, :, ::-1]
    sample = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255
    names = ['det_stride_8', 'det_stride_16', 'det_stride_32', 'drivable_area', 'lane_line']
    torch.onnx.export(frozen, torch.from_numpy(sample), str(a.output), opset_version=17,
                      input_names=['images'], output_names=names, dynamic_axes=None,
                      do_constant_folding=True, dynamo=False)
    graph = onnx.load(str(a.output))
    onnx.checker.check_model(graph)
    options = ort.SessionOptions(); options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(a.output), sess_options=options,
                                   providers=['CPUExecutionProvider'])
    comparisons = []
    rng = np.random.default_rng(27)
    for kind, array in [('image', sample), ('random', rng.random(sample.shape, dtype=np.float32))]:
        with torch.inference_mode():
            reference = model(torch.from_numpy(array))
        outputs = session.run(names, {'images': array})
        for name, ref, actual in zip(names, reference, outputs):
            ref = ref.numpy()
            np.testing.assert_allclose(actual, ref, atol=2e-4, rtol=1e-4)
            comparisons.append(dict(input=kind, name=name, shape=list(actual.shape),
                                    max_abs_error=float(np.max(np.abs(actual-ref)))))
    metadata = dict(source=str(a.weights), source_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),
                    onnx_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),
                    input_shape=[1,3,320,320], input_order='RGB', normalization='pixel/255',
                    opset=17, comparisons=comparisons, torch_version=torch.__version__,
                    onnx_version=onnx.__version__, onnxruntime_version=ort.__version__,
                    rvc4_conversion_status='pending')
    a.output.with_suffix('.validation.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata, indent=2), flush=True)


if __name__ == '__main__':
    main()
