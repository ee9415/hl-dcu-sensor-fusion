import json
import time

import depthai as dai
import numpy as np

from postprocess import (
    diagnostic_bgr_frame,
    decode_heads,
    map_letterboxed_box_to_source,
    non_max_suppression,
    segmentation_masks,
)


class DiagnosticFrameSource(dai.node.ThreadedHostNode):
    def __init__(self) -> None:
        super().__init__()
        self.out = self.createOutput(
            possibleDatatypes=[
                dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)
            ]
        )

    def run(self) -> None:
        frame_data = diagnostic_bgr_frame()
        sequence = 0
        while self.isRunning():
            frame = dai.ImgFrame()
            frame.setCvFrame(frame_data, dai.ImgFrame.Type.BGR888i)
            frame.setSequenceNum(sequence)
            self.out.send(frame)
            sequence += 1
            time.sleep(0.2)


class Yolopv2Preprocess(dai.node.HostNode):
    """Convert a BGR888i ImgFrame to the DLC's normalized RGB tensor."""

    def __init__(self) -> None:
        super().__init__()
        self.tensor_out = self.createOutput(
            possibleDatatypes=[
                dai.Node.DatatypeHierarchy(dai.DatatypeEnum.NNData, True)
            ]
        )

    def build(self, frame: dai.Node.Output) -> "Yolopv2Preprocess":
        self.link_args(frame)
        return self

    def process(self, frame: dai.ImgFrame) -> None:
        bgr = frame.getCvFrame()
        if bgr.shape != (320, 320, 3):
            raise ValueError(f"Expected a 320x320 BGR frame, got {bgr.shape}")

        # The original YOLOPv2 ONNX input is RGB in the [0, 1] range.  RVC4
        # DLC tensors use NHWC at their external boundary even though the ONNX
        # graph was authored as NCHW.  NNArchive preprocessing metadata is not
        # executed when a node output is linked manually, so do it explicitly.
        rgb_nhwc = np.ascontiguousarray(
            bgr[None, :, :, ::-1].astype(np.float16) / np.float16(255.0)
        )
        nn_data = dai.NNData()
        nn_data.addTensor(
            "images",
            rgb_nhwc,
            dataType=dai.TensorInfo.DataType.FP16,
        )
        nn_data.setTimestamp(frame.getTimestamp())
        nn_data.setSequenceNum(frame.getSequenceNum())
        self.tensor_out.send(nn_data)


class Yolopv2NnProbe(dai.node.HostNode):
    TENSOR_NAMES = (
        "det_stride_8",
        "det_stride_16",
        "det_stride_32",
        "drivable_area",
        "lane_line",
    )

    def __init__(self) -> None:
        super().__init__()
        self._frames = 0
        self._started_at = time.monotonic()
        self._last_status_at = 0.0
        self._tensor_shapes = None

    def build(self, nn: dai.NNData) -> "Yolopv2NnProbe":
        self.link_args(nn)
        return self

    def process(self, nn_data: dai.NNData) -> None:
        self._frames += 1
        now = time.monotonic()
        if self._tensor_shapes is None:
            tensors = {
                name: nn_data.getTensor(
                    name,
                    dequantize=True,
                    storageOrder=dai.TensorInfo.StorageOrder.NCHW,
                )
                for name in self.TENSOR_NAMES
            }
            self._tensor_shapes = {
                name: list(tensor.shape) for name, tensor in tensors.items()
            }
            tensor_stats = {
                name: {
                    "min": round(float(np.min(tensor)), 6),
                    "max": round(float(np.max(tensor)), 6),
                    "mean": round(float(np.mean(tensor)), 6),
                    "std": round(float(np.std(tensor)), 6),
                }
                for name, tensor in tensors.items()
            }
            print(f"NN_TENSOR_STATS {json.dumps(tensor_stats)}", flush=True)

            native_tensors = {
                name: nn_data.getTensor(
                    name,
                    dequantize=True,
                    storageOrder=dai.TensorInfo.StorageOrder.NHWC,
                )
                for name in self.TENSOR_NAMES
            }
            layout_comparison = {}
            for label, candidate in (
                ("requested_nchw", tensors),
                ("requested_nhwc", native_tensors),
            ):
                decoded = non_max_suppression(
                    decode_heads([candidate[name] for name in self.TENSOR_NAMES[:3]])
                )
                drivable, lane = segmentation_masks(
                    candidate[self.TENSOR_NAMES[3]],
                    candidate[self.TENSOR_NAMES[4]],
                    (1920, 1080),
                )
                layout_comparison[label] = {
                    "detections": int(len(decoded)),
                    "drivable_pixels": int(np.count_nonzero(drivable)),
                    "lane_pixels": int(np.count_nonzero(lane)),
                }
            print(
                f"NN_LAYOUT_COMPARISON {json.dumps(layout_comparison)}",
                flush=True,
            )

        if self._frames == 1 or now - self._last_status_at >= 5.0:
            elapsed = max(now - self._started_at, 1e-6)
            payload = {
                "status": "nn_running",
                "frames": self._frames,
                "average_fps": round(self._frames / elapsed, 2),
                "tensor_shapes": self._tensor_shapes,
            }
            print(f"NN_STATUS {json.dumps(payload)}", flush=True)
            self._last_status_at = now


class Yolopv2Postprocess(dai.node.HostNode):
    TENSOR_NAMES = (
        "det_stride_8",
        "det_stride_16",
        "det_stride_32",
        "drivable_area",
        "lane_line",
    )

    def __init__(self) -> None:
        super().__init__()
        img_frame_type = [
            dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)
        ]
        self.detections = self.createOutput(
            possibleDatatypes=[
                dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgDetections, True)
            ]
        )
        self.drivable = self.createOutput(possibleDatatypes=img_frame_type)
        self.lane = self.createOutput(possibleDatatypes=img_frame_type)
        self.result = self.createOutput(possibleDatatypes=img_frame_type)
        self._source_size = (1920, 1080)
        self._result_size = (960, 540)
        self._confidence_threshold = 0.30
        self._iou_threshold = 0.45
        self._frames = 0
        self._started_at = time.monotonic()
        self._last_health_at = self._started_at

    def build(
        self,
        preview: dai.Node.Output,
        nn: dai.Node.Output,
    ) -> "Yolopv2Postprocess":
        self.link_args(preview, nn)
        return self

    def set_source_size(self, width: int, height: int) -> None:
        self._source_size = (width, height)

    @staticmethod
    def _tensor(nn_data: dai.NNData, name: str) -> np.ndarray:
        return nn_data.getTensor(
            name,
            dequantize=True,
            storageOrder=dai.TensorInfo.StorageOrder.NCHW,
        )

    @staticmethod
    def _mask_frame(mask: np.ndarray, nn_data: dai.NNData) -> dai.ImgFrame:
        # OAK Visualizer 3.5.2 does not render GRAY8 streams. Expand the
        # binary mask to three channels so it can be displayed as BGR888i.
        bgr_mask = np.repeat(mask[:, :, None], 3, axis=2)
        frame = dai.ImgFrame()
        frame.setCvFrame(
            np.ascontiguousarray(bgr_mask),
            dai.ImgFrame.Type.BGR888i,
        )
        frame.setTimestamp(nn_data.getTimestamp())
        frame.setSequenceNum(nn_data.getSequenceNum())
        return frame

    @staticmethod
    def _blend_mask(
        frame: np.ndarray,
        mask: np.ndarray,
        color: tuple[int, int, int],
        alpha: float,
    ) -> None:
        selected = mask != 0
        if not np.any(selected):
            return
        source = frame[selected].astype(np.float32)
        overlay = np.asarray(color, dtype=np.float32)
        frame[selected] = (source * (1.0 - alpha) + overlay * alpha).astype(
            np.uint8
        )

    @staticmethod
    def _draw_box(
        frame: np.ndarray,
        detection: dai.ImgDetection,
        color: tuple[int, int, int] = (0, 255, 255),
        thickness: int = 3,
    ) -> None:
        height, width = frame.shape[:2]
        x1 = int(np.clip(detection.xmin, 0.0, 1.0) * (width - 1))
        y1 = int(np.clip(detection.ymin, 0.0, 1.0) * (height - 1))
        x2 = int(np.clip(detection.xmax, 0.0, 1.0) * (width - 1))
        y2 = int(np.clip(detection.ymax, 0.0, 1.0) * (height - 1))
        if x2 <= x1 or y2 <= y1:
            return
        for offset in range(thickness):
            left = min(x1 + offset, width - 1)
            right = max(x2 - offset, 0)
            top = min(y1 + offset, height - 1)
            bottom = max(y2 - offset, 0)
            frame[top, left : right + 1] = color
            frame[bottom, left : right + 1] = color
            frame[top : bottom + 1, left] = color
            frame[top : bottom + 1, right] = color

    def _result_frame(
        self,
        preview: dai.ImgFrame,
        drivable: np.ndarray,
        lane: np.ndarray,
        detections: list[dai.ImgDetection],
    ) -> dai.ImgFrame:
        source = preview.getCvFrame()
        result_width, result_height = self._result_size
        source_height, source_width = source.shape[:2]
        if (source_width, source_height) != self._source_size:
            raise ValueError(
                f"Expected source {self._source_size}, got {(source_width, source_height)}"
            )

        # 1920x1080 -> 960x540 and 320x180 -> 960x540 are exact integer
        # conversions, avoiding an OpenCV dependency in the device container.
        rendered = np.ascontiguousarray(source[::2, ::2])
        if rendered.shape[:2] != (result_height, result_width):
            raise ValueError(f"Unexpected result shape: {rendered.shape}")
        drivable_up = np.repeat(np.repeat(drivable, 3, axis=0), 3, axis=1)
        lane_up = np.repeat(np.repeat(lane, 3, axis=0), 3, axis=1)

        self._blend_mask(rendered, drivable_up, (0, 180, 0), 0.45)
        self._blend_mask(rendered, lane_up, (0, 0, 255), 0.70)
        for detection in detections:
            self._draw_box(rendered, detection)

        result = dai.ImgFrame()
        result.setCvFrame(rendered, dai.ImgFrame.Type.NV12)
        result.setTimestamp(preview.getTimestamp())
        result.setSequenceNum(preview.getSequenceNum())
        return result

    def process(self, preview: dai.ImgFrame, nn_data: dai.NNData) -> None:
        started_at = time.monotonic()
        heads = [self._tensor(nn_data, name) for name in self.TENSOR_NAMES[:3]]
        prediction = decode_heads(heads)
        decoded = non_max_suppression(
            prediction,
            confidence_threshold=self._confidence_threshold,
            iou_threshold=self._iou_threshold,
        )

        detection_messages = []
        for x1, y1, x2, y2, confidence, label in decoded:
            normalized = map_letterboxed_box_to_source(
                np.asarray((x1, y1, x2, y2)), self._source_size
            )
            if normalized[2] <= normalized[0] or normalized[3] <= normalized[1]:
                continue
            detection = dai.ImgDetection()
            detection.label = int(label)
            detection.confidence = float(confidence)
            detection.xmin, detection.ymin, detection.xmax, detection.ymax = map(
                float, normalized
            )
            detection_messages.append(detection)

        detections = dai.ImgDetections()
        detections.detections = detection_messages
        detections.setTimestamp(nn_data.getTimestamp())
        detections.setSequenceNum(nn_data.getSequenceNum())
        self.detections.send(detections)

        drivable, lane = segmentation_masks(
            self._tensor(nn_data, self.TENSOR_NAMES[3]),
            self._tensor(nn_data, self.TENSOR_NAMES[4]),
            self._source_size,
        )
        self.drivable.send(self._mask_frame(drivable, nn_data))
        self.lane.send(self._mask_frame(lane, nn_data))
        self.result.send(
            self._result_frame(preview, drivable, lane, detection_messages)
        )

        self._frames += 1
        now = time.monotonic()
        if now - self._last_health_at >= 5.0:
            elapsed = max(now - self._started_at, 1e-6)
            payload = {
                "status": "running",
                "frames": self._frames,
                "average_fps": round(self._frames / elapsed, 2),
                "postprocess_ms": round((now - started_at) * 1000.0, 2),
                "detections": len(detection_messages),
                "input_shape": [1, 3, 320, 320],
            }
            print(f"HEALTH {json.dumps(payload)}", flush=True)
            self._last_health_at = now
