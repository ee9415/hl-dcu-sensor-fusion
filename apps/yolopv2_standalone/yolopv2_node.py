import json
import time

import depthai as dai
import numpy as np
from corridor import CorridorTracker
from adaptive_order import AdaptiveOrderSelector
from conservative_centerline import ConservativeCenterline
from turn_preview import TurnPreview, TurnMailbox, draw_turn

from centerline import (
    CenterlineEstimate,
    CenterlineTemporalFilter,
    draw_centerline_overlay,
    estimate_centerline,
    render_bev,
)
from postprocess import (
    SpatialMeasurement,
    diagnostic_bgr_frame,
    decode_heads,
    estimate_spatial_from_depth,
    map_letterboxed_box_to_source,
    non_max_suppression,
    segmentation_masks,
)


FONT_5X7 = {
    " ": ("00000",)*7,
    "A": ("01110","10001","10001","11111","10001","10001","10001"),
    "B": ("11110","10001","10001","11110","10001","10001","11110"),
    "D": ("11110","10001","10001","10001","10001","10001","11110"),
    "E": ("11111","10000","10000","11110","10000","10000","11111"),
    "F": ("11111","10000","10000","11110","10000","10000","10000"),
    "G": ("01110","10001","10000","10111","10001","10001","01110"),
    "H": ("10001","10001","10001","11111","10001","10001","10001"),
    "I": ("11111","00100","00100","00100","00100","00100","11111"),
    "L": ("10000","10000","10000","10000","10000","10000","11111"),
    "N": ("10001","11001","11001","10101","10011","10011","10001"),
    "O": ("01110","10001","10001","10001","10001","10001","01110"),
    "P": ("11110","10001","10001","11110","10000","10000","10000"),
    "R": ("11110","10001","10001","11110","10100","10010","10001"),
    "S": ("01111","10000","10000","01110","00001","00001","11110"),
    "T": ("11111","00100","00100","00100","00100","00100","00100"),
    "U": ("10001","10001","10001","10001","10001","10001","01110"),
    "V": ("10001","10001","10001","10001","10001","01010","00100"),
    "W": ("10001","10001","10001","10101","10101","10101","01010"),
    "Y": ("10001","10001","01010","00100","00100","00100","00100"),
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00010", "00100", "01000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "4": ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    "5": ("11111", "10000", "10000", "11110", "00001", "00001", "11110"),
    "6": ("01110", "10000", "10000", "11110", "10001", "10001", "01110"),
    "7": ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    "8": ("01110", "10001", "10001", "01110", "10001", "10001", "01110"),
    "9": ("01110", "10001", "10001", "01111", "00001", "00001", "01110"),
    ".": ("00000", "00000", "00000", "00000", "00000", "00110", "00110"),
    "?": ("01110", "10001", "00001", "00010", "00100", "00000", "00100"),
    "=": ("00000", "11111", "00000", "11111", "00000", "00000", "00000"),
    "Z": ("11111", "00001", "00010", "00100", "01000", "10000", "11111"),
    "m": ("00000", "00000", "11010", "10101", "10101", "10101", "10101"),
}


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
        self.spatial_detections = self.createOutput(
            possibleDatatypes=[
                dai.Node.DatatypeHierarchy(
                    dai.DatatypeEnum.SpatialImgDetections, True
                )
            ]
        )
        self.drivable = self.createOutput(possibleDatatypes=img_frame_type)
        self.lane = self.createOutput(possibleDatatypes=img_frame_type)
        self.bev = self.createOutput(possibleDatatypes=img_frame_type)
        self.result = self.createOutput(possibleDatatypes=img_frame_type)
        self._source_size = (1920, 1080)
        self._result_size = (960, 540)
        self._confidence_threshold = 0.30
        self._iou_threshold = 0.45
        self._frames = 0
        self._started_at = time.monotonic()
        self._last_health_at = self._started_at
        self._centerline_filter = CenterlineTemporalFilter(measurement_following=True)
        self._corridor_tracker = CorridorTracker()
        self._order_selector = AdaptiveOrderSelector()
        self._shape_limiter = ConservativeCenterline()
        self._turn_preview = TurnPreview()
        self._turn_mailbox = TurnMailbox()
        self._turn_points = None
        self._limit_samples = []
        self._centerline_samples = []

    def build(
        self,
        preview: dai.Node.Output,
        nn: dai.Node.Output,
        aligned_depth: dai.Node.Output,
    ) -> "Yolopv2Postprocess":
        self.link_args(preview, nn, aligned_depth)
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
    def _bev_frame(bev: np.ndarray, nn_data: dai.NNData) -> dai.ImgFrame:
        frame = dai.ImgFrame()
        frame.setCvFrame(np.ascontiguousarray(bev), dai.ImgFrame.Type.NV12)
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
    def _box_pixels(
        frame: np.ndarray,
        detection: dai.ImgDetection,
    ) -> tuple[int, int, int, int]:
        height, width = frame.shape[:2]
        return (
            int(np.clip(detection.xmin, 0.0, 1.0) * (width - 1)),
            int(np.clip(detection.ymin, 0.0, 1.0) * (height - 1)),
            int(np.clip(detection.xmax, 0.0, 1.0) * (width - 1)),
            int(np.clip(detection.ymax, 0.0, 1.0) * (height - 1)),
        )

    @staticmethod
    def _draw_box(
        frame: np.ndarray,
        detection: dai.ImgDetection,
        color: tuple[int, int, int] = (0, 255, 255),
        thickness: int = 3,
    ) -> None:
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = Yolopv2Postprocess._box_pixels(frame, detection)
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

    @staticmethod
    def _draw_text(
        frame: np.ndarray,
        text: str,
        x: int,
        y: int,
        color: tuple[int, int, int],
        scale: int = 2,
    ) -> None:
        height, width = frame.shape[:2]
        cursor_x = max(x, 0)
        for character in text:
            glyph = FONT_5X7.get(character, FONT_5X7["?"])
            for row_index, row in enumerate(glyph):
                for column_index, selected in enumerate(row):
                    if selected != "1":
                        continue
                    left = cursor_x + column_index * scale
                    top = y + row_index * scale
                    if left >= width or top >= height:
                        continue
                    right = min(left + scale, width)
                    bottom = min(top + scale, height)
                    if right > 0 and bottom > 0:
                        frame[max(top, 0) : bottom, max(left, 0) : right] = color
            cursor_x += 6 * scale

    @staticmethod
    def _draw_distance(
        frame: np.ndarray,
        detection: dai.ImgDetection,
        measurement: SpatialMeasurement,
    ) -> None:
        x1, y1, _, _ = Yolopv2Postprocess._box_pixels(frame, detection)
        if measurement.valid:
            text = f"Z={measurement.z_mm / 1000.0:.1f}m"
            color = (0, 255, 255)
        else:
            text = "Z=?"
            color = (0, 0, 255)
        Yolopv2Postprocess._draw_text(
            frame,
            text,
            x1,
            max(y1 - 18, 2),
            color,
        )

    def _result_frame(
        self,
        preview: dai.ImgFrame,
        drivable: np.ndarray,
        lane: np.ndarray,
        detections: list[dai.ImgDetection],
        measurements: list[SpatialMeasurement],
        centerline: CenterlineEstimate,
        measured_centerline: CenterlineEstimate,
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
        draw_centerline_overlay(rendered, measured_centerline,
                                (drivable.shape[1], drivable.shape[0]), raw=True)
        if self._turn_preview.mode == 'road':
            draw_centerline_overlay(rendered, centerline, (drivable.shape[1], drivable.shape[0]))
        else:
            if self._turn_points is not None:
                draw_turn(rendered, self._turn_points, self._turn_preview.projection,
                          (drivable.shape[1], drivable.shape[0]))
            self._draw_text(rendered, 'TURN PREVIEW ONLY', 10, 12, (255, 80, 220))
        invalid_labels = 0
        for detection, measurement in zip(detections, measurements, strict=True):
            self._draw_box(rendered, detection)
            box_area = max(detection.xmax - detection.xmin, 0.0) * max(
                detection.ymax - detection.ymin,
                0.0,
            )
            if measurement.valid or (box_area >= 0.025 and invalid_labels < 4):
                self._draw_distance(rendered, detection, measurement)
                if not measurement.valid:
                    invalid_labels += 1

        result = dai.ImgFrame()
        result.setCvFrame(rendered, dai.ImgFrame.Type.NV12)
        result.setTimestamp(preview.getTimestamp())
        result.setSequenceNum(preview.getSequenceNum())
        return result

    def process(
        self,
        preview: dai.ImgFrame,
        nn_data: dai.NNData,
        aligned_depth: dai.ImgFrame,
    ) -> None:
        started_at = time.monotonic()
        depth_frame = aligned_depth.getFrame()
        depth_transformation = aligned_depth.getTransformation()
        if depth_transformation is None:
            intrinsics = np.zeros((3, 3), dtype=np.float64)
        else:
            intrinsics = np.asarray(
                depth_transformation.getIntrinsicMatrix(), dtype=np.float64
            )
        heads = [self._tensor(nn_data, name) for name in self.TENSOR_NAMES[:3]]
        prediction = decode_heads(heads)
        decoded = non_max_suppression(
            prediction,
            confidence_threshold=self._confidence_threshold,
            iou_threshold=self._iou_threshold,
        )

        detection_messages = []
        spatial_messages = []
        measurements = []
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
            measurement = estimate_spatial_from_depth(
                depth_frame,
                normalized,
                intrinsics,
            )
            measurements.append(measurement)
            if measurement.valid:
                spatial = dai.SpatialImgDetection()
                spatial.label = detection.label
                spatial.confidence = detection.confidence
                spatial.xmin = detection.xmin
                spatial.ymin = detection.ymin
                spatial.xmax = detection.xmax
                spatial.ymax = detection.ymax
                spatial.spatialCoordinates = dai.Point3f(
                    measurement.x_mm,
                    measurement.y_mm,
                    measurement.z_mm,
                )
                spatial_messages.append(spatial)

        detections = dai.ImgDetections()
        detections.detections = detection_messages
        detections.setTimestamp(nn_data.getTimestamp())
        detections.setSequenceNum(nn_data.getSequenceNum())
        self.detections.send(detections)

        spatial_detections = dai.SpatialImgDetections()
        spatial_detections.detections = spatial_messages
        spatial_detections.unit = dai.LengthUnit.MILLIMETER
        spatial_detections.setTimestamp(nn_data.getTimestamp())
        spatial_detections.setSequenceNum(nn_data.getSequenceNum())
        if depth_transformation is not None:
            spatial_detections.setTransformation(depth_transformation)
        self.spatial_detections.send(spatial_detections)

        drivable, lane = segmentation_masks(
            self._tensor(nn_data, self.TENSOR_NAMES[3]),
            self._tensor(nn_data, self.TENSOR_NAMES[4]),
            self._source_size,
        )
        measured_centerline = estimate_centerline(drivable)
        corridor_centerline = self._corridor_tracker.update(
            measured_centerline, timestamp_s=nn_data.getTimestamp().total_seconds()
        )
        ordered_centerline = self._order_selector.update(
            corridor_centerline, timestamp_s=nn_data.getTimestamp().total_seconds())
        centerline = self._centerline_filter.update(
            ordered_centerline, timestamp_s=nn_data.getTimestamp().total_seconds())
        centerline = self._shape_limiter.update(
            centerline, timestamp_s=nn_data.getTimestamp().total_seconds())
        self._limit_samples.append(self._shape_limiter.metrics.copy())
        bev = render_bev(centerline)
        self._turn_mailbox.poll(self._turn_preview, centerline)
        self._turn_points = self._turn_preview.update(centerline, nn_data.getTimestamp().total_seconds())
        if self._turn_preview.mode != 'road':
            bev[:] = (14, 22, 30)
            if self._turn_points is not None:
                draw_turn(bev, self._turn_points)
            self._draw_text(bev, 'TURN PREVIEW ONLY', 10, 10, (255, 80, 220))
            self._draw_text(bev, self._turn_preview.mode.upper(), 10, 32, (255, 80, 220))
            self._draw_text(bev, self._turn_preview.status.upper().replace('_', ' '), 10, 54, (255, 80, 220))
        gap_px = None
        if centerline.valid and measured_centerline.valid:
            raw_points = measured_centerline.image_curve
            filtered_points = centerline.image_curve
            raw_points = raw_points[np.argsort(raw_points[:, 1])]
            filtered_points = filtered_points[np.argsort(filtered_points[:, 1])]
            lo = max(0., raw_points[0, 1], filtered_points[0, 1])
            hi = min(drivable.shape[0] - 1., raw_points[-1, 1], filtered_points[-1, 1])
            if hi - lo >= 5:
                rows = np.linspace(lo, hi, 40)
                # Result overlay is three times the segmentation-mask resolution.
                gap_px = float(np.mean(np.abs(np.interp(rows, raw_points[:, 1], raw_points[:, 0]) -
                                               np.interp(rows, filtered_points[:, 1], filtered_points[:, 0]))) * 3)
        self._centerline_samples.append((centerline.valid, corridor_centerline.corridor_state, gap_px,
                                        centerline.model_degree, ordered_centerline.target_degree))
        self.drivable.send(self._mask_frame(drivable, nn_data))
        self.lane.send(self._mask_frame(lane, nn_data))
        self.bev.send(self._bev_frame(bev, nn_data))
        self.result.send(
            self._result_frame(
                preview,
                drivable,
                lane,
                detection_messages,
                measurements,
                centerline,
                measured_centerline,
            )
        )

        self._frames += 1
        now = time.monotonic()
        if now - self._last_health_at >= 5.0:
            elapsed = max(now - self._started_at, 1e-6)
            payload = {
                "turn_preview_mode": self._turn_preview.mode,
                "turn_preview_status": self._turn_preview.status,
                "turn_preview_control_eligible": False,
                "centerline_limit_frames": sum(s['limited'] for s in self._limit_samples),
                "centerline_constraint_conflicts": sum(s['conflict'] for s in self._limit_samples),
                "centerline_rate_bounds_max": {
                    key: np.max([s[key] for s in self._limit_samples if s['compared']], axis=0).tolist()
                    if any(s['compared'] for s in self._limit_samples) else None
                    for key in ('input_rate_bounds', 'output_rate_bounds')},
                "centerline_degree": centerline.model_degree,
                "centerline_target_degree": ordered_centerline.target_degree,
                "centerline_order_transition": centerline.order_transition,
                "centerline_order_fit_error": round(ordered_centerline.order_fit_error, 4),
                "centerline_degree_counts": {str(d): sum(s[0] and s[3] == d for s in self._centerline_samples)
                                             for d in (1, 2, 3)},
                "centerline_window_frames": len(self._centerline_samples),
                "centerline_window_valid": sum(int(s[0]) for s in self._centerline_samples),
                "centerline_window_guard": sum(s[1] == "expansion_guard" for s in self._centerline_samples),
                "centerline_raw_gap_mean_px": (
                    round(float(np.mean([s[2] for s in self._centerline_samples if s[2] is not None])), 2)
                    if any(s[2] is not None for s in self._centerline_samples) else None),
                "centerline_raw_gap_max_frame_mean_px": (
                    round(max(s[2] for s in self._centerline_samples if s[2] is not None), 2)
                    if any(s[2] is not None for s in self._centerline_samples) else None),
                "status": "running",
                "frames": self._frames,
                "average_fps": round(self._frames / elapsed, 2),
                "postprocess_ms": round((now - started_at) * 1000.0, 2),
                "detections": len(detection_messages),
                "depth_valid_detections": len(spatial_messages),
                "centerline_valid": centerline.valid,
                "centerline_corridor_state": corridor_centerline.corridor_state,
                "centerline_tracking_state": centerline.tracking_state,
                "centerline_measurement_age_s": round(centerline.measurement_age_s, 3),
                "centerline_motion_compensated": centerline.motion_compensated,
                "centerline_coefficients": (
                    [round(value, 6) for value in centerline.coefficients]
                    if centerline.valid
                    else None
                ),
                "centerline_residual_m": (
                    round(centerline.residual_m, 3) if centerline.valid else None
                ),
                "input_shape": [1, 3, 320, 320],
            }
            print(f"HEALTH {json.dumps(payload)}", flush=True)
            self._centerline_samples.clear()
            self._limit_samples.clear()
            self._last_health_at = now
