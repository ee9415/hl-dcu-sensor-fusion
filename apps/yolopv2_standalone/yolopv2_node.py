import json
import time
from dataclasses import replace

import depthai as dai
import numpy as np
from telemetry_server import TelemetryServer
from stop_line import StopLineTracker, StopLineEstimate, draw_stop_line
from image_boundaries import ImageBoundaryTracker, ImageBoundaryEstimate
from virtual_guidance import GuidanceConfig, VirtualGuidance, GuidanceEstimate, draw_guidance
from lane_overlay import (crop_lane_scores,render_lane_overlay,lane_score_colors,RAINBOW_EDGES,
                          reference_band_colors,BAND_LEVELS,BAND_COLORS,BAND_MODES,effective_preview_threshold,
                          TopViewRenderer, overlay_pixel_confidence)
from lane_boundaries import (BoundaryConfig, BoundaryEstimate,
                             draw_boundaries, render_boundary_bev, projection_for)

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
    "C": ("01111","10000","10000","10000","10000","10000","01111"),
    "D": ("11110","10001","10001","10001","10001","10001","11110"),
    "E": ("11111","10000","10000","11110","10000","10000","11111"),
    "F": ("11111","10000","10000","11110","10000","10000","10000"),
    "G": ("01110","10001","10000","10111","10001","10001","01110"),
    "H": ("10001","10001","10001","11111","10001","10001","10001"),
    "I": ("11111","00100","00100","00100","00100","00100","11111"),
    "K": ("10001","10010","10100","11000","10100","10010","10001"),
    "L": ("10000","10000","10000","10000","10000","10000","11111"),
    "M": ("10001","11011","10101","10101","10001","10001","10001"),
    "N": ("10001","11001","11001","10101","10011","10011","10001"),
    "O": ("01110","10001","10001","10001","10001","10001","01110"),
    "P": ("11110","10001","10001","11110","10000","10000","10000"),
    "R": ("11110","10001","10001","11110","10100","10010","10001"),
    "S": ("01111","10000","10000","01110","00001","00001","11110"),
    "T": ("11111","00100","00100","00100","00100","00100","00100"),
    "U": ("10001","10001","10001","10001","10001","10001","01110"),
    "V": ("10001","10001","10001","10001","10001","01010","00100"),
    "W": ("10001","10001","10001","10101","10101","10101","01010"),
    "X": ("10001","10001","01010","00100","01010","10001","10001"),
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
    "<": ("00010","00100","01000","10000","01000","00100","00010"),
    "Z": ("11111", "00001", "00010", "00100", "01000", "10000", "11111"),
    "m": ("00000", "00000", "11010", "10101", "10101", "10101", "10101"),
}
# (glyph, scale) -> read-only boolean pixel mask; bounded by the font size.
GLYPH_MASKS = {}


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
        self.model_size = (320, 320)
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
        if bgr.shape != (self.model_size[1], self.model_size[0], 3):
            raise ValueError(f"Expected BGR frame for {self.model_size}, got {bgr.shape}")

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
        self.model_size = (320, 320)
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
                    self.model_size,
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
        self.model_size = (320, 320)
        self._result_size = (960, 540)
        self._telemetry = TelemetryServer()
        self._confidence_threshold = 0.30
        self._iou_threshold = 0.45
        self._frames = 0
        self._started_at = time.monotonic()
        self._last_health_at = self._started_at
        self._boundary_tracker = ImageBoundaryTracker(BoundaryConfig.load())
        self._stop_line_tracker = StopLineTracker()
        self._guidance = VirtualGuidance(GuidanceConfig.load())
        self._guidance_estimate = GuidanceEstimate()
        self._boundary_visible = True
        self._contour_visible = False
        self._lane_overlay_mode = 'normal'
        self._lane_overlay_threshold = .5
        self._lane_scores = None
        self._lane_preview_error = None
        self.video_input = None
        self._input_epoch = None
        self._processing_key = None
        # Include the final camera row; reserve the canvas below it as unseen
        # ground. Never synthesize or stretch the last visible road pixels.
        self._top_view = TopViewRenderer(near_row=419, horizontal_scale=1.75)
        # Fit from 61 near-road rows per side, not a symmetric image trapezoid.
        # The left/right reference points belong to the same straight corridor.
        # Normalized road-plane anchors; not metric camera calibration.
        self._top_config = replace(self._boundary_tracker.config,
            near_left_u=.075724, near_right_u=.798382, near_v=1.,
            far_left_u=.382381, far_right_u=.567136, far_v=.62)

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
    def _glyph_mask(glyph: tuple[str, ...], scale: int) -> np.ndarray:
        key = (glyph, scale)
        mask = GLYPH_MASKS.get(key)
        if mask is None:
            # Each "1" becomes a scale x scale cell; cells tile without overlap.
            bits = np.array([[selected == "1" for selected in row] for row in glyph])
            mask = np.repeat(np.repeat(bits, scale, axis=0), scale, axis=1)
            mask.setflags(write=False)
            GLYPH_MASKS[key] = mask
        return mask

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
            mask = Yolopv2Postprocess._glyph_mask(glyph, scale)
            # Same pixels as painting each cell clipped to the frame.
            top, bottom = max(y, 0), min(y + mask.shape[0], height)
            left, right = cursor_x, min(cursor_x + mask.shape[1], width)
            if bottom > top and right > left:
                frame[top:bottom, left:right][
                    mask[top - y : bottom - y, left - cursor_x : right - cursor_x]
                ] = color
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

    def _draw_lane_band_legend(self,frame,threshold):
        left=frame.shape[1]-110;top=60;step=18
        frame[top-5:top+11*step+5,left:]=(8,8,8)
        active=int(np.searchsorted(BAND_LEVELS,np.float32(threshold),side='right'))
        for row,index in enumerate(range(10,-1,-1)):
            y=top+row*step
            if index==active:
                frame[y-1:y+16,left+5:left+27]=(255,255,255)
            frame[y:y+15,left+6:left+26]=BAND_COLORS[index]
            label=f'{BAND_LEVELS[index-1]:.2f}' if index else '<0.05'
            self._draw_text(frame,label,left+33,y+3,(255,255,255),scale=1)

    def _result_frame(
        self,
        preview: dai.ImgFrame,
        drivable: np.ndarray,
        lane: np.ndarray,
        detections: list[dai.ImgDetection],
        measurements: list[SpatialMeasurement],
        boundaries: BoundaryEstimate,
        stop_line,
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
        full = boundaries.state != 'disabled'
        if self._lane_overlay_mode != 'raw_rgb':
            from lane_overlay import overlay_lane_quadratics,overlay_contour_skeleton
            contours = getattr(self,'_lane_contour_result',None)
            if self._contour_visible and contours is not None:
                overlay_contour_skeleton(rendered,contours)
            if self._boundary_visible and contours is not None:
                if full:
                    from lane_overlay import overlay_tracked_skeleton
                    overlay_tracked_skeleton(rendered,boundaries,contours.directional_mask.shape)
                else:
                    overlay_lane_quadratics(rendered,self._lane_quadratics,contours.directional_mask.shape)
            self._draw_text(rendered,'MAGENTA SKELETON / YELLOW RETAINED',10,12,(0,220,255))
        invalid_labels = 0
        for detection, measurement in zip(detections, measurements, strict=True):
            self._draw_box(rendered, detection)
            box_area = max(detection.xmax - detection.xmin, 0.0) * max(
                detection.ymax - detection.ymin,
                0.0,
            )
            if full and (measurement.valid or (box_area >= 0.025 and invalid_labels < 4)):
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
        source = (self.video_input.metadata(nn_data.getSequenceNum())
                  if self.video_input is not None else {})
        if self.video_input is not None and not source:
            return  # Do not process a frame whose source provenance expired.
        processing_mode=source.get('processing_mode','full')
        self._boundary_visible=source.get('boundary_visible',True)
        self._contour_visible=source.get('contour_visible',False)
        self._lane_overlay_mode=source.get('lane_overlay_mode','normal')
        self._lane_overlay_threshold=source.get('lane_overlay_threshold',.5)
        processing_key=(processing_mode,source.get('processing_epoch',0),source.get('source_media_epoch'))
        if (source and source.get('source_epoch') != self._input_epoch) or processing_key!=self._processing_key:
            self._boundary_tracker = ImageBoundaryTracker(self._boundary_tracker.config)
            self._stop_line_tracker = StopLineTracker()
            self._guidance.reset()
            self._guidance_estimate = GuidanceEstimate()
            self._input_epoch = source.get('source_epoch')
            self._processing_key = processing_key
        depth_frame = aligned_depth.getFrame()
        depth_transformation = aligned_depth.getTransformation()
        if source.get('input_source') == 'pc':
            # Never assign live stereo distances to unrelated PC footage.
            depth_frame = np.zeros_like(depth_frame)
            depth_transformation = None
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
                np.asarray((x1, y1, x2, y2)), self._source_size, self.model_size
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
            self.model_size,
        )
        boundary_started = time.monotonic()
        from lane_contours import extract_lane_contours
        # Perception is independent of display switches. Full mode uses one
        # contour-fed tracker for RGB, TOP, guidance and telemetry.
        self._lane_contour_result = extract_lane_contours(lane)
        self._lane_quadratics = None
        self._lane_scores=None
        self._lane_preview_error=None
        if self._lane_overlay_mode != 'raw_rgb':
            try:
                self._lane_scores=crop_lane_scores(self._tensor(nn_data,self.TENSOR_NAMES[4]),lane.shape,self.model_size)
            except ValueError as exc:
                self._lane_preview_error=str(exc)
        if processing_mode=='full':
            boundaries = self._boundary_tracker.update_contours(
                self._lane_contour_result, lane.shape, nn_data.getTimestamp().total_seconds(),
                rgb=preview.getCvFrame())
            from ego_lane_quadratic import tracked_quadratics
            self._lane_quadratics = tracked_quadratics(boundaries, lane.shape)
            boundary_finished = time.monotonic()
            stop_line=self._stop_line_tracker.update(preview.getCvFrame(),boundaries,
                             nn_data.getTimestamp().total_seconds(),detection_messages)
            stop_finished=time.monotonic()
            self._guidance_estimate = self._guidance.update(
                drivable, boundaries, nn_data.getTimestamp().total_seconds())
        else:
            empty=np.empty((0,2),float)
            boundaries=ImageBoundaryEstimate(empty,empty,np.eye(3),'disabled','disabled',0.,0.,0.,
                                             np.array([[1.,0.,0.],[0.,1.,0.]]),'disabled','none')
            stop_line=StopLineEstimate(state='disabled')
            self._guidance_estimate = GuidanceEstimate(reason='basic_mode')
            boundary_finished=stop_finished=bev_finished=boundary_started
            # Basic mode deliberately bypasses temporal perception/guidance.
            from ego_lane_quadratic import select_ego_quadratics, add_virtual_side, complete_partial_lanes
            self._lane_quadratics = complete_partial_lanes(
                add_virtual_side(select_ego_quadratics(self._lane_contour_result,lane.shape),
                                 self._lane_contour_result,lane.shape), self._lane_contour_result,lane.shape)
        # TOP video remains available in basic mode; no boundary tracking needed.
        top_projection = projection_for(lane.shape, self._top_config)
        bev = self._top_view.render(preview.getCvFrame(), lane, top_projection, 'raw_rgb')
        if self._contour_visible and self._lane_overlay_mode != 'raw_rgb':
            self._top_view.overlay_lane_groups(bev,self._lane_contour_result)
        if self._boundary_visible and self._lane_overlay_mode != 'raw_rgb':
            from lane_overlay import overlay_lane_quadratics
            if processing_mode == 'full':
                from lane_overlay import overlay_tracked_skeleton
                overlay_tracked_skeleton(bev,boundaries,lane.shape,
                                         self._top_view.projection,self._top_view.valid)
            else:
                overlay_lane_quadratics(bev,self._lane_quadratics,lane.shape,
                                        self._top_view.projection,self._top_view.valid)
        bev[~self._top_view.valid] = 0
        self._draw_text(bev,'TOP TRACKED SKELETON',10,10,(0,220,255))
        self._draw_text(bev,'MAGENTA REAL / YELLOW VIRTUAL',10,30,(0,220,255))
        bev_finished = time.monotonic()
        capture_s = preview.getTimestamp().total_seconds()
        capture_wall = time.time() - (dai.Clock.now().total_seconds() - capture_s)
        capture_mono = time.monotonic() - (dai.Clock.now().total_seconds() - capture_s)
        self._telemetry.publish({
            **source,
            'processing_mode':processing_mode,
            'lane_perception_threshold':.5,
            'lane_preview_effective_threshold':effective_preview_threshold(self._lane_overlay_mode,self._lane_overlay_threshold),
            'lane_perception_pixels':int(np.count_nonzero(lane)),
            'lane_overlay_error':self._lane_preview_error,
            'lane_preview_selected_pixels':int(np.count_nonzero(self._lane_scores>=effective_preview_threshold(self._lane_overlay_mode,self._lane_overlay_threshold))) if self._lane_scores is not None else None,
            'lane_preview_score_min':float(self._lane_scores.min()) if self._lane_scores is not None else None,
            'lane_preview_score_max':float(self._lane_scores.max()) if self._lane_scores is not None else None,
            'depth_available': source.get('input_source', 'camera') == 'camera',
            'frame_seq': int(preview.getSequenceNum()),
            'nn_seq': int(nn_data.getSequenceNum()),
            'depth_seq': int(aligned_depth.getSequenceNum()),
            'capture_timestamp_s': capture_s,
            'nn_timestamp_s': nn_data.getTimestamp().total_seconds(),
            'depth_timestamp_s': aligned_depth.getTimestamp().total_seconds(),
            'capture_wall_est_s': capture_wall,
            'capture_monotonic_est_s': capture_mono,
            # Legacy columns remain empty/disabled for older CSV consumers.
            'centerline_valid': False,
            'tracking_state': 'disabled',
            'coefficients': None,
            'turn_mode': 'disabled',
            **boundaries.telemetry(self._boundary_tracker.config),
            **stop_line.telemetry(),
            **self._guidance_estimate.telemetry(),
            'objects': [dict(label=d.label, confidence=d.confidence,
                             xmin=d.xmin, ymin=d.ymin, xmax=d.xmax, ymax=d.ymax,
                             depth_valid=bool(m.valid),
                             x_m=m.x_mm/1000 if m.valid else None,
                             y_m=m.y_mm/1000 if m.valid else None,
                             z_m=m.z_mm/1000 if m.valid else None)
                        for d, m in zip(detection_messages, measurements, strict=True)]})
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
                boundaries,
                stop_line,
            )
        )

        self._frames += 1
        now = time.monotonic()
        if now - self._last_health_at >= 5.0:
            elapsed = max(now - self._started_at, 1e-6)
            payload = {
                **boundaries.telemetry(self._boundary_tracker.config),
                **stop_line.telemetry(),
                'guidance_valid': self._guidance_estimate.valid,
                'boundary_visible': self._boundary_visible,
                'guidance_boundaries_available': self._guidance_estimate.boundaries_available,
                'guidance_path_display_available': self._guidance_estimate.visible_path is not None,
                'guidance_boundary_state': self._guidance_estimate.boundary_state,
                'guidance_source': self._guidance_estimate.source,
                'guidance_reason': self._guidance_estimate.reason,
                'guidance_enabled': self._guidance.config.enabled,
                "status": "running",
                "processing_mode":processing_mode,
                "frames": self._frames,
                "average_fps": round(self._frames / elapsed, 2),
                "postprocess_ms": round((now - started_at) * 1000.0, 2),
                "boundary_ms": round((boundary_finished-boundary_started)*1000,2),
                "stop_line_ms": round((stop_finished-boundary_finished)*1000,2),
                "bev_draw_ms": round((bev_finished-boundary_finished)*1000,2),
                "boundary_candidates": self._boundary_tracker.last_candidate_count,
                "detections": len(detection_messages),
                "depth_valid_detections": len(spatial_messages),
                "input_shape": [1, 3, self.model_size[1], self.model_size[0]],
            }
            print(f"HEALTH {json.dumps(payload)}", flush=True)
            self._last_health_at = now
