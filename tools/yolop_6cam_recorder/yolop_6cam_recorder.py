#!/usr/bin/env python3
"""Six OAK-D PoE YOLOP inference and H.265 recorder for DepthAI 2.32.

The program intentionally uses one DepthAI pipeline per camera and one host worker
thread per device.  A worker failure is reported to the main thread and stops the
whole session; it is never silently ignored.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import queue
import shutil
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import depthai as dai
import numpy as np


DCU_IP = "192.168.201.4"
INPUT_WIDTH = 320
INPUT_HEIGHT = 320
STATUS_INTERVAL_SEC = 5.0
CAMERA_STALL_TIMEOUT_SEC = 15.0
MIN_FREE_BYTES_AT_START = 1024 * 1024 * 1024
MIN_FREE_BYTES_DURING_RECORDING = 256 * 1024 * 1024

CAMERAS: Mapping[str, str] = {
    "cam_01": "1944301001E1761300",
    "cam_02": "19443010E131771300",
    "cam_03": "194430105130731300",
    "cam_04": "19443010517C731300",
    "cam_05": "194430105111771300",
    "cam_06": "1944301071DA761300",
}

# Leave names unset unless these exact names are known for the blob.  CLI options
# override this dictionary.  Shapes are comma-separated NCHW/NHWC dimensions.
DEFAULT_TENSOR_CONFIG: Mapping[str, Optional[str]] = {
    "det_layer": None,
    "drivable_layer": None,
    "lane_layer": None,
    "det_shape": None,
    "drivable_shape": None,
    "lane_shape": None,
}

DEFAULT_CLASS_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic_light", "fire_hydrant", "stop_sign",
]


class RecorderError(RuntimeError):
    """Expected fatal error with a user-facing message."""


@dataclass
class Counters:
    encoded_frames: int = 0
    inference_frames: int = 0
    json_records: int = 0
    started_monotonic: float = 0.0
    last_frame_monotonic: float = 0.0


@dataclass
class TensorSpec:
    name: str
    shape: Tuple[int, ...]
    data_type: str


def iso_utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_shape(value: Optional[str]) -> Optional[Tuple[int, ...]]:
    if not value:
        return None
    try:
        shape = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"shape must contain integers: {value}") from exc
    if not shape or any(dim <= 0 for dim in shape):
        raise argparse.ArgumentTypeError(f"shape dimensions must be positive: {value}")
    return shape


def parse_class_names(value: str) -> List[str]:
    candidate = Path(value).expanduser()
    if candidate.is_file():
        try:
            loaded = json.loads(candidate.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RecorderError(f"Cannot read class-name JSON '{candidate}': {exc}") from exc
        if not isinstance(loaded, list) or not all(isinstance(v, str) for v in loaded):
            raise RecorderError("--class-names JSON must be an array of strings")
        return loaded
    names = [item.strip() for item in value.split(",") if item.strip()]
    if not names:
        raise RecorderError("--class-names is empty")
    return names


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run YOLOP and record H.265/JSONL from six OAK-D PoE cameras."
    )
    parser.add_argument("--blob", help="YOLOP .blob file (required unless --record-only)")
    parser.add_argument(
        "--record-only", action="store_true",
        help="record 320x320 H.265 without loading a Blob or running inference",
    )
    parser.add_argument("--output", default="recordings", help="parent output directory")
    parser.add_argument("--fps", type=float, default=10.0, help="camera/encoder FPS (default: 10)")
    parser.add_argument("--bitrate-kbps", type=int, default=2000, help="H.265 bitrate per camera")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 records until Ctrl+C")
    parser.add_argument("--inspect-nn", action="store_true", help="print actual NN tensor metadata and exit")
    parser.add_argument("--inspect-camera", choices=tuple(CAMERAS), default="cam_01")
    parser.add_argument("--det-layer", default=DEFAULT_TENSOR_CONFIG["det_layer"])
    parser.add_argument("--drivable-layer", default=DEFAULT_TENSOR_CONFIG["drivable_layer"])
    parser.add_argument("--lane-layer", default=DEFAULT_TENSOR_CONFIG["lane_layer"])
    parser.add_argument("--det-shape", type=parse_shape, default=parse_shape(DEFAULT_TENSOR_CONFIG["det_shape"]))
    parser.add_argument("--drivable-shape", type=parse_shape, default=parse_shape(DEFAULT_TENSOR_CONFIG["drivable_shape"]))
    parser.add_argument("--lane-shape", type=parse_shape, default=parse_shape(DEFAULT_TENSOR_CONFIG["lane_shape"]))
    parser.add_argument(
        "--det-format", choices=("auto", "decoded-xyxy", "yolo-xywh"), default="auto",
        help="decoded Nx6 or YOLO Nx(5+classes) detection layout",
    )
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument(
        "--class-names", default=",".join(DEFAULT_CLASS_NAMES),
        help="comma-separated names or path to a JSON string array",
    )
    return parser


def validate_args(args: argparse.Namespace) -> Optional[Path]:
    if args.record_only and args.inspect_nn:
        raise RecorderError("--record-only and --inspect-nn cannot be used together")
    blob: Optional[Path] = None
    if not args.record_only:
        if not args.blob:
            raise RecorderError("--blob is required unless --record-only is specified")
        blob = Path(args.blob).expanduser().resolve()
        if not blob.is_file():
            raise RecorderError(f"Blob load failure: file does not exist: {blob}")
        if blob.suffix.lower() != ".blob":
            raise RecorderError(f"Blob load failure: expected a .blob file: {blob}")
    if not (0 < args.fps <= 60):
        raise RecorderError("--fps must be greater than 0 and no more than 60")
    if args.bitrate_kbps <= 0:
        raise RecorderError("--bitrate-kbps must be greater than 0")
    if args.duration < 0:
        raise RecorderError("--duration cannot be negative")
    if not (0.0 <= args.confidence <= 1.0 and 0.0 <= args.iou <= 1.0):
        raise RecorderError("--confidence and --iou must be between 0 and 1")
    return blob


def discover_cameras() -> Tuple[Dict[str, dai.DeviceInfo], List[Tuple[str, str]]]:
    try:
        available = list(dai.Device.getAllAvailableDevices())
    except Exception as exc:
        raise RecorderError(f"DepthAI device discovery failed: {exc}") from exc

    by_mxid: Dict[str, dai.DeviceInfo] = {}
    for info in available:
        mxid = info.getMxId()
        if mxid:
            by_mxid[mxid] = info

    mapped: Dict[str, dai.DeviceInfo] = {}
    missing: List[Tuple[str, str]] = []
    for camera_name, expected_mxid in CAMERAS.items():
        info = by_mxid.get(expected_mxid)
        if info is None:
            missing.append((camera_name, expected_mxid))
        else:
            mapped[camera_name] = info
    return mapped, missing


def print_discovery(mapped: Mapping[str, dai.DeviceInfo], missing: Sequence[Tuple[str, str]]) -> None:
    print("DepthAI camera discovery (MX ID mapping):")
    for camera_name, mxid in CAMERAS.items():
        state = "FOUND" if camera_name in mapped else "MISSING"
        print(f"  {camera_name}: {mxid} [{state}]")
    if missing:
        print("Missing required cameras:", file=sys.stderr)
        for camera_name, mxid in missing:
            print(f"  - {camera_name}: MX ID {mxid}", file=sys.stderr)


def create_manip(pipeline: dai.Pipeline, frame_type: Any) -> Any:
    manip = pipeline.create(dai.node.ImageManip)
    # setResizeThumbnail performs a centered letterbox and preserves aspect ratio.
    manip.initialConfig.setResizeThumbnail(INPUT_WIDTH, INPUT_HEIGHT)
    manip.initialConfig.setFrameType(frame_type)
    manip.inputImage.setBlocking(False)
    manip.inputImage.setQueueSize(4)
    return manip


def build_pipeline(
    blob: Optional[Path],
    fps: float,
    bitrate_kbps: int,
    include_encoder: bool = True,
    include_nn: bool = True,
) -> dai.Pipeline:
    try:
        pipeline = dai.Pipeline()
        camera = pipeline.create(dai.node.ColorCamera)
        camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camera.setFps(fps)
        camera.setInterleaved(False)
        camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        if include_nn:
            if blob is None:
                raise RecorderError("Internal error: NN pipeline requested without a Blob")
            nn_manip = create_manip(pipeline, dai.RawImgFrame.Type.BGR888p)
            camera.video.link(nn_manip.inputImage)

            nn = pipeline.create(dai.node.NeuralNetwork)
            nn.setBlobPath(str(blob))
            nn.setNumInferenceThreads(2)
            nn.input.setBlocking(False)
            nn.input.setQueueSize(2)
            nn_manip.out.link(nn.input)

            nn_out = pipeline.create(dai.node.XLinkOut)
            nn_out.setStreamName("nn")
            nn.out.link(nn_out.input)

        if include_encoder:
            encoder_manip = create_manip(pipeline, dai.RawImgFrame.Type.NV12)
            camera.video.link(encoder_manip.inputImage)

            encoder = pipeline.create(dai.node.VideoEncoder)
            encoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
            encoder.setBitrateKbps(bitrate_kbps)
            encoder_manip.out.link(encoder.input)

            encoded_out = pipeline.create(dai.node.XLinkOut)
            encoded_out.setStreamName("h265")
            encoder.bitstream.link(encoded_out.input)
        return pipeline
    except Exception as exc:
        context = f" with Blob '{blob}'" if include_nn else " in record-only mode"
        raise RecorderError(f"Pipeline creation failed{context}: {exc}") from exc


def tensor_metadata(packet: Any) -> List[TensorSpec]:
    specs: List[TensorSpec] = []
    try:
        tensors = packet.getRaw().tensors
    except Exception:
        tensors = []
    for tensor in tensors:
        dims = tuple(int(value) for value in tensor.dims)
        specs.append(TensorSpec(tensor.name, dims, str(tensor.dataType)))
    if not specs:
        for name in packet.getAllLayerNames():
            specs.append(TensorSpec(name, tuple(), "unknown"))
    return specs


def read_tensor(packet: Any, spec: TensorSpec, shape_override: Optional[Tuple[int, ...]]) -> np.ndarray:
    errors: List[str] = []
    data: Optional[np.ndarray] = None
    for method_name, dtype in (("getLayerFp16", np.float32), ("getLayerInt32", np.int32), ("getLayerUInt8", np.uint8)):
        method = getattr(packet, method_name, None)
        if method is None:
            continue
        try:
            values = method(spec.name)
            if len(values) > 0:
                data = np.asarray(values, dtype=dtype)
                break
        except Exception as exc:
            errors.append(f"{method_name}: {exc}")
    if data is None:
        detail = "; ".join(errors) if errors else "no supported layer accessor"
        raise RecorderError(f"Cannot read NN tensor '{spec.name}' ({detail})")

    shape = shape_override or spec.shape
    if shape:
        expected = math.prod(shape)
        if data.size != expected:
            raise RecorderError(
                f"Output tensor mismatch for '{spec.name}': metadata/config shape {shape} "
                f"requires {expected} values, received {data.size}. Run --inspect-nn and set --*-shape."
            )
        data = data.reshape(shape)
    return data


def choose_layer(
    role: str,
    configured_name: Optional[str],
    specs: Sequence[TensorSpec],
    used: Iterable[str],
) -> TensorSpec:
    by_name = {spec.name: spec for spec in specs}
    if configured_name:
        if configured_name not in by_name:
            raise RecorderError(
                f"Configured {role} tensor '{configured_name}' is absent. Available: {sorted(by_name)}. "
                "Run --inspect-nn and correct the layer option."
            )
        return by_name[configured_name]

    keywords = {
        "detection": ("det", "detect", "box", "object"),
        "drivable": ("drive", "drivable", "da_seg", "area"),
        "lane": ("lane", "ll_seg"),
    }[role]
    unused = [spec for spec in specs if spec.name not in set(used)]
    matches = [spec for spec in unused if any(word in spec.name.lower() for word in keywords)]
    if len(matches) == 1:
        return matches[0]

    if role == "detection":
        shape_matches = [
            spec for spec in unused
            if spec.shape and len(spec.shape) >= 2 and spec.shape[-1] >= 6
        ]
        if len(shape_matches) == 1:
            return shape_matches[0]

    raise RecorderError(
        f"Cannot uniquely identify {role} tensor. Available tensors: "
        f"{[(s.name, s.shape) for s in specs]}. Run --inspect-nn and pass the explicit --{role if role != 'detection' else 'det'}-layer option."
    )


def sigmoid(values: np.ndarray) -> np.ndarray:
    clipped = np.clip(values, -40.0, 40.0)
    return 1.0 / (1.0 + np.exp(-clipped))


def iou_xyxy(box: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_a = max(0.0, float(box[2] - box[0])) * max(0.0, float(box[3] - box[1]))
    area_b = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0.0, boxes[:, 3] - boxes[:, 1])
    return intersection / np.maximum(area_a + area_b - intersection, 1e-9)


def class_aware_nms(rows: List[Dict[str, Any]], threshold: float) -> List[Dict[str, Any]]:
    kept: List[Dict[str, Any]] = []
    for class_id in sorted({int(row["class_id"]) for row in rows}):
        candidates = [row for row in rows if int(row["class_id"]) == class_id]
        candidates.sort(key=lambda row: float(row["confidence"]), reverse=True)
        while candidates:
            best = candidates.pop(0)
            kept.append(best)
            if not candidates:
                break
            other_boxes = np.asarray([row["bbox_xyxy_normalized"] for row in candidates], dtype=np.float32)
            overlaps = iou_xyxy(np.asarray(best["bbox_xyxy_normalized"], dtype=np.float32), other_boxes)
            candidates = [row for row, overlap in zip(candidates, overlaps) if overlap <= threshold]
    return sorted(kept, key=lambda row: float(row["confidence"]), reverse=True)


def normalize_box(box: Sequence[float], xywh: bool) -> List[float]:
    values = np.asarray(box, dtype=np.float32)
    if xywh:
        cx, cy, width, height = values
        values = np.asarray([cx - width / 2, cy - height / 2, cx + width / 2, cy + height / 2])
    if float(np.max(np.abs(values))) > 2.0:
        values[[0, 2]] /= INPUT_WIDTH
        values[[1, 3]] /= INPUT_HEIGHT
    return np.clip(values, 0.0, 1.0).round(6).tolist()


def decode_detections(
    tensor: np.ndarray,
    det_format: str,
    confidence_threshold: float,
    iou_threshold: float,
    class_names: Sequence[str],
) -> List[Dict[str, Any]]:
    if tensor.ndim == 1:
        if tensor.size % 6 != 0:
            raise RecorderError("Detection tensor is flat and its length is not divisible by 6; configure --det-shape")
        rows = tensor.reshape(-1, 6)
    else:
        width = tensor.shape[-1]
        if width < 6:
            raise RecorderError(f"Detection tensor last dimension must be >= 6, got {tensor.shape}")
        rows = tensor.reshape(-1, width)

    selected_format = det_format
    if selected_format == "auto":
        # Nx6 is conventionally [x1,y1,x2,y2,score,class]. Wider tensors are
        # conventionally [cx,cy,w,h,objectness,class probabilities...].
        selected_format = "decoded-xyxy" if rows.shape[1] == 6 else "yolo-xywh"

    results: List[Dict[str, Any]] = []
    if selected_format == "decoded-xyxy":
        if rows.shape[1] != 6:
            raise RecorderError(f"decoded-xyxy requires Nx6, received {rows.shape}")
        for row in rows:
            score = float(row[4])
            if score < confidence_threshold:
                continue
            class_id = int(round(float(row[5])))
            label = class_names[class_id] if 0 <= class_id < len(class_names) else str(class_id)
            results.append({
                "class_id": class_id,
                "label": label,
                "confidence": round(score, 6),
                "bbox_xyxy_normalized": normalize_box(row[:4], xywh=False),
            })
    else:
        for row in rows:
            obj = float(row[4])
            class_scores = np.asarray(row[5:], dtype=np.float32)
            if obj < 0.0 or obj > 1.0 or np.any(class_scores < 0.0) or np.any(class_scores > 1.0):
                obj = float(sigmoid(np.asarray([obj]))[0])
                class_scores = sigmoid(class_scores)
            class_id = int(np.argmax(class_scores))
            score = obj * float(class_scores[class_id])
            if score < confidence_threshold:
                continue
            label = class_names[class_id] if class_id < len(class_names) else str(class_id)
            results.append({
                "class_id": class_id,
                "label": label,
                "confidence": round(score, 6),
                "bbox_xyxy_normalized": normalize_box(row[:4], xywh=True),
            })
    return class_aware_nms(results, iou_threshold)


def segmentation_summary(tensor: np.ndarray) -> Dict[str, Any]:
    values = np.squeeze(tensor)
    if values.ndim == 2:
        if np.any(values < 0.0) or np.any(values > 1.0):
            values = sigmoid(values)
        mask = values >= 0.5
        height, width = mask.shape
        return {
            "shape": list(tensor.shape), "mask_width": int(width), "mask_height": int(height),
            "foreground_pixels": int(np.count_nonzero(mask)),
            "foreground_ratio": round(float(np.mean(mask)), 6), "threshold": 0.5,
        }
    if values.ndim != 3:
        raise RecorderError(f"Segmentation tensor must reduce to 2D or 3D, received {tensor.shape}")

    # Accept CHW and HWC. Smallest dimension is normally the class dimension.
    if values.shape[0] <= 16:
        mask = np.argmax(values, axis=0)
    elif values.shape[-1] <= 16:
        mask = np.argmax(values, axis=-1)
    else:
        raise RecorderError(f"Cannot infer class axis for segmentation tensor {tensor.shape}; configure its shape")
    counts = np.bincount(mask.reshape(-1).astype(np.int64))
    total = int(mask.size)
    return {
        "shape": list(tensor.shape), "mask_width": int(mask.shape[1]), "mask_height": int(mask.shape[0]),
        "class_pixel_counts": {str(index): int(count) for index, count in enumerate(counts)},
        "foreground_pixels": int(total - (counts[0] if counts.size else 0)),
        "foreground_ratio": round(float(np.mean(mask != 0)), 6),
    }


class PostProcessor:
    def __init__(self, args: argparse.Namespace, class_names: Sequence[str]) -> None:
        self.args = args
        self.class_names = class_names
        self.spec_by_role: Optional[Dict[str, TensorSpec]] = None

    def _resolve(self, packet: Any) -> Dict[str, TensorSpec]:
        specs = tensor_metadata(packet)
        if not specs:
            raise RecorderError("NN output has no tensors")
        det = choose_layer("detection", self.args.det_layer, specs, ())
        drivable = choose_layer("drivable", self.args.drivable_layer, specs, (det.name,))
        lane = choose_layer("lane", self.args.lane_layer, specs, (det.name, drivable.name))
        if len({det.name, drivable.name, lane.name}) != 3:
            raise RecorderError("Detection, drivable and lane tensor names must be different")
        self.spec_by_role = {"detection": det, "drivable": drivable, "lane": lane}
        print(
            "Resolved NN tensors: "
            f"detection={det.name}{det.shape}, drivable={drivable.name}{drivable.shape}, lane={lane.name}{lane.shape}",
            flush=True,
        )
        return self.spec_by_role

    def process(self, packet: Any) -> Tuple[List[Dict[str, Any]], Dict[str, Any], Dict[str, Any]]:
        specs = self.spec_by_role or self._resolve(packet)
        det = read_tensor(packet, specs["detection"], self.args.det_shape)
        drivable = read_tensor(packet, specs["drivable"], self.args.drivable_shape)
        lane = read_tensor(packet, specs["lane"], self.args.lane_shape)
        return (
            decode_detections(det, self.args.det_format, self.args.confidence, self.args.iou, self.class_names),
            segmentation_summary(drivable),
            segmentation_summary(lane),
        )


def device_time_seconds(packet: Any) -> Optional[float]:
    try:
        return round(float(packet.getTimestampDevice().total_seconds()), 9)
    except Exception:
        return None


class CameraWorker(threading.Thread):
    def __init__(
        self,
        camera_name: str,
        mxid: str,
        device: dai.Device,
        encoded_queue: Any,
        nn_queue: Optional[Any],
        session_dir: Path,
        args: argparse.Namespace,
        class_names: Sequence[str],
        stop_event: threading.Event,
        error_queue: "queue.Queue[Tuple[str, BaseException]]",
    ) -> None:
        super().__init__(name=f"worker-{camera_name}", daemon=False)
        self.camera_name = camera_name
        self.mxid = mxid
        self.device = device
        self.encoded_queue = encoded_queue
        self.nn_queue = nn_queue
        self.session_dir = session_dir
        self.args = args
        self.class_names = class_names
        self.stop_event = stop_event
        self.error_queue = error_queue
        self.counters = Counters(started_monotonic=time.monotonic())
        self._lock = threading.Lock()

    def snapshot(self) -> Counters:
        with self._lock:
            return Counters(**asdict(self.counters))

    def _check_space(self) -> None:
        free = shutil.disk_usage(self.session_dir).free
        if free < MIN_FREE_BYTES_DURING_RECORDING:
            raise RecorderError(
                f"Storage space low for {self.camera_name}: {free / (1024 ** 2):.1f} MiB free "
                f"(< {MIN_FREE_BYTES_DURING_RECORDING / (1024 ** 2):.0f} MiB)"
            )

    def run(self) -> None:
        h265_path = self.session_dir / f"{self.camera_name}.h265"
        jsonl_path = self.session_dir / f"{self.camera_name}.jsonl"
        postprocessor = None if self.nn_queue is None else PostProcessor(self.args, self.class_names)
        try:
            with h265_path.open("wb", buffering=1024 * 1024) as video_file, jsonl_path.open(
                "w", encoding="utf-8", buffering=1
            ) as jsonl_file:
                next_space_check = time.monotonic() + STATUS_INTERVAL_SEC
                while True:
                    did_work = False
                    while True:
                        packet = self.encoded_queue.tryGet()
                        if packet is None:
                            break
                        video_file.write(packet.getData().tobytes())
                        if self.args.record_only:
                            record = {
                                "host_time": iso_utc_now(),
                                "device_time_seconds": device_time_seconds(packet),
                                "camera_name": self.camera_name,
                                "mx_id": self.mxid,
                                "frame_number": int(packet.getSequenceNum()),
                                "objects": [],
                                "drivable_area": None,
                                "lane_area": None,
                                "mode": "record_only",
                            }
                            jsonl_file.write(
                                json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
                            )
                        with self._lock:
                            self.counters.encoded_frames += 1
                            if self.args.record_only:
                                self.counters.json_records += 1
                            self.counters.last_frame_monotonic = time.monotonic()
                        did_work = True

                    if self.nn_queue is not None and postprocessor is not None:
                        while True:
                            packet = self.nn_queue.tryGet()
                            if packet is None:
                                break
                            objects, drivable, lane = postprocessor.process(packet)
                            record = {
                                "host_time": iso_utc_now(),
                                "device_time_seconds": device_time_seconds(packet),
                                "camera_name": self.camera_name,
                                "mx_id": self.mxid,
                                "frame_number": int(packet.getSequenceNum()),
                                "objects": objects,
                                "drivable_area": drivable,
                                "lane_area": lane,
                                "mode": "yolop",
                            }
                            jsonl_file.write(
                                json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
                            )
                            with self._lock:
                                self.counters.inference_frames += 1
                                self.counters.json_records += 1
                                self.counters.last_frame_monotonic = time.monotonic()
                            did_work = True

                    now = time.monotonic()
                    if now >= next_space_check:
                        video_file.flush()
                        self._check_space()
                        next_space_check = now + STATUS_INTERVAL_SEC
                    # Drain packets that were already present once after a stop
                    # request, then leave the context manager to flush/close files.
                    if self.stop_event.is_set():
                        break
                    if not did_work:
                        time.sleep(0.002)
        except OSError as exc:
            if exc.errno == errno.ENOSPC:
                wrapped: BaseException = RecorderError(f"Storage space exhausted while writing {self.camera_name}: {exc}")
            else:
                wrapped = RecorderError(f"File write error for {self.camera_name}: {exc}")
            self.error_queue.put((self.camera_name, wrapped))
            self.stop_event.set()
        except BaseException as exc:
            self.error_queue.put((self.camera_name, exc))
            self.stop_event.set()


def create_session_dir(parent: Path) -> Path:
    try:
        parent.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(parent).free
        if free < MIN_FREE_BYTES_AT_START:
            raise RecorderError(
                f"Insufficient storage: {free / (1024 ** 3):.2f} GiB free in {parent}; at least 1 GiB required"
            )
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = parent / f"yolop_{stamp}"
        suffix = 1
        while session_dir.exists():
            session_dir = parent / f"yolop_{stamp}_{suffix:02d}"
            suffix += 1
        session_dir.mkdir()
        return session_dir
    except OSError as exc:
        raise RecorderError(f"Cannot create output directory '{parent}': {exc}") from exc


def inspect_nn(args: argparse.Namespace, blob: Path, mapped: Mapping[str, dai.DeviceInfo]) -> int:
    camera_name = args.inspect_camera
    info = mapped.get(camera_name)
    if info is None:
        raise RecorderError(f"Inspection camera {camera_name} ({CAMERAS[camera_name]}) is not connected")
    device: Optional[dai.Device] = None
    try:
        print(f"Inspecting {camera_name} ({CAMERAS[camera_name]}); no recording files will be created.")
        device = dai.Device(
            build_pipeline(blob, args.fps, args.bitrate_kbps, include_encoder=False, include_nn=True),
            info,
            dai.UsbSpeed.SUPER,
        )
        nn_queue = device.getOutputQueue("nn", maxSize=2, blocking=True)
        packet = nn_queue.get()
        report = {
            "camera_name": camera_name,
            "mx_id": device.getMxId(),
            "sequence_num": int(packet.getSequenceNum()),
            "tensors": [asdict(spec) for spec in tensor_metadata(packet)],
        }
        for item in report["tensors"]:
            item["shape"] = list(item["shape"])
        print(json.dumps(report, indent=2, ensure_ascii=False))
        print("Use the printed names with --det-layer, --drivable-layer and --lane-layer.")
        print("If a shape is empty/incorrect, supply --det-shape/--drivable-shape/--lane-shape.")
        return 0
    except Exception as exc:
        raise RecorderError(f"NN inspection failed on {camera_name}: {exc}") from exc
    finally:
        if device is not None:
            device.close()


def write_metadata(
    session_dir: Path,
    args: argparse.Namespace,
    blob: Optional[Path],
    devices: Mapping[str, dai.Device],
) -> None:
    metadata = {
        "created_at": iso_utc_now(),
        "dcu_ip": DCU_IP,
        "python_version": sys.version.split()[0],
        "depthai_version": getattr(dai, "__version__", "unknown"),
        "mode": "record_only" if args.record_only else "yolop",
        "blob": str(blob) if blob is not None else None,
        "input_size": [INPUT_WIDTH, INPUT_HEIGHT],
        "resize_mode": "centered_letterbox_setResizeThumbnail",
        "fps": args.fps,
        "bitrate_kbps_per_camera": args.bitrate_kbps,
        "duration_seconds": args.duration,
        "tensor_config": {
            "detection": {"name": args.det_layer, "shape": args.det_shape, "format": args.det_format},
            "drivable": {"name": args.drivable_layer, "shape": args.drivable_shape},
            "lane": {"name": args.lane_layer, "shape": args.lane_shape},
        },
        "cameras": [
            {"camera_name": name, "mx_id": CAMERAS[name], "device_name": devices[name].getDeviceName()}
            for name in CAMERAS
        ],
    }
    try:
        (session_dir / "session_metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    except OSError as exc:
        raise RecorderError(f"Cannot write session metadata: {exc}") from exc


def run_recording(
    args: argparse.Namespace,
    blob: Optional[Path],
    mapped: Mapping[str, dai.DeviceInfo],
) -> int:
    session_dir = create_session_dir(Path(args.output).expanduser().resolve())
    print(f"Session output: {session_dir}")
    class_names = [] if args.record_only else parse_class_names(args.class_names)
    stop_event = threading.Event()
    error_queue: "queue.Queue[Tuple[str, BaseException]]" = queue.Queue()
    devices: Dict[str, dai.Device] = {}
    output_queues: Dict[str, Tuple[Any, Optional[Any]]] = {}
    workers: List[CameraWorker] = []
    previous_handlers: Dict[int, Any] = {}

    def request_stop(signum: int, _frame: Any) -> None:
        print(f"\nSignal {signum} received; closing all cameras and files...", flush=True)
        stop_event.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_stop)

        # Open every camera before starting any writer. A partial connection never
        # becomes a recording session.
        for camera_name in CAMERAS:
            try:
                pipeline = build_pipeline(
                    blob,
                    args.fps,
                    args.bitrate_kbps,
                    include_encoder=True,
                    include_nn=not args.record_only,
                )
                device = dai.Device(pipeline, mapped[camera_name], dai.UsbSpeed.SUPER)
                actual_mxid = device.getMxId()
                if actual_mxid != CAMERAS[camera_name]:
                    device.close()
                    raise RecorderError(
                        f"MX ID changed while connecting {camera_name}: expected {CAMERAS[camera_name]}, got {actual_mxid}"
                    )
                devices[camera_name] = device
                # Register non-blocking host queues immediately so the first
                # pipeline cannot back-pressure while the remaining devices open.
                encoded_queue = device.getOutputQueue(
                    "h265", maxSize=max(60, int(args.fps * 10)), blocking=False
                )
                nn_queue = None
                if not args.record_only:
                    nn_queue = device.getOutputQueue(
                        "nn", maxSize=max(30, int(args.fps * 5)), blocking=False
                    )
                output_queues[camera_name] = (encoded_queue, nn_queue)
                print(f"Connected {camera_name}: {actual_mxid} ({device.getDeviceName()})")
                if stop_event.is_set():
                    raise RecorderError("Stop requested while opening cameras; recording was not started")
            except Exception as exc:
                raise RecorderError(f"Connection/pipeline start failed for {camera_name} ({CAMERAS[camera_name]}): {exc}") from exc

        write_metadata(session_dir, args, blob, devices)
        for camera_name, device in devices.items():
            encoded_queue, nn_queue = output_queues[camera_name]
            # Discard pre-session packets accumulated while later cameras were
            # connecting. Every saved file therefore starts after all six passed.
            while encoded_queue.tryGet() is not None:
                pass
            if nn_queue is not None:
                while nn_queue.tryGet() is not None:
                    pass
            worker = CameraWorker(
                camera_name, CAMERAS[camera_name], device, encoded_queue, nn_queue,
                session_dir, args,
                class_names, stop_event, error_queue,
            )
            workers.append(worker)
            worker.start()

        started = time.monotonic()
        next_status = started + STATUS_INTERVAL_SEC
        mode_text = "record-only" if args.record_only else "YOLOP inference + recording"
        print(f"All six cameras are running in {mode_text} mode. Press Ctrl+C to stop.")
        while not stop_event.is_set():
            try:
                failed_camera, exc = error_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                raise RecorderError(f"Fatal camera error [{failed_camera}]: {exc}") from exc

            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                print(f"Duration {args.duration:g}s reached; stopping.")
                stop_event.set()
                break
            if now >= next_status:
                elapsed = max(now - started, 1e-9)
                print(f"--- status {elapsed:.1f}s ---")
                for worker in workers:
                    count = worker.snapshot()
                    last_activity = count.last_frame_monotonic or count.started_monotonic
                    if now - last_activity > CAMERA_STALL_TIMEOUT_SEC:
                        raise RecorderError(
                            f"Camera stalled [{worker.camera_name}]: no encoded or NN packet for "
                            f"{now - last_activity:.1f}s"
                        )
                    record_fps = count.encoded_frames / elapsed
                    if args.record_only:
                        print(
                            f"{worker.camera_name}: inference=disabled, record={record_fps:.2f} FPS, "
                            f"saved_frames={count.encoded_frames}, json_records={count.json_records}"
                        )
                    else:
                        infer_fps = count.inference_frames / elapsed
                        print(
                            f"{worker.camera_name}: inference={infer_fps:.2f} FPS, "
                            f"record={record_fps:.2f} FPS, saved_frames={count.encoded_frames}, "
                            f"json_records={count.json_records}"
                        )
                next_status = now + STATUS_INTERVAL_SEC
            time.sleep(0.05)

        # A worker sets the event after enqueuing its exception. Do not let that
        # path look like a successful duration/signal stop.
        try:
            failed_camera, exc = error_queue.get_nowait()
        except queue.Empty:
            pass
        else:
            raise RecorderError(f"Fatal camera error [{failed_camera}]: {exc}") from exc
        return 0
    finally:
        stop_event.set()
        for worker in workers:
            worker.join(timeout=10.0)
            if worker.is_alive():
                print(f"WARNING: {worker.camera_name} writer did not stop within 10 seconds", file=sys.stderr)
        for camera_name, device in reversed(list(devices.items())):
            try:
                device.close()
                print(f"Closed {camera_name}")
            except Exception as exc:
                print(f"WARNING: failed to close {camera_name}: {exc}", file=sys.stderr)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        print("All files and DepthAI devices are closed.")


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        blob = validate_args(args)
        mapped, missing = discover_cameras()
        print_discovery(mapped, missing)
        if args.inspect_nn:
            if blob is None:
                raise RecorderError("--inspect-nn requires --blob")
            return inspect_nn(args, blob, mapped)
        if missing:
            raise RecorderError("Six-camera requirement is not satisfied; recording was not started")
        return run_recording(args, blob, mapped)
    except RecorderError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        # Normally handled by the installed signal handler; this covers startup.
        print("Interrupted before recording startup completed", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
