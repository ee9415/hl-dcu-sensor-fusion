from __future__ import annotations

import numpy as np


INPUT_SIZE = 320
STRIDES = (8, 16, 32)
ANCHORS = (
    ((12.0, 16.0), (19.0, 36.0), (40.0, 28.0)),
    ((36.0, 75.0), (76.0, 55.0), (72.0, 146.0)),
    ((142.0, 110.0), (192.0, 243.0), (459.0, 401.0)),
)


def diagnostic_bgr_frame() -> np.ndarray:
    """Return a deterministic image for exact host/device tensor comparison."""
    y, x = np.indices((INPUT_SIZE, INPUT_SIZE), dtype=np.uint16)
    blue = (3 * x + y) % 256
    green = (x + 5 * y) % 256
    red = (7 * x + 11 * y) % 256
    return np.ascontiguousarray(
        np.stack((blue, green, red), axis=-1).astype(np.uint8)
    )


def sigmoid(value: np.ndarray) -> np.ndarray:
    value = np.clip(value, -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-value))


def _as_nchw(head: np.ndarray) -> np.ndarray:
    head = np.asarray(head)
    if head.ndim != 4:
        raise ValueError(f"Detection head must be 4-D, got {head.shape}")
    if head.shape[1] == 255:
        return head
    if head.shape[-1] == 255:
        return head.transpose(0, 3, 1, 2)
    raise ValueError(f"Unable to locate 255 detection channels in {head.shape}")


def decode_heads(heads: list[np.ndarray]) -> np.ndarray:
    if len(heads) != 3:
        raise ValueError(f"Expected 3 detection heads, got {len(heads)}")

    decoded = []
    for raw, stride, anchors in zip(heads, STRIDES, ANCHORS, strict=True):
        raw = _as_nchw(raw).astype(np.float32, copy=False)
        batch, _, height, width = raw.shape
        if batch != 1:
            raise ValueError(f"Only batch 1 is supported, got {batch}")
        prediction = raw.reshape(batch, 3, 85, height, width).transpose(0, 1, 3, 4, 2)
        prediction = sigmoid(prediction)

        grid_y, grid_x = np.meshgrid(
            np.arange(height, dtype=np.float32),
            np.arange(width, dtype=np.float32),
            indexing="ij",
        )
        grid = np.stack((grid_x, grid_y), axis=-1)[None, None]
        anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(1, 3, 1, 1, 2)

        prediction[..., 0:2] = (prediction[..., 0:2] * 2.0 - 0.5 + grid) * stride
        prediction[..., 2:4] = (prediction[..., 2:4] * 2.0) ** 2 * anchor_grid
        decoded.append(prediction.reshape(-1, 85))

    return np.concatenate(decoded, axis=0)


def _box_iou(one: np.ndarray, many: np.ndarray) -> np.ndarray:
    top_left = np.maximum(one[:2], many[:, :2])
    bottom_right = np.minimum(one[2:4], many[:, 2:4])
    intersection = np.prod(np.maximum(bottom_right - top_left, 0.0), axis=1)
    one_area = np.prod(np.maximum(one[2:4] - one[:2], 0.0))
    many_area = np.prod(np.maximum(many[:, 2:4] - many[:, :2], 0.0), axis=1)
    return intersection / np.maximum(one_area + many_area - intersection, 1e-7)


def non_max_suppression(
    prediction: np.ndarray,
    confidence_threshold: float = 0.30,
    iou_threshold: float = 0.45,
    max_detections: int = 100,
) -> np.ndarray:
    objectness = prediction[:, 4]
    candidate = prediction[objectness > confidence_threshold]
    if not len(candidate):
        return np.empty((0, 6), dtype=np.float32)

    class_scores = candidate[:, 5:] * candidate[:, 4:5]
    labels = np.argmax(class_scores, axis=1)
    scores = class_scores[np.arange(len(candidate)), labels]
    keep = scores > confidence_threshold
    candidate, labels, scores = candidate[keep], labels[keep], scores[keep]
    if not len(candidate):
        return np.empty((0, 6), dtype=np.float32)

    xywh = candidate[:, :4]
    boxes = np.empty_like(xywh)
    boxes[:, 0:2] = xywh[:, 0:2] - xywh[:, 2:4] / 2.0
    boxes[:, 2:4] = xywh[:, 0:2] + xywh[:, 2:4] / 2.0

    order = np.argsort(scores)[::-1]
    selected: list[int] = []
    while len(order) and len(selected) < max_detections:
        current = int(order[0])
        selected.append(current)
        if len(order) == 1:
            break
        remaining = order[1:]
        same_class = labels[remaining] == labels[current]
        overlaps = _box_iou(boxes[current], boxes[remaining])
        order = remaining[~(same_class & (overlaps > iou_threshold))]

    selected_array = np.asarray(selected, dtype=np.int64)
    return np.column_stack(
        (boxes[selected_array], scores[selected_array], labels[selected_array].astype(np.float32))
    ).astype(np.float32)


def map_letterboxed_box_to_source(box: np.ndarray, source_size: tuple[int, int]) -> np.ndarray:
    source_width, source_height = source_size
    scale = min(INPUT_SIZE / source_width, INPUT_SIZE / source_height)
    resized_width = source_width * scale
    resized_height = source_height * scale
    pad_x = (INPUT_SIZE - resized_width) / 2.0
    pad_y = (INPUT_SIZE - resized_height) / 2.0
    mapped = box.astype(np.float32, copy=True)
    mapped[[0, 2]] = (mapped[[0, 2]] - pad_x) / resized_width
    mapped[[1, 3]] = (mapped[[1, 3]] - pad_y) / resized_height
    return np.clip(mapped, 0.0, 1.0)


def _as_nchw_segmentation(tensor: np.ndarray, channels: int) -> np.ndarray:
    tensor = np.asarray(tensor)
    if tensor.ndim != 4 or tensor.shape[0] != 1:
        raise ValueError(f"Segmentation tensor must be batch-1 4-D, got {tensor.shape}")
    if tensor.shape[1] == channels:
        return tensor
    if tensor.shape[-1] == channels:
        return tensor.transpose(0, 3, 1, 2)
    raise ValueError(f"Unable to locate {channels} channels in {tensor.shape}")


def segmentation_masks(
    drivable: np.ndarray,
    lane: np.ndarray,
    source_size: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray]:
    drivable = _as_nchw_segmentation(drivable, 2)
    lane = _as_nchw_segmentation(lane, 1)
    drivable_mask = np.argmax(drivable, axis=1)[0].astype(np.uint8) * 255
    lane_mask = (lane[0, 0] >= 0.5).astype(np.uint8) * 255

    source_width, source_height = source_size
    scale = min(INPUT_SIZE / source_width, INPUT_SIZE / source_height)
    content_width = int(round(source_width * scale))
    content_height = int(round(source_height * scale))
    left = (INPUT_SIZE - content_width) // 2
    top = (INPUT_SIZE - content_height) // 2
    right = left + content_width
    bottom = top + content_height
    return drivable_mask[top:bottom, left:right], lane_mask[top:bottom, left:right]
