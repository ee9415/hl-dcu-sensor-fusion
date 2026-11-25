"""A-YOLOM v4 vehicle/person/traffic-light heads and two-class segmentation."""
import cv2
import numpy as np

from postprocess import sigmoid

CLASS_NAMES = ('vehicle', 'person', 'tl_red', 'tl_yellow', 'tl_green')
TENSOR_NAMES = ('det_stride_8', 'det_stride_16', 'det_stride_32',
                'drivable_area', 'lane_line',
                'person_tl_stride_8', 'person_tl_stride_16', 'person_tl_stride_32')


def as_nchw(tensor, channels, model_size, stride=1):
    width, height = model_size
    raw = np.asarray(tensor, dtype=np.float32)
    expected = (1, channels, height // stride, width // stride)
    if raw.shape == (1, height // stride, width // stride, channels):
        raw = raw.transpose(0, 3, 1, 2)
    if raw.shape != expected or not np.isfinite(raw).all():
        raise ValueError(f'Invalid A-YOLOM tensor: expected {expected}, got {raw.shape}')
    return raw


def decode_prediction(tensors, model_size):
    """Return xywh/objectness/classes for the common class-aware NMS.

    YOLOv8 has no objectness. A constant 1 preserves its class probabilities.
    The added detector's four local classes map to global IDs 1 through 4.
    """
    predictions = []
    for prefix, classes, offset in (('det', 1, 0), ('person_tl', 4, 1)):
        for stride in (8, 16, 32):
            raw = as_nchw(tensors[f'{prefix}_stride_{stride}'], 64 + classes,
                          model_size, stride)[0]
            _, height, width = raw.shape
            distribution = raw[:64].reshape(4, 16, height, width)
            distribution = np.exp(distribution - distribution.max(axis=1, keepdims=True))
            distribution /= distribution.sum(axis=1, keepdims=True)
            distance = (distribution * np.arange(16, dtype=np.float32)[None, :, None, None]).sum(axis=1)
            yy, xx = np.mgrid[:height, :width].astype(np.float32)
            x1, y1 = xx + .5 - distance[0], yy + .5 - distance[1]
            x2, y2 = xx + .5 + distance[2], yy + .5 + distance[3]
            xywh = np.stack(((x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1), axis=-1).reshape(-1, 4) * stride
            scores = np.zeros((height*width, len(CLASS_NAMES)), dtype=np.float32)
            scores[:, offset:offset+classes] = sigmoid(raw[64:]).reshape(classes, -1).T
            predictions.append(np.column_stack((xywh, np.ones(height*width, np.float32), scores)))
    return np.concatenate(predictions)


def segmentation_scores(tensor, source_size, model_size, output_size=(320, 180)):
    """Crop padding and resize logits before class selection, retaining ROS geometry."""
    logits = as_nchw(tensor, 2, model_size)[0].transpose(1, 2, 0)
    width, height = model_size
    ratio = min(width/source_size[0], height/source_size[1])
    content_width, content_height = (round(source_size[0]*ratio), round(source_size[1]*ratio))
    left, top = (width-content_width)//2, (height-content_height)//2
    logits = logits[top:top+content_height, left:left+content_width]
    logits = cv2.resize(logits, output_size, interpolation=cv2.INTER_LINEAR)
    return sigmoid(logits[:, :, 1] - logits[:, :, 0])


def segmentation_masks(road, lane, source_size, model_size):
    # Strict > preserves background (channel 0) when the two logits are tied.
    return tuple((segmentation_scores(t, source_size, model_size) > .5).astype(np.uint8)*255
                 for t in (road, lane))
