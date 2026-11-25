import json
import sys
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo


def load_core():
    folder = Path(get_package_share_directory('hl_oak_perception'))/'core'
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))


def image_message(array, encoding, header):
    image = Image()
    image.header = header
    image.height, image.width = array.shape[:2]
    image.encoding = encoding
    image.is_bigendian = False
    image.step = image.width * (3 if encoding == 'bgr8' else 2 if encoding == '16UC1' else 1)
    # Avoid the generated setter's per-byte Python validation loop.
    image.data.frombytes(memoryview(np.ascontiguousarray(array)).cast('B'))
    return image


def image_array(message):
    dtype = '<u2' if message.encoding == '16UC1' else 'u1'
    channels = 3 if message.encoding == 'bgr8' else 1
    shape = (message.height, message.width, channels) if channels == 3 else (message.height, message.width)
    expected_step = message.width*channels*np.dtype(dtype).itemsize
    if message.step != expected_step or message.is_bigendian or len(message.data) != message.height*message.step:
        raise ValueError('Invalid image buffer')
    return np.frombuffer(message.data, dtype=dtype).reshape(shape)


def camera_info(array, intrinsics, header):
    info = CameraInfo()
    info.header = header
    info.height, info.width = array.shape[:2]
    matrix = np.array(intrinsics, dtype=float)
    info.k = matrix.ravel().tolist()
    info.r = np.eye(3).ravel().tolist()
    info.p = np.column_stack((matrix, np.zeros(3))).ravel().tolist()
    info.distortion_model = 'plumb_bob'
    info.d = [0.]*5  # Both streams are rectified by the device pipeline.
    return info


def json_text(value):
    def convert(item):
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(type(item).__name__)
    return json.dumps(value, default=convert, allow_nan=False)
