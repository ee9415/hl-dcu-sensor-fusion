"""Display-only RGB-aligned depth colorization; raw millimetres stay untouched."""
import numpy as np


def colorize_depth(depth_mm):
    depth = np.asarray(depth_mm)
    if depth.ndim != 2 or not depth.size:
        raise ValueError('depth must be a nonempty 2-D array')
    # Fixed log scale: near red (0.3 m), far blue (10 m); no per-frame autoscale.
    valid = np.isfinite(depth) & (depth > 0)
    safe = np.where(valid, depth, 300).astype(np.float32)
    t = np.clip(np.log(np.maximum(safe, 300)/300)/np.log(10000/300), 0, 1)
    stops = np.array([[0, 0, 255], [0, 255, 255], [0, 255, 0],
                      [255, 255, 0], [255, 0, 0]], dtype=np.float32)
    scaled = t*4
    lower = np.minimum(scaled.astype(np.int32), 3)
    fraction = (scaled-lower)[..., None]
    bgr = ((1-fraction)*stops[lower]+fraction*stops[lower+1]).astype(np.uint8)
    bgr[~valid] = 0
    return np.ascontiguousarray(bgr)
