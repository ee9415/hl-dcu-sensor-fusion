"""DepthAI adapter: explicit provenance, geometry, and bounded export cadence."""
import json
from pathlib import Path
import time

import numpy as np

from perception_server import PerceptionServer


def load_config():
    path = Path(__file__).with_name('ros_offload.json')
    return json.loads(path.read_text()) if path.exists() else {}


class PerceptionExport:
    def __init__(self):
        config = load_config()
        self.camera_postprocess = config.get('camera_postprocess', True)
        self.max_sync_error = float(config.get('max_sync_error_s', .075))
        self.server = PerceptionServer(port=int(config.get('port', 8560)),
                                       fps=config.get('export_fps'))
        self.rejected = 0

    def publish(self, preview, nn, depth, depth_frame, intrinsics, drivable, lane,
                detections, measurements, source, model_size):
        if not self.server.wanted():
            return
        times = [frame.getTimestamp().total_seconds() for frame in (preview, nn, depth)]
        if (preview.getSequenceNum() != nn.getSequenceNum() or
                abs(times[0]-times[1]) > .001 or
                (source.get('input_source', 'camera') == 'camera' and
                 abs(times[0]-times[2]) > self.max_sync_error)):
            self.rejected += 1
            raise RuntimeError('Perception export rejected an unsynchronized inference frame')
        rgb = preview.getCvFrame()
        source_height, source_width = rgb.shape[:2]
        transform = preview.getTransformation()
        rgb_k = (np.asarray(transform.getIntrinsicMatrix(), dtype=float)
                 if transform is not None else np.zeros((3, 3)))
        rgb_k = rgb_k.copy()
        rgb_k[:2] /= 2  # Export exact stride-2 RGB, retaining the full field of view.
        available = source.get('input_source', 'camera') == 'camera'
        self.server.publish(dict(
            source=source, frame_seq=int(preview.getSequenceNum()),
            nn_seq=int(nn.getSequenceNum()), depth_seq=int(depth.getSequenceNum()),
            capture_timestamp_s=times[0], nn_timestamp_s=times[1],
            depth_timestamp_s=times[2], sent_monotonic_s=time.monotonic(),
            source_size=[source_width, source_height], model_size=list(model_size),
            mask_coordinates='source_fov_letterbox_removed',
            box_coordinates='normalized_source_xyxy',
            frame_id='cam01_rgb_optical_frame', rgb_encoding='bgr8',
            rgb_intrinsics=rgb_k.tolist(), depth_intrinsics=intrinsics.tolist(),
            rectified=True, depth_aligned_to='rgb', depth_unit='mm',
            depth_invalid_value=0, depth_available=available,
            camera_postprocess=self.camera_postprocess,
            objects=[dict(label=d.label, confidence=d.confidence,
                          xmin=d.xmin, ymin=d.ymin, xmax=d.xmax, ymax=d.ymax,
                          depth_valid=bool(m.valid),
                          x_m=m.x_mm/1000 if m.valid else None,
                          y_m=m.y_mm/1000 if m.valid else None,
                          z_m=m.z_mm/1000 if m.valid else None)
                     for d, m in zip(detections, measurements)],
        ), dict(rgb=np.ascontiguousarray(rgb[::2, ::2]),
                drivable=drivable, lane=lane,
                depth=np.asarray(depth_frame, dtype='<u2')))
