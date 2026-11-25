"""ROS-independent host adapter reusing the camera's actual tracking algorithms."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from image_boundaries import ImageBoundaryTracker
from lane_boundaries import BoundaryConfig, projection_for
from lane_contours import extract_lane_contours
from lane_overlay import TopViewRenderer, overlay_tracked_skeleton
from skeleton_response import response_config
from stop_line import StopLineTracker, draw_stop_line
from top_midpoint_guidance import displayed_midpoint_guidance
from top_view_controls import top_view_config
from virtual_guidance import GuidanceConfig, draw_guidance


def validate_frame(metadata, arrays):
    rgb, road, lane, depth = (arrays[k] for k in ('rgb', 'drivable', 'lane', 'depth'))
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or
            road.dtype != np.uint8 or lane.dtype != np.uint8 or road.ndim != 2 or
            road.shape != lane.shape or depth.dtype != np.dtype('<u2') or depth.ndim != 2):
        raise ValueError('Invalid perception image encodings/shapes')
    if (not np.isin(lane, (0, 255)).all() or not np.isin(road, (0, 255)).all() or
            metadata.get('mask_coordinates') != 'source_fov_letterbox_removed' or
            metadata.get('box_coordinates') != 'normalized_source_xyxy' or
            metadata.get('depth_unit') != 'mm' or metadata.get('depth_aligned_to') != 'rgb'):
        raise ValueError('Invalid perception coordinate contract')
    width, height = metadata['source_size']
    if rgb.shape[:2] != ((height+1)//2, (width+1)//2) or depth.shape != (height, width):
        raise ValueError('RGB/Depth geometry mismatch')
    for key in ('capture_timestamp_s', 'nn_timestamp_s', 'depth_timestamp_s'):
        if not np.isfinite(metadata[key]):
            raise ValueError('Invalid timestamp')
    if (metadata['frame_seq'] != metadata['nn_seq'] or
            abs(metadata['capture_timestamp_s']-metadata['nn_timestamp_s']) > .001 or
            (metadata['depth_available'] and
             abs(metadata['capture_timestamp_s']-metadata['depth_timestamp_s']) > .075)):
        raise ValueError('Unsynchronized frame')
    if metadata['source'].get('input_source', 'camera') != 'camera' and metadata['depth_available']:
        raise ValueError('PC video must not carry live camera depth')
    for key in ('rgb_intrinsics', 'depth_intrinsics'):
        matrix = np.asarray(metadata[key], dtype=float)
        if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            raise ValueError('Invalid calibration')
    for obj in metadata['objects']:
        box = np.array([obj[k] for k in ('xmin', 'ymin', 'xmax', 'ymax')])
        if (not np.isfinite(box).all() or np.any(box < 0) or np.any(box > 1) or
                box[0] >= box[2] or box[1] >= box[3] or
                not np.isfinite(obj['confidence']) or not 0 <= obj['confidence'] <= 1):
            raise ValueError('Invalid object box')


class HostPostprocess:
    def __init__(self):
        self.base = BoundaryConfig.load()
        self.guidance_config = GuidanceConfig.load()
        self.top_config = replace(self.base, near_left_u=.075724, near_right_u=.798382,
            near_v=1., far_left_u=.382381, far_right_u=.567136, far_v=.62)
        self.key = None
        self.reset()

    def reset(self):
        self.tracker = ImageBoundaryTracker(self.base)
        self.stop = StopLineTracker()
        self.top = TopViewRenderer(near_row=419, horizontal_scale=1.75)
        self.last_timestamp = None
        self.key = None

    def process(self, metadata, arrays):
        validate_frame(metadata, arrays)
        source = metadata['source']
        key = (metadata['session'], source.get('input_source', 'camera'),
               source.get('source_epoch'), source.get('source_media_epoch'),
               source.get('processing_epoch'), arrays['lane'].shape)
        timestamp = metadata['capture_timestamp_s']
        if (key != self.key or self.last_timestamp is None or
                not 0 < timestamp-self.last_timestamp <= 1.):
            self.reset()
        self.key, self.last_timestamp = key, timestamp
        self.tracker.config = response_config(self.base, source.get('skeleton_response_speed', 1.))
        rgb, lane = arrays['rgb'], arrays['lane']
        contours = extract_lane_contours(lane)
        boundaries = self.tracker.update_contours(contours, lane.shape, timestamp, rgb=rgb)
        detections = [SimpleNamespace(**obj) for obj in metadata['objects']]
        stop = self.stop.update(rgb, boundaries, timestamp, detections)
        projection = projection_for(lane.shape, top_view_config(self.top_config,
            source.get('top_view_spread', 0.), source.get('top_view_ahead', 0.)))
        top = self.top.render(rgb, lane, projection, 'raw_rgb')
        guidance = displayed_midpoint_guidance(boundaries, self.top.projection,
            self.guidance_config, near_row=self.top.near_row)
        debug = rgb.copy()
        overlay_tracked_skeleton(debug, boundaries, lane.shape)
        draw_stop_line(debug, stop)
        draw_guidance(debug, guidance, show_boundaries=False)
        overlay_tracked_skeleton(top, boundaries, lane.shape, self.top.projection, self.top.valid)
        draw_guidance(top, guidance, projection=self.top.projection, show_boundaries=False)
        top[~self.top.valid] = 0
        result = dict(session=metadata['session'], frame_seq=metadata['frame_seq'],
            capture_timestamp_s=timestamp, processing_location='ros', stale=False,
            geometry_units='image_pixels_320x180', metric_path_available=False,
            **boundaries.telemetry(self.tracker.config), **stop.telemetry(), **guidance.telemetry())
        return result, debug, top
