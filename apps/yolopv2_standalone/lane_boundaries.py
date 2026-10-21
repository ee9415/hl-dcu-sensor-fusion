"""Halla campus virtual boundary pair. No center path is estimated.

Distances use the existing *uncalibrated* development IPM. Width in meters
is a requested physical width, not a claim of metric accuracy before calibration.
Only NumPy is required on the device.
"""
from dataclasses import dataclass, fields
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class BoundaryConfig:
    vehicle_width_m: float = 1.8
    left_margin_m: float = .3
    right_margin_m: float = .3
    observation_gate_m: float = .65
    max_lateral_rate_m_s: float = 1.0
    max_missing_s: float = .5
    max_forward_m: float = 25.0
    max_lane_gap_s: float = 2.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError(f'{field.name} must be a finite number')
            if value < 0 or (value == 0 and field.name not in ('left_margin_m', 'right_margin_m')):
                raise ValueError(f'{field.name} is out of range')
        if self.width_m >= 12 or self.max_forward_m > 40:
            raise ValueError('Requested corridor exceeds development BEV extent')

    @property
    def width_m(self):
        return self.vehicle_width_m + self.left_margin_m + self.right_margin_m

    @classmethod
    def load(cls, path=None):
        # Fail explicitly on malformed settings; do not silently use another width.
        with Path(path or Path(__file__).with_name('lane_boundaries.json')).open(encoding='utf-8') as stream:
            return cls(**json.load(stream))


def transform(points, matrix):
    points = np.asarray(points, dtype=float)
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    return homogeneous[:, :2] / homogeneous[:, 2:3]


def projection_for(shape):
    """Same development trapezoid as the previous BEV, independent of its logic."""
    h, w = shape
    source = np.array([[.04*(w-1), .98*(h-1)], [.96*(w-1), .98*(h-1)],
                       [.62*(w-1), .38*(h-1)], [.38*(w-1), .38*(h-1)]])
    destination = np.array([[0, 479], [639, 479], [639, 0], [0, 0]])
    a, b = [], []
    for (x, y), (u, v) in zip(source, destination):
        a.extend([[x,y,1,0,0,0,-u*x,-u*y], [0,0,0,x,y,1,-v*x,-v*y]])
        b.extend([u, v])
    return np.linalg.inv(np.append(np.linalg.solve(a, b), 1).reshape(3, 3))


def white_lane_evidence(rgb, lane):
    """Refine the NN lane region to local white/gray paint peaks in the RGB image."""
    h, w = lane.shape
    sy, sx = rgb.shape[0]//h, rgb.shape[1]//w
    if rgb.shape[:2] != (h*sy, w*sx):
        raise ValueError('RGB size must be an integer multiple of the lane mask')
    image = rgb.reshape(h, sy, w, sx, 3).mean(axis=(1, 3))
    brightness = image.mean(axis=2)
    padded = np.pad(brightness, ((0, 0), (4, 4)), mode='edge')
    background = (padded[:, :w]+padded[:, 8:8+w])*.5
    supported = np.zeros_like(lane, bool)
    for offset in range(-3, 4):
        source = np.arange(w)+offset
        valid = (source >= 0) & (source < w)
        supported[:, valid] |= lane[:, source[valid]] != 0
    neutral = image.max(axis=2)-image.min(axis=2) < 75
    return (supported & neutral & (brightness >= 75) & (brightness-background >= 10)).astype(np.uint8)*255


@dataclass(frozen=True)
class BoundaryEstimate:
    valid: bool
    state: str
    forward: np.ndarray
    left: np.ndarray
    right: np.ndarray
    projection: np.ndarray
    left_state: str = 'unavailable'
    right_state: str = 'unavailable'

    def telemetry(self, config):
        return dict(boundary_valid=self.valid, boundary_state=self.state,
                    left_boundary_state=self.left_state, right_boundary_state=self.right_state,
                    vehicle_width_m=config.vehicle_width_m, left_margin_m=config.left_margin_m,
                    right_margin_m=config.right_margin_m, corridor_width_m=config.width_m,
                    minimum_clearance_width_m=config.width_m,
                    boundary_width_min_dev=float(np.min(self.right-self.left)) if self.valid else None,
                    boundary_width_max_dev=float(np.max(self.right-self.left)) if self.valid else None,
                    clearance_sufficient=bool(np.min(self.right-self.left) >= config.width_m) if self.valid else False,
                    boundary_units='uncalibrated_bev_development_units',
                    boundary_forward_dev=self.forward.tolist() if self.valid else [],
                    left_boundary_dev=self.left.tolist() if self.valid else [],
                    right_boundary_dev=self.right.tolist() if self.valid else [])


class BoundaryTracker:
    def __init__(self, config=None):
        self.config = config or BoundaryConfig()
        self.previous = None
        self.last_time = None
        self.last_supported_time = None
        self.side_observed_time = [None, None]
        self.pending_acquisition = None

    def update(self, road, lane, timestamp_s, rgb=None):
        road, lane = np.asarray(road), np.asarray(lane)
        if road.ndim != 2 or min(road.shape) < 2 or road.shape != lane.shape:
            raise ValueError('Road and lane masks must have the same non-empty 2-D shape')
        if rgb is not None:
            lane = white_lane_evidence(rgb, lane)
        return self.update_bev(*self._sample(road, lane), timestamp_s,
                               projection=projection_for(road.shape))

    def _sample(self, road, lane):
        # The segmentation heads may exclude painted lane pixels from the road.
        # Restore only short, lane-supported horizontal holes bracketed by road;
        # a marking must not split otherwise connected pavement into two roads.
        road = road.copy()
        max_gap = max(1, round(road.shape[1] * 6 / 320))
        for y in range(road.shape[0]):
            indices = np.flatnonzero(road[y])
            gaps = np.flatnonzero(np.diff(indices) > 1)
            for a, b in zip(indices[gaps], indices[gaps+1]):
                if 1 < b-a <= max_gap+1 and np.any(lane[y, a+1:b]):
                    road[y, a+1:b] = 255
        # A small fixed BEV raster bounds runtime. Nearest-neighbor binary sampling.
        lateral = np.linspace(-6, 6, 241)
        forward = np.linspace(.5, self.config.max_forward_m, 60)
        xx, yy = np.meshgrid((lateral/12+.5)*639, (1-forward/40)*479)
        pixels = np.rint(transform(np.column_stack((xx.ravel(), yy.ravel())),
                                    projection_for(road.shape))).astype(int)
        x, y = pixels.T
        inside = (x >= 0) & (x < road.shape[1]) & (y >= 0) & (y < road.shape[0])
        outputs = []
        for mask in (road, lane):
            sampled = np.zeros(len(x), dtype=bool)
            sampled[inside] = mask[y[inside], x[inside]] != 0
            outputs.append(sampled.reshape(xx.shape))
        return forward, lateral, *outputs

    def update_bev(self, forward, lateral, road, lane, timestamp_s, projection=None):
        """Track each painted boundary directly. Vehicle width never sets its position."""
        c = self.config
        projection = np.eye(3) if projection is None else projection
        empty = np.empty(0)

        def invalid(state):
            # Expired paint evidence is hidden, but must not trigger immediate
            # acquisition of the much wider road's remote outer stripes.
            if state != 'lane_unavailable':
                self.previous = None
                self.side_observed_time = [None, None]
            return BoundaryEstimate(False, state, empty, empty, empty, projection)

        if not np.isfinite(timestamp_s):
            return invalid('invalid_timestamp')
        if self.last_time is not None and timestamp_s <= self.last_time:
            return invalid('nonmonotonic_timestamp')
        dt = .1 if self.last_time is None else min(timestamp_s-self.last_time, .2)
        if self.last_time is not None and timestamp_s-self.last_time > c.max_missing_s:
            self.previous = None
            self.side_observed_time = [None, None]
        self.last_time = timestamp_s
        forward, lateral = np.asarray(forward), np.asarray(lateral)
        prior = self.previous
        if np.count_nonzero(np.any(road, axis=1)) < 12:
            if prior is not None and self.last_supported_time is not None and timestamp_s-self.last_supported_time <= c.max_missing_s:
                return BoundaryEstimate(True, 'held', prior.forward, prior.left, prior.right,
                                        projection, 'held', 'held')
            return invalid('road_unavailable')
        predictions = [None, None] if prior is None else [
            np.interp(forward, prior.forward, prior.left),
            np.interp(forward, prior.forward, prior.right)]
        if prior is not None and any(t is None or timestamp_s-t > c.max_lane_gap_s
                                     for t in self.side_observed_time):
            # Reacquire a stable returning pair after a long gap, without accepting
            # the expanded road's much wider outer stripes as the current lane.
            fresh = BoundaryTracker(c).update_bev(forward, lateral, road, lane,
                                                  timestamp_s, projection)
            allowed = fresh.valid and np.median(fresh.right-fresh.left) <= 1.25*np.median(prior.right-prior.left)
            if allowed:
                pending = self.pending_acquisition
                stable = pending is not None and all(
                    np.median(abs(getattr(fresh, side)-np.interp(fresh.forward, pending[0].forward,
                              getattr(pending[0], side)))) < .25 for side in ('left', 'right'))
                count = pending[1]+1 if stable else 1
                self.pending_acquisition = (fresh, count)
                if count >= 3:
                    self.previous = fresh
                    self.side_observed_time = [timestamp_s, timestamp_s]
                    self.last_supported_time = timestamp_s
                    self.pending_acquisition = None
                    return fresh
            else:
                self.pending_acquisition = None
        candidates = []
        for i in range(len(forward)):
            indices = np.flatnonzero(lane[i])
            runs = [r for r in np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1) if len(r)]
            # Use the middle of each paint stripe, not its inward-facing pixel.
            values = [float(np.median(lateral[r])) for r in runs if lateral[r[-1]]-lateral[r[0]] <= .7]
            values = [v for v in values if np.any(road[i, abs(lateral-v) <= .4])]
            candidates.append(values)
        curves, states, supports = [], [], []
        for side in (0, 1):
            chosen_i, chosen_x = [], []
            prediction = predictions[side]
            for i, values in enumerate(candidates):
                values = np.asarray(values)
                if prediction is None:
                    # Acquire the nearest stripe on each side of the camera axis.
                    values = values[values < -.15] if side == 0 else values[values > .15]
                    reference = (chosen_x[-1] if chosen_x else 0.)
                    if len(chosen_i) >= 2:
                        slope = (chosen_x[-1]-chosen_x[-2])/(forward[chosen_i[-1]]-forward[chosen_i[-2]])
                        reference += np.clip(slope, -.5, .5)*(forward[i]-forward[chosen_i[-1]])
                    gate = c.observation_gate_m if chosen_i else 6.
                else:
                    reference, gate = prediction[i], c.observation_gate_m
                values = values[abs(values-reference) <= gate]
                if len(values):
                    chosen_i.append(i)
                    chosen_x.append(float(values[np.argmin(abs(values-reference))]))
            indices = np.asarray(chosen_i, dtype=int)
            values = np.asarray(chosen_x)
            enough = len(indices) >= 8 and np.ptp(forward[indices]) >= 4.
            if enough:
                # Robust independent quadratic fit; no equal-width or midpoint constraint.
                keep = np.ones(len(values), bool)
                for _ in range(3):
                    poly = np.polynomial.Polynomial.fit(forward[indices][keep], values[keep], 2)
                    residual = abs(poly(forward[indices])-values)
                    keep = residual <= max(.12, 2.5*float(np.median(residual)))
                    if np.count_nonzero(keep) < 8:
                        enough = False
                        break
                if enough:
                    indices, values = indices[keep], values[keep]
                    poly = np.polynomial.Polynomial.fit(forward[indices], values, 2)
                    curve = poly(forward)
                    if prediction is not None:
                        # Observed paint must not be pulled toward vehicle-width anchors.
                        delta = curve-prediction
                        curve = prediction + delta*min(1., c.max_lateral_rate_m_s*dt/max(float(np.max(abs(delta))), 1e-9))
                    self.side_observed_time[side] = timestamp_s
            if not enough:
                curve = None
            curves.append(curve)
            states.append('observed' if enough else 'inferred')
            supports.append(indices if enough else np.empty(0, dtype=int))
        if prior is None and any(curve is None for curve in curves):
            return invalid('lane_unavailable')
        for side in (0, 1):
            if curves[side] is None:
                last = self.side_observed_time[side]
                if last is None or timestamp_s-last > c.max_lane_gap_s:
                    return invalid('lane_unavailable')
                other = 1-side
                # Continue the missing side with the last *observed* inter-boundary width.
                delta = curves[other]-predictions[other] if curves[other] is not None else 0.
                curves[side] = predictions[side]+delta
        left, right = curves
        # Do not extrapolate past the observed road, or turn road expansion into new width.
        end = len(forward)
        start = 0
        if all(len(s) for s in supports):
            start = max(int(s[0]) for s in supports)
            end = min(end, *(int(s[-1])+1 for s in supports))
        elif prior is not None:
            start = int(np.searchsorted(forward, prior.forward[0]))
            end = min(end, int(np.searchsorted(forward, prior.forward[-1], side='right')))
        for i in range(start, end):
            if right[i]-left[i] < .5:
                end = i
                break
            interior = (lateral >= left[i]+.15) & (lateral <= right[i]-.15)
            if not np.any(interior) or np.mean(road[i, interior]) < .90:
                end = i
                break
        if end-start < 12:
            return invalid('boundary_conflict')
        self.last_supported_time = timestamp_s
        state = 'tracking' if all(s == 'observed' for s in states) else 'inferred'
        if np.min(right[start:end]-left[start:end]) < c.width_m:
            state = 'insufficient_width'
        result = BoundaryEstimate(True, state, forward[start:end].copy(), left[start:end], right[start:end],
                                  projection, *states)
        self.previous = result
        return result


def draw_polyline(frame, points, color, thickness=2, dashed=False):
    h, w = frame.shape[:2]
    for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
        if dashed and index % 6 >= 3:
            continue
        # Bound all raster work even for projections near the horizon.
        if not np.all(np.isfinite([start, end])) or np.max(np.abs([start, end])) > 10000:
            continue
        count = min(2000, int(np.max(np.abs(end-start)))+2)
        x, y = np.rint(np.linspace(start, end, count)).astype(int).T
        for dy in range(-thickness, thickness+1):
            for dx in range(-thickness, thickness+1):
                xx, yy = x+dx, y+dy
                valid = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
                frame[yy[valid], xx[valid]] = color


def boundary_points(estimate, side):
    return np.column_stack(((getattr(estimate, side)/12+.5)*639,
                            (1-estimate.forward/40)*479))


def draw_boundaries(frame, estimate, mask_size=None):
    if not estimate.valid:
        return
    for side, color, state in (('left', (0, 220, 255), estimate.left_state),
                               ('right', (255, 180, 0), estimate.right_state)):
        points = boundary_points(estimate, side)
        if mask_size is not None:
            points = transform(points, estimate.projection)
            points *= np.array([frame.shape[1]/mask_size[0], frame.shape[0]/mask_size[1]])
        draw_polyline(frame, points, color, dashed=state != 'observed')


def render_boundary_bev(estimate):
    frame = np.full((480, 640, 3), (14, 22, 30), np.uint8)
    frame[::60] = (43, 58, 68)
    draw_boundaries(frame, estimate)
    return frame
