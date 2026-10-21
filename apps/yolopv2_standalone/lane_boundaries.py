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

    def update(self, road, lane, timestamp_s):
        road, lane = np.asarray(road), np.asarray(lane)
        if road.ndim != 2 or min(road.shape) < 2 or road.shape != lane.shape:
            raise ValueError('Road and lane masks must have the same non-empty 2-D shape')
        return self.update_bev(*self._sample(road, lane), timestamp_s,
                               projection=projection_for(road.shape))

    def _sample(self, road, lane):
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
        """Fit two quadratic boundary polynomials with an exact width constraint.

        Each side selects observations against its own prediction. A joint equality
        constraint keeps right(s)-left(s)=configured width. No midpoint is computed.
        """
        c = self.config
        projection = np.eye(3) if projection is None else projection
        empty = np.empty(0)
        def invalid(state):
            self.previous = None
            self.last_supported_time = None
            return BoundaryEstimate(False, state, empty, empty, empty, projection)

        if not np.isfinite(timestamp_s):
            return invalid('invalid_timestamp')
        if self.last_time is not None and timestamp_s <= self.last_time:
            return invalid('nonmonotonic_timestamp')
        dt = .1 if self.last_time is None else min(timestamp_s-self.last_time, .2)
        if self.last_time is not None and timestamp_s-self.last_time > c.max_missing_s:
            self.previous = None
            self.last_supported_time = None
        self.last_time = timestamp_s
        forward, lateral = np.asarray(forward), np.asarray(lateral)
        prior = self.previous
        left = np.full(len(forward), -c.vehicle_width_m/2-c.left_margin_m)
        right = np.full(len(forward), c.vehicle_width_m/2+c.right_margin_m)
        if prior is not None:
            left = np.interp(forward, prior.forward, prior.left)
            right = np.interp(forward, prior.forward, prior.right)
        basis = np.column_stack((np.ones(len(forward)), forward/40, (forward/40)**2))
        rows, targets, weights, observation_indices, observation_sides = [], [], [], [], []
        observed = [0, 0]
        supported = np.zeros(len(forward), bool)
        blocked = np.zeros(len(forward), bool)
        for i, (lo, hi) in enumerate(zip(left, right)):
            indices = np.flatnonzero(road[i])
            runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1)
            # Select only a connected road run overlapping this boundary pair.
            candidates = [r for r in runs if len(r) and lateral[r[-1]] >= lo and lateral[r[0]] <= hi]
            if not candidates:
                continue
            run = max(candidates, key=lambda r: min(hi, lateral[r[-1]])-max(lo, lateral[r[0]]))
            road_lo, road_hi = lateral[run[0]], lateral[run[-1]]
            if road_hi-road_lo < c.width_m-.05:
                blocked[i] = True
                continue
            supported[i] = road_lo <= lo+.08 and road_hi >= hi-.08
            lane_x = lateral[np.flatnonzero(lane[i])]
            for side, prediction in enumerate((lo, hi)):
                # Keep left/right associations separate, including a missing side.
                choices = lane_x[np.abs(lane_x-prediction) <= c.observation_gate_m]
                choices = choices[(choices >= road_lo-.1) & (choices <= road_hi+.1)]
                edge = road_lo if side == 0 else road_hi
                weight = 6.
                if len(choices):
                    value = choices[np.argmin(np.abs(choices-prediction))]
                elif abs(edge-prediction) <= c.observation_gate_m:
                    value, weight = edge, 1.
                else:
                    continue  # Wide road edge must not pull the pair outward.
                row = np.zeros(6)
                row[side*3:side*3+3] = basis[i]
                rows.append(row); targets.append(value); weights.append(weight)
                observation_indices.append(i); observation_sides.append(side)
                observed[side] += 1
        # Never draw through a visible near constriction. Far constrictions truncate.
        stop = np.flatnonzero(blocked)
        end = int(stop[0]) if len(stop) else len(forward)
        if end < 12:
            if len(stop):
                return invalid('insufficient_width')
        support = supported[:end]
        if np.count_nonzero(support) < 12:
            # A visible conflicting road is not a dropout; never hold across it.
            if not np.any(road[:12]) and prior is not None and self.last_supported_time is not None and timestamp_s-self.last_supported_time <= c.max_missing_s:
                return BoundaryEstimate(True, 'held', prior.forward, prior.left, prior.right,
                                        projection, 'held', 'held')
            return invalid('boundary_conflict' if np.any(road[:12]) else 'road_unavailable')
        self.last_supported_time = timestamp_s
        selected = [i for i, sample in enumerate(observation_indices) if sample < end]
        rows = [rows[i] for i in selected]
        targets = [targets[i] for i in selected]
        weights = [weights[i] for i in selected]
        observed = [sum(observation_sides[i] == side for i in selected) for side in (0, 1)]
        # Prior/vehicle-anchored boundaries regularize sparse markings independently.
        for side, prediction in enumerate((left, right)):
            for i in range(end):
                row = np.zeros(6); row[side*3:side*3+3] = basis[i]
                rows.append(row); targets.append(prediction[i]); weights.append(2.)
        a = np.asarray(rows); b = np.asarray(targets); w = np.sqrt(weights)
        a, b = a*w[:, None], b*w
        constraint = np.column_stack((-np.eye(3), np.eye(3)))
        system = np.block([[a.T@a+np.eye(6)*1e-6, constraint.T],
                           [constraint, np.zeros((3, 3))]])
        rhs = np.r_[a.T@b, c.width_m, 0., 0.]
        coef = np.linalg.solve(system, rhs)[:6]
        fitted_left, fitted_right = basis@coef[:3], basis@coef[3:]
        # A common bounded shift preserves the exact pair width after smoothing.
        delta = fitted_left-left
        shift = delta * min(1., c.max_lateral_rate_m_s*dt/max(float(np.max(np.abs(delta))), 1e-9))
        fitted_left = left+shift
        fitted_right = right+shift
        # Validate both curves against every available road row, not just observations.
        for i in range(end):
            between = (lateral >= fitted_left[i]+.05) & (lateral <= fitted_right[i]-.05)
            if not np.any(between) or not np.all(road[i, between]):
                end = i
                break
        if end < 12:
            return invalid('boundary_conflict')
        states = ['observed' if n >= 8 else 'inferred' for n in observed]
        result = BoundaryEstimate(True, 'tracking' if all(n >= 8 for n in observed) else 'inferred',
                                  forward[:end].copy(), fitted_left[:end], fitted_right[:end],
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
