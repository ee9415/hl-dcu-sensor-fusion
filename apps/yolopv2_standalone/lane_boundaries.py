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
    max_lane_gap_s: float = 2.0  # Delay before wider-search reacquisition, not expiry.

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
    """Find neutral paint ridges, including regions missed by the NN lane head."""
    h, w = lane.shape
    sy, sx = rgb.shape[0]//h, rgb.shape[1]//w
    if rgb.shape[:2] != (h*sy, w*sx):
        raise ValueError('RGB size must be an integer multiple of the lane mask')
    image = rgb.reshape(h, sy, w, sx, 3).mean(axis=(1, 3))
    brightness = image.mean(axis=2)
    # Suppress asphalt grain before looking for narrow paint ridges.
    padded_image = np.pad(brightness, 1, mode='edge')
    brightness = sum(weight*padded_image[dy:dy+h, dx:dx+w]
                     for dy, wy in enumerate((1, 2, 1))
                     for dx, wx in enumerate((1, 2, 1))
                     for weight in (wy*wx,))/16.
    padded = np.pad(brightness, ((0, 0), (4, 4)), mode='edge')
    background = (padded[:, :w]+padded[:, 8:8+w])*.5
    supported = np.zeros_like(lane, bool)
    for offset in range(-3, 4):
        source = np.arange(w)+offset
        valid = (source >= 0) & (source < w)
        supported[:, valid] |= lane[:, source[valid]] != 0
    neutral = image.max(axis=2)-image.min(axis=2) < 75
    # Independent RGB evidence must be brighter than BOTH shoulders; a single
    # bright vehicle/curb edge is insufficient. Multiple scales cover perspective.
    ridge = np.zeros_like(lane, bool)
    for radius in (2, 4, 6):
        padded = np.pad(brightness, ((0, 0), (radius, radius)), mode='edge')
        shoulders = np.maximum(padded[:, :w], padded[:, 2*radius:2*radius+w])
        ridge |= brightness-shoulders >= 12
    evidence = (supported & (brightness-background >= 10)) | ridge
    return (evidence & neutral & (brightness >= 75)).astype(np.uint8)*255


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

    @property
    def left_valid(self):
        return bool(np.count_nonzero(np.isfinite(self.left)) >= 8)

    @property
    def right_valid(self):
        return bool(np.count_nonzero(np.isfinite(self.right)) >= 8)

    @property
    def pair_valid(self):
        common = np.isfinite(self.left) & np.isfinite(self.right)
        return bool(np.count_nonzero(common) >= 8 and np.all(self.right[common]-self.left[common] > .5))

    def telemetry(self, config):
        width = self.right-self.left
        width = width[np.isfinite(width)] if self.pair_valid else np.empty(0)
        def values(array):
            return [float(v) if np.isfinite(v) else None for v in array] if self.valid else []
        return dict(boundary_valid=self.valid, boundary_pair_valid=self.pair_valid,
                    left_boundary_valid=self.left_valid, right_boundary_valid=self.right_valid,
                    boundary_state=self.state,
                    left_boundary_state=self.left_state, right_boundary_state=self.right_state,
                    vehicle_width_m=config.vehicle_width_m, left_margin_m=config.left_margin_m,
                    right_margin_m=config.right_margin_m, corridor_width_m=config.width_m,
                    minimum_clearance_width_m=config.width_m,
                    boundary_width_min_dev=float(np.min(width)) if len(width) else None,
                    boundary_width_max_dev=float(np.max(width)) if len(width) else None,
                    clearance_sufficient=bool(np.min(width) >= config.width_m) if len(width) else None,
                    boundary_units='uncalibrated_bev_development_units',
                    boundary_forward_dev=self.forward.tolist() if self.valid else [],
                    left_boundary_dev=values(self.left), right_boundary_dev=values(self.right))


class BoundaryTracker:
    def __init__(self, config=None):
        self.config = config or BoundaryConfig()
        self.previous = None
        self.last_time = None
        self.last_supported_time = None
        self.side_observed_time = [None, None]
        self.pending_acquisition = [None, None]
        self.side_history = [None, None]
        self.side_measurement = [None, None]

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

    def _observe_side(self, forward, candidates, side, prediction=None, seed=None):
        if prediction is None and seed is None:
            # Try several longitudinal tracks instead of locking onto the
            # closest bright fragment at the bottom (arrows/parking crossbars).
            fits = []
            first = next((i for i, values in enumerate(candidates)
                          if any(v < -.6 if side == 0 else v > .6 for v in values)), 0)
            for start in sorted(set([first, *range(0, len(forward)//2, 5)])):
                values = np.asarray(candidates[start])
                values = values[values < -.6] if side == 0 else values[values > .6]
                for value in sorted(values, key=abs)[:4]:
                    curve, indices = self._observe_side(forward, candidates, side,
                                                        seed=(start, value))
                    if curve is not None:
                        fits.append((curve, indices))
            if fits:
                return max(fits, key=lambda fit: (len(fit[1]), np.ptp(forward[fit[1]])))
            return None, np.empty(0, dtype=int)
        chosen_i, chosen_x = [], []
        for i, values in enumerate(candidates):
            if seed is not None and i < seed[0]:
                continue
            values = np.asarray(values)
            if prediction is None:
                values = values[values < -.6] if side == 0 else values[values > .6]
                reference = chosen_x[-1] if chosen_x else seed[1]
                if len(chosen_i) >= 2:
                    slope = (chosen_x[-1]-chosen_x[-2])/(forward[chosen_i[-1]]-forward[chosen_i[-2]])
                    reference += np.clip(slope, -.5, .5)*(forward[i]-forward[chosen_i[-1]])
                gate = self.config.observation_gate_m if chosen_i else .05
            else:
                reference, gate = prediction[i], self.config.observation_gate_m
            values = values[abs(values-reference) <= gate]
            if len(values):
                chosen_i.append(i)
                chosen_x.append(float(values[np.argmin(abs(values-reference))]))
        indices, values = np.asarray(chosen_i, dtype=int), np.asarray(chosen_x)
        if len(indices) < 8 or np.ptp(forward[indices]) < 3.:
            return None, np.empty(0, dtype=int)
        keep = np.ones(len(values), bool)
        for _ in range(3):
            poly = np.polynomial.Polynomial.fit(forward[indices][keep], values[keep], 2)
            residual = abs(poly(forward[indices])-values)
            keep = residual <= min(.25, max(.12, 2.5*float(np.median(residual))))
            if np.count_nonzero(keep) < 8:
                return None, np.empty(0, dtype=int)
        indices, values = indices[keep], values[keep]
        # Do not span disconnected paint/noise clusters with a fitted curve.
        groups = np.split(np.arange(len(indices)), np.flatnonzero(np.diff(indices) > 3)+1)
        longest = max(groups, key=len)
        indices, values = indices[longest], values[longest]
        if len(indices) < 8:
            return None, np.empty(0, dtype=int)
        if np.ptp(forward[indices]) < 3.:
            return None, np.empty(0, dtype=int)
        poly = np.polynomial.Polynomial.fit(forward[indices], values, 2)
        return poly(forward), indices

    def _smooth_side(self, curve, prediction, forward, indices, history, dt):
        if prediction is None or history is None:
            return curve
        # Limit only the common observed range. Extrapolated polynomial tails
        # must not throttle the visible line's response.
        common = (forward >= max(forward[indices[0]], history[0][0])) & (
            forward <= min(forward[indices[-1]], history[0][-1]))
        if not np.any(common):
            return curve
        delta = curve-prediction
        scale = min(1., self.config.max_lateral_rate_m_s*dt/max(float(np.max(abs(delta[common]))), 1e-9))
        return prediction+delta*scale

    def update_bev(self, forward, lateral, road, lane, timestamp_s, projection=None):
        """Observe and continue each side independently during continuous input."""
        c = self.config
        projection = np.eye(3) if projection is None else projection
        empty = np.empty(0)
        def unavailable(state):
            return BoundaryEstimate(False, state, empty, empty, empty, projection)
        def reset():
            self.side_history = [None, None]
            self.side_measurement = [None, None]
            self.side_observed_time = [None, None]
            self.pending_acquisition = [None, None]
            self.previous = None

        if not np.isfinite(timestamp_s):
            reset()
            return unavailable('invalid_timestamp')
        if self.last_time is not None and timestamp_s <= self.last_time:
            reset()
            return unavailable('nonmonotonic_timestamp')
        dt = .1 if self.last_time is None else min(timestamp_s-self.last_time, .2)
        if self.last_time is not None and timestamp_s-self.last_time > c.max_missing_s:
            reset()
        self.last_time = timestamp_s
        forward, lateral = np.asarray(forward), np.asarray(lateral)
        road_present = np.count_nonzero(np.any(road, axis=1)) >= 8
        histories = list(self.side_history)
        predictions = [None if h is None else np.interp(forward, h[0], h[1]) for h in histories]
        references = [None if h is None else np.interp(forward, h[0], h[1]) for h in self.side_measurement]
        candidates = []
        for i in range(len(forward)):
            indices = np.flatnonzero(lane[i])
            runs = [r for r in np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1) if len(r)]
            values = [float(np.median(lateral[r])) for r in runs if lateral[r[-1]]-lateral[r[0]] <= .7]
            candidates.append([v for v in values if np.any(road[i, abs(lateral-v) <= .4])])
        curves, supports, states = [], [], []
        for side in (0, 1):
            curve, indices = self._observe_side(forward, candidates, side, references[side])
            curves.append(curve)
            supports.append(indices)
            states.append('observed' if curve is not None else 'unavailable')

        # Search independently of old association. Stable, longer evidence can
        # replace a stale short fragment; one side does not require the other.
        expired = [histories[i] is not None and self.side_observed_time[i] is not None and
                   timestamp_s-self.side_observed_time[i] > c.max_lane_gap_s for i in (0, 1)]
        if any(histories[i] is not None for i in (0, 1)):
            fresh = [self._observe_side(forward, candidates, i) for i in (0, 1)]
            pair_widens = False
            if all(f[0] is not None for f in fresh) and all(h is not None for h in histories):
                lo = max(h[0][0] for h in histories)
                hi = min(h[0][-1] for h in histories)
                common = (forward >= lo) & (forward <= hi)
                if np.any(common):
                    old_width = predictions[1][common]-predictions[0][common]
                    new_width = fresh[1][0][common]-fresh[0][0][common]
                    pair_widens = np.any(new_width <= .5) or np.median(new_width) > 1.25*np.median(old_width)
                    # A short, wrongly acquired pair must not permanently lock
                    # out much longer actual paint solely because of its width.
                    stronger_pair = all(len(fresh[i][1]) >= max(20, len(histories[i][0])+8)
                                        for i in (0, 1))
                    if stronger_pair and np.all(new_width > .5):
                        pair_widens = False
            for side in (0, 1):
                candidate, indices = fresh[side]
                if candidate is None or histories[side] is None:
                    self.pending_acquisition[side] = None
                    continue
                # A short noisy fragment must not outrank a continuous stripe.
                better = len(indices) >= max(16, len(supports[side])+8)
                missing = curves[side] is None and (expired[side] or len(indices) >= 16)
                if pair_widens or not (missing or better):
                    self.pending_acquisition[side] = None
                    continue
                pending = self.pending_acquisition[side]
                count = pending[1]+1 if pending is not None and np.median(abs(candidate[indices]-pending[0][indices])) < .25 else 1
                self.pending_acquisition[side] = (candidate, count)
                if count >= 3:
                    curves[side], supports[side], states[side] = candidate, indices, 'observed'
                    self.pending_acquisition[side] = None

        raw_curves = list(curves)
        for side in (0, 1):
            if curves[side] is not None:
                # Returning paint may lie on a different segmentation region.
                # Keep the displayed history as the interpolation anchor anyway.
                curves[side] = self._smooth_side(curves[side], predictions[side], forward,
                                                supports[side], histories[side], dt)
        # Missing sides use only their own history, moved by the other side's
        # observed change when available. Never invent an unseen opposite edge.
        observed_curves = list(curves)
        for side in (0, 1):
            if curves[side] is not None:
                continue
            last, history = self.side_observed_time[side], histories[side]
            # A paint/segmentation gap has no age limit. Frame discontinuities
            # above still reset tracking; startup never invents an unseen side.
            if history is None or last is None:
                continue
            other = 1-side
            delta = np.zeros(len(forward))
            if observed_curves[other] is not None and predictions[other] is not None:
                measured = supports[other]
                change = np.clip(observed_curves[other]-predictions[other],
                                 -c.max_lateral_rate_m_s*dt, c.max_lateral_rate_m_s*dt)
                # Continue endpoint motion beyond the visible side's range;
                # never copy an unconstrained polynomial extrapolation tail.
                delta = np.interp(forward, forward[measured], change[measured])
            curves[side] = predictions[side]+delta
            supports[side] = np.flatnonzero((forward >= history[0][0]) & (forward <= history[0][-1]))
            states[side] = 'inferred' if road_present else 'held'

        rendered = []
        for side in (0, 1):
            output = np.full(len(forward), np.nan)
            curve, indices = curves[side], supports[side]
            if curve is not None and len(indices):
                selected = np.arange(indices[0], indices[-1]+1)
                if road_present and states[side] == 'observed':
                    # Validate this edge against nearby pavement, independently
                    # of visibility/road coverage at the opposite edge.
                    # Validate the measurement, not the in-flight interpolation:
                    # otherwise a returning edge disappears until it has moved
                    # all the way into the newly segmented road.
                    nearby = np.array([np.any(road[i, abs(lateral-raw_curves[side][i]) <= .45]) for i in selected])
                    runs = [r for r in np.split(selected[nearby], np.flatnonzero(np.diff(selected[nearby]) > 1)+1) if len(r)]
                    selected = max(runs, key=len) if runs else np.empty(0, dtype=int)
                if len(selected) >= 8 and np.ptp(forward[selected]) >= 3.:
                    output[selected] = curve[selected]
                else:
                    states[side] = 'unavailable'
            else:
                states[side] = 'unavailable'
            rendered.append(output)

        # If fitted curves cross, suppress only the inferred/weaker conflicting
        # portion. The other observed edge must remain visible.
        common = np.isfinite(rendered[0]) & np.isfinite(rendered[1])
        crossing = np.flatnonzero(common & (rendered[1]-rendered[0] <= .5))
        if len(crossing):
            weaker = 0 if states[0] != 'observed' and states[1] == 'observed' else 1
            rendered[weaker][crossing[0]:] = np.nan
            if np.count_nonzero(np.isfinite(rendered[weaker])) < 8:
                rendered[weaker][:] = np.nan
                states[weaker] = 'unavailable'

        for side in (0, 1):
            finite = np.isfinite(rendered[side])
            if np.count_nonzero(finite) >= 8:
                self.side_history[side] = (forward[finite].copy(), rendered[side][finite].copy())
                if states[side] == 'observed':
                    self.side_observed_time[side] = timestamp_s
                    self.side_measurement[side] = (forward[finite].copy(), raw_curves[side][finite].copy())
        selected = np.isfinite(rendered[0]) | np.isfinite(rendered[1])
        if not np.any(selected):
            return unavailable('lane_unavailable' if road_present else 'road_unavailable')
        left, right = rendered[0][selected], rendered[1][selected]
        both = np.isfinite(left) & np.isfinite(right)
        state = 'partial' if not all(np.count_nonzero(np.isfinite(x)) >= 8 for x in (left, right)) else (
            'tracking' if all(s == 'observed' for s in states) else 'inferred')
        if np.count_nonzero(both) >= 8 and np.min(right[both]-left[both]) < c.width_m:
            state = 'insufficient_width'
        if not road_present:
            state = 'held'
        result = BoundaryEstimate(True, state, forward[selected].copy(), left, right, projection, *states)
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
        if not getattr(estimate, side+'_valid'):
            continue
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
