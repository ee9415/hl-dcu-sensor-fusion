"""Shared-vertex trapezoid corridor, local paint search, no metric assumptions."""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from image_boundaries import ImageBoundaryEstimate, road_motion
from lane_boundaries import BoundaryConfig, projection_for, white_lane_evidence


@dataclass(frozen=True)
class TrapezoidConfig:
    rows: tuple = (174, 154, 138, 125, 114, 105, 98, 92)
    base_left: tuple = (61.9, 79.9, 94.3, 106, 115.9, 124, 130.3, 135.7)
    base_right: tuple = (258.1, 240.1, 225.7, 214, 204.1, 196, 189.7, 184.3)
    search_radius: float = 22.
    max_search_radius: float = 40.
    correction_px: float = 5.
    road_shift_px: float = 2.
    min_paint_support: float = .55
    show_regions: bool = True

    def __post_init__(self):
        rows = np.asarray(self.rows, float)
        edges = np.asarray([self.base_left, self.base_right], float)
        if (len(rows) < 3 or edges.shape != (2, len(rows)) or
                not np.isfinite(rows).all() or not np.isfinite(edges).all() or
                np.any(np.diff(rows) >= -3) or rows[0] > 178 or rows[-1] < 70 or
                np.any(edges[0] < 1) or np.any(edges[1] > 318) or
                np.any(edges[1]-edges[0] < 12)):
            raise ValueError('Invalid shared-vertex trapezoid geometry')
        limits = (self.search_radius, self.max_search_radius, self.correction_px, self.road_shift_px)
        if not all(np.isfinite(x) and x > 0 for x in limits):
            raise ValueError('Search and correction limits must be positive')
        if not 0 < self.min_paint_support <= 1 or self.max_search_radius < self.search_radius:
            raise ValueError('Invalid paint support/search settings')

    @classmethod
    def load(cls, path=None):
        path = Path(path) if path else Path(__file__).with_name('trapezoid_boundaries.json')
        return cls(**json.loads(path.read_text())) if path.exists() else cls()


@dataclass
class TrapezoidEstimate(ImageBoundaryEstimate):
    vertices: object = None  # (2, nodes, 2), shared between neighboring regions
    node_states: object = None
    node_confidence: object = None
    region_count: int = 0
    show_regions: bool = False

    def telemetry(self, config=None):
        result = super().telemetry(config)
        result.update(boundary_model='trapezoid_segments', segment_units='image_pixels',
                      segment_distance_calibrated=False, segment_length_m=None,
                      boundary_region_count=self.region_count,
                      boundary_region_vertices=self.vertices.tolist(),
                      boundary_node_states=self.node_states,
                      boundary_node_confidence=self.node_confidence.tolist())
        return result


def resize_image(image, height=180, width=320):
    if image.shape[:2] == (height, width):
        return image
    yy = np.minimum(((np.arange(height)+.5)*image.shape[0]/height).astype(int), image.shape[0]-1)
    xx = np.minimum(((np.arange(width)+.5)*image.shape[1]/width).astype(int), image.shape[1]-1)
    return image[yy[:, None], xx[None, :]]


def interpolate_vertices(rows, values, sample_rows):
    """Shape-preserving cubic interpolation; no extrapolation beyond end vertices."""
    x, y = rows[::-1], values[::-1]
    h = np.diff(x)
    slopes = np.diff(y)/h
    tangent = np.zeros(len(x))
    tangent[0], tangent[-1] = slopes[0], slopes[-1]
    for i in range(1, len(x)-1):
        if slopes[i-1]*slopes[i] > 0:
            w1, w2 = 2*h[i]+h[i-1], h[i]+2*h[i-1]
            tangent[i] = (w1+w2)/(w1/slopes[i-1]+w2/slopes[i])
    j = np.clip(np.searchsorted(x, sample_rows)-1, 0, len(x)-2)
    t = np.clip((sample_rows-x[j])/h[j], 0, 1)
    return ((2*t**3-3*t**2+1)*y[j] + (t**3-2*t**2+t)*h[j]*tangent[j] +
            (-2*t**3+3*t**2)*y[j+1] + (t**3-t**2)*h[j]*tangent[j+1])


class TrapezoidBoundaryTracker:
    def __init__(self, config=None, segment_config=None):
        self.config = config or BoundaryConfig.load()  # BEV display only
        self.settings = segment_config or TrapezoidConfig.load()
        self.rows = np.asarray(self.settings.rows, float)
        self.base = np.asarray([self.settings.base_left, self.settings.base_right], float)
        self.last_candidate_count = 0
        self._reset()

    def _reset(self):
        self.edges = self.base.copy()
        self.width = self.base[1]-self.base[0]
        self.scores = np.zeros_like(self.edges)
        self.ages = np.zeros_like(self.edges)
        self.pending = np.full_like(self.edges, np.nan)
        self.hits = np.zeros_like(self.edges, int)
        self.previous = None
        self.last_time = None
        self.velocity = np.zeros((2, 3))
        self.visible_nodes = len(self.rows)

    def _evidence(self, rgb, road, lane, detections):
        image = resize_image(rgb).astype(np.float32)
        road = resize_image(road) > 0
        lane = resize_image(lane)
        bright = image.mean(axis=2)
        paint = white_lane_evidence(image, lane) > 0
        # Local brightness prevents white vehicles/shadows from setting a global threshold.
        for y in range(int(self.rows[-1])-7, 180):
            samples = bright[y, road[y]]
            threshold = max(110., float(np.median(samples))+16.) if len(samples) else 125.
            paint[y] &= bright[y] >= threshold
        padded = np.pad(road, ((3, 3), (5, 5)))
        allowed = np.zeros_like(road)
        for dy in (0, 3, 6):
            for dx in (0, 5, 10):
                allowed |= padded[dy:dy+180, dx:dx+320]
        objects = np.zeros_like(road)
        for box in detections:
            a, b = np.clip([int(box.xmin*320), int(np.ceil(box.xmax*320))], 0, 320)
            c, d = np.clip([int(box.ymin*180), int(np.ceil(box.ymax*180))], 0, 180)
            objects[c:d, a:b] = True
        paint &= allowed & ~objects
        # Suppress broad horizontal paint (parking bars / stop lines).
        for y in range(int(self.rows[-1])-7, 180):
            xs = np.flatnonzero(paint[y])
            if len(xs):
                for group in np.split(xs, np.flatnonzero(np.diff(xs) > 1)+1):
                    if len(group) > 10:
                        paint[y, group] = False
        return image[::2, ::2].mean(axis=2), road, allowed & ~objects, paint

    def _predict(self, matrix):
        result = []
        for edge in self.edges:
            points = np.column_stack((edge, self.rows, np.ones(len(edge)))) @ matrix.T
            order = np.argsort(points[:, 1])
            result.append(np.interp(self.rows, points[order, 1], points[order, 0]))
        result = np.asarray(result)
        # Local flow is not a license to collapse/cross or expand the corridor.
        gap = result[1]-result[0]
        if (np.any(result < 1) or np.any(result > 318) or
                np.any(gap < self.width*.7) or np.any(gap > self.width*1.3)):
            return self.edges.copy(), False
        return result, True

    def _road_coverage(self, road, left, right, y):
        xs = np.rint(np.linspace(left+(right-left)*.15, right-(right-left)*.15, 15)).astype(int)
        ys = np.clip(np.arange(int(y)-2, int(y)+3), 0, 179)
        return float(road[ys[:, None], np.clip(xs, 0, 319)].mean())

    def _paint_candidate(self, paint, side, k, predicted, expected):
        y = self.rows[k]
        lower, upper = max(int(self.rows[-1]), int(y)-7), min(178, int(y)+7)
        ys = np.arange(lower, upper+1, 2)
        if k == 0:
            slope = (predicted[side, 1]-predicted[side, 0])/(self.rows[1]-self.rows[0])
        else:
            slope = (predicted[side, k]-predicted[side, k-1])/(y-self.rows[k-1])
        radius = min(self.settings.max_search_radius, self.settings.search_radius+2*self.ages[side, k])
        offsets = np.arange(-radius, radius+1, 2.)
        centers = expected+offsets
        midpoint = predicted[:, k].mean()
        # Side identity is tied to the shared corridor, not independent polynomial fits.
        legal = (centers < midpoint-4) if side == 0 else (centers > midpoint+4)
        tracks = np.rint(centers[:, None]+np.clip(slope, -2, 2)*(ys-y)).astype(int)
        hits = np.zeros(tracks.shape, bool)
        for dx in (-1, 0, 1):
            xx = tracks+dx
            hits |= (xx >= 0) & (xx < 320) & paint[ys[None, :], np.clip(xx, 0, 319)]
        support = hits.mean(axis=1)
        legal &= (support >= self.settings.min_paint_support) & (hits.sum(axis=1) >= 4)
        self.last_candidate_count += len(centers)
        if not legal.any():
            return None
        rank = support-.12*abs(offsets)/max(radius, 1)
        rank[~legal] = -np.inf
        best = int(np.argmax(rank))
        return float(centers[best]), float(support[best])

    def update(self, road, lane, timestamp_s, rgb=None, detections=()):
        if rgb is None:
            raise ValueError('Trapezoid tracking requires RGB')
        dt = .1 if self.last_time is None else timestamp_s-self.last_time
        if not np.isfinite(timestamp_s) or dt <= 0 or dt > 1:
            self._reset()
            dt = .1
        dt = float(np.clip(dt, .01, .2))
        gray, road, allowed, paint = self._evidence(rgb, road, lane, detections)
        motion, motion_score = road_motion(self.previous, gray, allowed[::2, ::2])
        if self.previous is not None and motion_score == 0 and np.mean(abs(gray-self.previous)) > 45:
            self._reset()
        identity = np.eye(2, 3)
        if motion_score:
            alpha = 1-np.exp(-dt/.25)
            self.velocity = (1-alpha)*self.velocity+alpha*(motion-identity)/dt
        else:
            self.velocity *= np.exp(-dt/.6)
            motion = identity+self.velocity*dt
        predicted, motion_usable = self._predict(motion)
        self.previous, self.last_time = gray, timestamp_s if np.isfinite(timestamp_s) else None
        self.ages += dt
        self.scores *= np.exp(-dt*.8)
        self.last_candidate_count = 0
        target = predicted.copy()
        measured = np.full_like(target, np.nan)
        strength = np.zeros_like(target)
        states = np.full(target.shape, 'predicted' if motion_score and motion_usable else 'held', dtype=object)
        # Road guidance moves each cross-section as a pair; it never sets its width.
        for k, y in enumerate(self.rows):
            if self._road_coverage(road, *predicted[:, k], y) < .65:
                shifts = np.arange(-18., 19., 3.)
                utility = [self._road_coverage(road, *(predicted[:, k]+shift), y)-.003*abs(shift)
                           for shift in shifts]
                shift = shifts[int(np.argmax(utility))]
                target[:, k] += np.clip(shift, -self.settings.road_shift_px, self.settings.road_shift_px)
            for side in (0, 1):
                # Carry local bend into the next trapezoid, without moving all distant nodes.
                carry = 0. if k == 0 else .5*(target[side, k-1]-predicted[side, k-1])
                candidate = self._paint_candidate(paint, side, k, predicted, target[side, k]+carry)
                if candidate is None:
                    self.hits[side, k] = 0
                    continue
                value, score = candidate
                far = abs(value-predicted[side, k]) > 8
                if far and self.last_time is not None:
                    self.hits[side, k] = self.hits[side, k]+1 if abs(value-self.pending[side, k]) < 6 else 1
                    self.pending[side, k] = value
                    if self.hits[side, k] < 3:
                        continue
                measured[side, k], strength[side, k] = value, score
                target[side, k] = value
            found = np.isfinite(measured[:, k])
            if found.sum() == 1:
                side = int(np.flatnonzero(found)[0])
                target[1-side, k] = target[side, k]+self.width[k]*(1 if side == 0 else -1)
                states[1-side, k] = 'inferred'
            elif found.all():
                gap = target[1, k]-target[0, k]
                if not self.width[k]*.65 <= gap <= self.width[k]*1.4:
                    loser = int(strength[1, k] < strength[0, k])
                    measured[loser, k] = np.nan
                    target[loser, k] = target[1-loser, k]+self.width[k]*(-1 if loser == 0 else 1)
                    states[loser, k] = 'inferred'

        # Smooth corrections, not old geometry: no evidence must not gradually
        # straighten a held curve or change its learned perspective width.
        smooth = target.copy()
        change = target-predicted
        for k in range(1, len(self.rows)-1):
            t = (self.rows[k]-self.rows[k-1])/(self.rows[k+1]-self.rows[k-1])
            chord = change[:, k-1]*(1-t)+change[:, k+1]*t
            smooth[:, k] = predicted[:, k]+.85*change[:, k]+.15*chord
        if np.isfinite(measured).any() or np.max(abs(change)) > .01:
            # A shared corridor must bend coherently, not zigzag around isolated
            # bright pixels. Penalize changes in heading, leaving a straight
            # corridor in the nullspace. Width is not part of this translation.
            bend = np.zeros((len(self.rows)-2, len(self.rows)))
            for k in range(1, len(self.rows)-1):
                near, far = self.rows[k-1]-self.rows[k], self.rows[k]-self.rows[k+1]
                bend[k-1, k-1:k+2] = [10/near, -10/near-10/far, 10/far]
            middle = np.linalg.solve(np.eye(len(self.rows))+1.5*bend.T@bend, smooth.mean(axis=0))
            gap = smooth[1]-smooth[0]
            smooth = np.array([middle-gap/2, middle+gap/2])
        delta = np.clip(smooth-predicted, -self.settings.correction_px, self.settings.correction_px)
        edges = predicted+delta
        # Preserve a reasonable perspective width and ordering, even during complete loss.
        gap = np.clip(edges[1]-edges[0], self.width*.8, self.width*1.2)
        middle = edges.mean(axis=0)
        middle = np.clip(middle, 1+gap/2, 318-gap/2)
        edges = np.array([middle-gap/2, middle+gap/2])
        residual = abs(edges-measured)
        found = np.isfinite(measured)
        self.scores[found] = strength[found]*np.exp(-residual[found]/5)
        self.ages[found] = 0
        states[found & (residual <= 3)] = 'observed'
        states[found & (residual > 3)] = 'aligning'
        inferred = states == 'inferred'
        self.scores[inferred] = np.minimum(self.scores[inferred], .15)
        for k in range(len(self.rows)):
            if np.all(states[:, k] == 'observed'):
                self.width[k] = .95*self.width[k]+.05*(edges[1, k]-edges[0, k])
        self.edges = edges
        # Stop at the last contiguous road-supported cross-section. If segmentation
        # disappears entirely, retain the previous extent instead of growing skyward.
        if road.any():
            supported = [self._road_coverage(road, *edges[:, k], y) >= .3 for k, y in enumerate(self.rows)]
            visible = 0
            for ok in supported:
                if not ok:
                    break
                visible += 1
            self.visible_nodes = visible
        count = self.visible_nodes
        sample_rows = np.linspace(self.rows[0], self.rows[-1], 48)
        curves, supports, side_states, confidences, residuals = [], [], [], [], []
        for side in (0, 1):
            values = interpolate_vertices(self.rows, edges[side], sample_rows)
            visible = sample_rows >= self.rows[max(0, count-1)] if count >= 2 else np.zeros(48, bool)
            values[~visible] = np.nan
            curves.append(np.column_stack((values, sample_rows)))
            interval = np.clip(np.searchsorted(-self.rows, -sample_rows)-1, 0, len(self.rows)-2)
            observed = states[side] == 'observed'
            support = observed[interval] & observed[interval+1] & visible
            supports.append(support)
            relevant = states[side, :count]
            if count < 2:
                state = 'unavailable'
            elif not np.any(relevant == 'aligning') and np.count_nonzero(relevant == 'observed') >= max(2, int(np.ceil(.6*count))):
                state = 'observed'
            elif np.any(relevant == 'aligning'):
                state = 'aligning'
            elif np.any(relevant == 'inferred'):
                state = 'inferred'
            elif np.any(relevant == 'observed'):
                state = 'partial'
            else:
                state = 'predicted' if motion_score and motion_usable else 'held'
            side_states.append(state)
            confidences.append(float(np.mean(self.scores[side, :count])) if count else 0.)
            r = residual[side, :count]
            residuals.append(float(np.max(r[np.isfinite(r)])) if np.isfinite(r).any() else None)
        reference = 'both' if found[0].any() and found[1].any() else 'left' if found[0].any() else 'right' if found[1].any() else 'none'
        return TrapezoidEstimate(
            *curves, projection_for((180, 320), self.config), *side_states, *confidences,
            motion_score, motion, 'tracking' if found.any() else 'predicted', reference,
            *supports, *residuals, *[float(np.max(a)) for a in self.ages],
            vertices=np.stack([np.column_stack((edge, self.rows)) for edge in edges]),
            node_states=states.tolist(), node_confidence=self.scores.copy(),
            region_count=max(0, count-1), show_regions=self.settings.show_regions)
