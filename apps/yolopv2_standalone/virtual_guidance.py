"""Optional image-space corridor and quadratic path; never feeds lane tracking.

All distances/rates are pixels in the 320x180 image, not calibrated meters.
The bottom-center start is an installation proxy, not a measured vehicle pose.
"""
from dataclasses import dataclass, fields
import json
import os
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class GuidanceConfig:
    enabled: bool = False
    confidence: float = .55
    confirm_frames: int = 3
    target_y: float = .50
    fallback_y: float = .75
    start_x: float = .50
    start_y: float = .995
    boundary_rate: float = 40.
    path_rate: float = 40.
    smoothing_seconds: float = .25
    road_margin: float = 3.
    initial_width_fraction: float = .80

    def __post_init__(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if f.name == 'enabled':
                if not isinstance(v, bool):
                    raise ValueError('enabled must be boolean')
            elif isinstance(v, bool) or not np.isfinite(v) or v <= 0:
                raise ValueError(f'{f.name} must be positive and finite')
        if (not 0 < self.confidence <= 1 or
                not 0 < self.target_y <= self.fallback_y < self.start_y <= 1 or
                not 0 < self.start_x < 1 or not 0 < self.initial_width_fraction <= 1 or
                not isinstance(self.confirm_frames, int)):
            raise ValueError('Invalid guidance geometry or confirmation count')

    @classmethod
    def load(cls, path=None):
        p = Path(path or Path(__file__).with_name('virtual_guidance.json'))
        values = json.loads(p.read_text(encoding='utf8')) if p.exists() else {}
        override = os.environ.get('OAK_VIRTUAL_GUIDANCE')
        if override is not None:
            if override not in ('0', '1'):
                raise ValueError('OAK_VIRTUAL_GUIDANCE must be 0 or 1')
            values['enabled'] = override == '1'
        return cls(**values)


@dataclass
class GuidanceEstimate:
    valid: bool = False
    reason: str = 'disabled'
    source: str = 'none'
    left: object = None
    right: object = None
    path: object = None
    coefficients: object = None
    target: object = None
    real: tuple = (False, False)
    width_source: str = 'none'
    rate_limited: bool = False
    boundary_state: str = 'current'
    road_supported: bool = True
    display_path: object = None

    @property
    def visible_path(self):
        return self.path if self.valid else self.display_path

    @property
    def boundaries_available(self):
        return self.left is not None and self.right is not None

    def telemetry(self):
        def points(p):
            return [] if p is None else np.asarray(p).tolist()
        return dict(guidance_valid=self.valid, guidance_reason=self.reason,
                    guidance_boundaries_available=self.boundaries_available,
                    guidance_boundary_state=self.boundary_state if self.boundaries_available else 'none',
                    guidance_boundary_road_supported=self.road_supported and self.boundaries_available,
                    guidance_path_display_available=self.visible_path is not None,
                    guidance_path_display_state=('current' if self.valid else 'estimated' if self.display_path is not None else 'none'),
                    guidance_display_path_image=points(self.visible_path),
                    guidance_source=self.source, guidance_units='image_pixels_320x180',
                    guidance_start_kind='configured_image_proxy',
                    guidance_left_real=self.real[0], guidance_right_real=self.real[1],
                    guidance_width_source=self.width_source,
                    guidance_rate_limited=self.rate_limited,
                    guidance_left_image=points(self.left),
                    guidance_right_image=points(self.right),
                    guidance_path_image=points(self.path),
                    guidance_target=points(self.target),
                    guidance_coefficients=points(self.coefficients),
                    guidance_polynomial='x=c0+c1*y+c2*y^2')


def _runs(row):
    edges = np.diff(np.r_[False, row, False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)-1))


def road_corridor(mask, start_x):
    """Follow one connected run from near the image bottom; never span branches.

    A run must overlap the preceding row (8-connectivity). At forks choose the
    closest midpoint, rather than averaging separate paved islands together.
    """
    h, w = mask.shape
    left = np.full(h, np.nan)
    right = left.copy()
    prior = None
    for y in range(h-1, int(h*.40)-1, -1):
        runs = [(a, b) for a, b in _runs(mask[y]) if b-a >= 12]
        if prior is None:
            if y < int(h*.85):
                break
            runs = [(a, b) for a, b in runs if a <= start_x <= b]
        else:
            runs = [(a, b) for a, b in runs if a <= prior[1]+1 and b >= prior[0]-1]
        if not runs:
            if prior is not None:
                break
            continue
        reference = start_x if prior is None else sum(prior)*.5
        prior = min(runs, key=lambda r: abs(sum(r)*.5-reference))
        left[y], right[y] = prior
    return left, right


def _fit(y, x):
    # Normalizing y keeps polynomial fitting well-conditioned.
    return np.polynomial.polynomial.polyfit((y-90.)/90., x, 2)


def _eval(c, y):
    return np.polynomial.polynomial.polyval((y-90.)/90., c)


def _peak(c, y):
    """Exact max |quadratic| on an interval, including between-pixel extrema."""
    lo, hi = float(np.min(y)), float(np.max(y))
    locations = [lo, hi]
    if abs(c[2]) > 1e-15:
        vertex = 90.-90.*c[1]/(2*c[2])
        if lo < vertex < hi:
            locations.append(vertex)
    return float(np.max(abs(_eval(c, np.array(locations)))))


def _path_from_corridor(pair, y, start):
    """Single quadratic with fixed start and corridor-midpoint target."""
    middle=.5*(pair[0]+pair[1])
    endpoint=float(_eval(middle,y[0]))
    span=start[1]-y[0]
    k=float(np.clip(middle[2]*(span/90.)**2,-.9*abs(endpoint-start[0]),.9*abs(endpoint-start[0])))
    t=(start[1]-y)/span
    return _fit(y,start[0]+(endpoint-start[0]-k)*t+k*t*t)


class VirtualGuidance:
    def __init__(self, config=None):
        self.config = config or GuidanceConfig()
        self.reset()

    def reset(self):
        self.last_time = None
        self.hits = [0, 0]
        self.last_observations = [None, None]
        self.corridor = None
        self.path_coefficient = None
        self.width = None
        self.last_source = None
        self.target_y = None
        self.display_corridor = None
        self.display_y = None
        self.display_path_coefficient = None

    def _boundary_fallback(self, reason, road_left, road_right, start, dt):
        """A missing/invalid path must not erase independently displayable edges.

        Prefer a current road-only pair. If there is no feasible pair, retain
        the previous curves as explicitly unverified display estimates only.
        Display-only path estimates are separate from valid path outputs.
        """
        cfg = self.config
        candidate = None
        ys = None
        for top in range(round(179*cfg.target_y), round(179*cfg.fallback_y)+1):
            y = np.linspace(top, start[1], int(start[1])-top+1)
            rows = y.astype(int)
            known = np.isfinite(road_left[rows])
            if known.sum() < 12 or known.mean() < .9 or not known[0]:
                continue
            pair = [_fit(y[known], road_left[rows][known]+cfg.road_margin),
                    _fit(y[known], road_right[rows][known]-cfg.road_margin)]
            pair[0][0] += max(0.,float(np.max(road_left[rows][known]+cfg.road_margin-_eval(pair[0],y[known]))))
            pair[1][0] -= max(0.,float(np.max(_eval(pair[1],y[known])-road_right[rows][known]+cfg.road_margin)))
            left, right = [_eval(c,y) for c in pair]
            if np.all(right-left >= 12) and np.min(left)>=0 and np.max(right)<=319:
                candidate, ys = pair, y
                break
        state = 'road_fallback'
        supported = candidate is not None
        limited = False
        if candidate is None:
            if self.display_corridor is None:
                return GuidanceEstimate(reason=reason,boundary_state='none',road_supported=False)
            candidate = [c.copy() for c in self.display_corridor]
            ys = self.display_y.copy()
            state = 'held_unverified'
            display_path_c=(self.display_path_coefficient.copy() if self.display_path_coefficient is not None
                            else _path_from_corridor(candidate,ys,start))
        elif self.display_corridor is not None:
            # Keep the existing display domain while estimating through a loss.
            # New road curves may be extrapolated for DISPLAY ONLY; never mark
            # such a path as validated. This also prevents a target-height jump.
            ys=self.display_y.copy()
            desired_path=_path_from_corridor(candidate,ys,start)
            alpha=1-np.exp(-dt/cfg.smoothing_seconds)
            for old,new in zip(self.display_corridor,candidate):
                delta=_peak(new-old,ys)
                alpha=min(alpha,cfg.boundary_rate*dt/max(delta,1e-9))
                limited |= delta>cfg.boundary_rate*dt
            if self.display_path_coefficient is not None:
                delta=_peak(desired_path-self.display_path_coefficient,ys)
                alpha=min(alpha,cfg.path_rate*dt/max(delta,1e-9))
                limited |= delta>cfg.path_rate*dt
            candidate=[old+alpha*(new-old) for old,new in zip(self.display_corridor,candidate)]
            display_path_c=(self.display_path_coefficient+alpha*(desired_path-self.display_path_coefficient)
                            if self.display_path_coefficient is not None else _path_from_corridor(candidate,ys,start))
            rows=ys.astype(int);known=np.isfinite(road_left[rows])
            supported=bool(known.mean()>=.9 and
                np.all(_eval(candidate[0],ys[known])>=road_left[rows][known]-3) and
                np.all(_eval(candidate[1],ys[known])<=road_right[rows][known]+3))
            if not supported:
                state='transition_unverified'
        else:
            display_path_c=_path_from_corridor(candidate,ys,start)
        self.display_corridor=[c.copy() for c in candidate]
        self.display_y=ys.copy()
        self.display_path_coefficient=display_path_c.copy()
        display_path=np.column_stack((_eval(display_path_c,ys),ys))[::-1]
        return GuidanceEstimate(reason=reason,source='road' if supported else 'held',
            left=np.column_stack((_eval(candidate[0],ys),ys)),
            right=np.column_stack((_eval(candidate[1],ys),ys)),
            real=(False,False),width_source='display_fallback',rate_limited=limited,
            boundary_state=state,road_supported=supported,display_path=display_path)

    def _observed(self, boundaries, side, dt):
        name = ('left', 'right')[side]
        p = np.asarray(getattr(boundaries, 'image_'+name), float)
        support = getattr(boundaries, name+'_support', None)
        good = np.isfinite(p).all(axis=1) if p.ndim == 2 else np.zeros(0, bool)
        if support is not None:
            good &= np.asarray(support, bool)
        quality = (getattr(boundaries, name+'_state') == 'observed' and
                   getattr(boundaries, name+'_confidence') >= self.config.confidence and
                   not getattr(boundaries, 'restored_pair', False) and good.sum() >= 8)
        c = None
        if quality and np.ptp(p[good, 1]) >= 16:
            c = _fit(p[good, 1], p[good, 0])
            old = self.last_observations[side]
            # A discontinuous candidate must confirm again, even if labeled observed.
            if old is not None and np.max(abs(_eval(c-old, p[good, 1]))) > max(8., dt*80):
                self.hits[side] = 0
            self.hits[side] += 1
            self.last_observations[side] = c
        else:
            self.hits[side] = 0
            self.last_observations[side] = None
        return c if self.hits[side] >= self.config.confirm_frames else None

    def update(self, road, boundaries, timestamp_s):
        cfg = self.config
        if not cfg.enabled:
            return GuidanceEstimate()
        road = np.asarray(road)
        if road.shape != (180, 320):
            raise ValueError('Guidance requires the unletterboxed 320x180 mask')
        if not np.isfinite(timestamp_s):
            self.reset()
            return GuidanceEstimate(reason='invalid_timestamp')
        dt = .1 if self.last_time is None else timestamp_s-self.last_time
        if dt <= 0 or dt > 1:
            self.reset()
            dt = .1
        self.last_time = timestamp_s
        # Never grant a larger movement allowance after a stalled callback.
        dt = min(dt, .2)
        start = np.array([319*cfg.start_x, 179*cfg.start_y])
        road_left, road_right = road_corridor(road > 0, start[0])
        available = np.flatnonzero(np.isfinite(road_left) &
                                   (road_right-road_left >= 24+2*cfg.road_margin) &
                                   (np.arange(180) >= round(179*cfg.target_y)) &
                                   (np.arange(180) <= round(179*cfg.fallback_y)))
        real_c = [self._observed(boundaries, s, dt) for s in (0, 1)]
        real = tuple(c is not None for c in real_c)
        source = ('both' if all(real) else 'left' if real[0] else 'right' if real[1] else 'road')
        def invalid(reason):
            # Clear path validity while preserving the independent boundary display.
            self.corridor = None
            self.path_coefficient = None
            return self._boundary_fallback(reason,road_left,road_right,start,dt)
        if not len(available):
            return invalid('no_connected_road_at_target')
        # Keep a still-valid fallback height until a source reset. This avoids
        # repeatedly dropping/restarting a path for one-row mask fluctuations.
        target_y = int(self.target_y if self.target_y in available else available[0])
        y = np.linspace(target_y, start[1], int(start[1])-target_y+1)
        rows = y.astype(int)
        known = np.isfinite(road_left[rows])
        if known.sum() < 12 or known.mean() < .9:
            return invalid('insufficient_connected_road')
        margin = cfg.road_margin
        # Segmentation can exclude the paint itself. Allow a small boundary
        # tolerance, but do not extrapolate a real curve across a different road.
        for s, c in enumerate(real_c):
            if c is not None:
                values = _eval(c, y[known])
                if np.any(values < road_left[rows][known]-8) or np.any(values > road_right[rows][known]+8):
                    real_c[s] = None
        real = tuple(c is not None for c in real_c)
        source = ('both' if all(real) else 'left' if real[0] else 'right' if real[1] else 'road')
        road_c = [_fit(y[known], road_left[rows][known]+margin),
                  _fit(y[known], road_right[rows][known]-margin)]
        # Keep fitted road edges inside the connected area with a single
        # coefficient shift, rather than clipping individual curve samples.
        road_c[0][0] += max(0., float(np.max(road_left[rows][known]+margin-_eval(road_c[0], y[known]))))
        road_c[1][0] -= max(0., float(np.max(_eval(road_c[1], y[known])-road_right[rows][known]+margin)))
        width_source = 'road_fraction'
        candidate = [None, None]
        if all(real):
            candidate = real_c
            width_source = 'observed_pair'
        elif any(real):
            side = 0 if real[0] else 1
            if self.width is not None:
                width = self.width
                width_source = 'learned_pair'
            else:
                width = (road_c[1]-road_c[0])*cfg.initial_width_fraction
            # Reduce a guessed/learned image width as one polynomial scale
            # when the visible paved corridor narrows; never clip its samples.
            values = _eval(real_c[side], y[known])
            widths = _eval(width, y[known])
            room = (road_right[rows][known]-margin-values if side == 0
                    else values-road_left[rows][known]-margin)
            if np.any(widths <= 0) or np.min(room) < 12:
                return invalid('insufficient_width_from_real_boundary')
            width = width*max(0., min(1., float(np.min(room/widths))))
            candidate[side] = real_c[side]
            candidate[1-side] = real_c[side]+width*(1 if side == 0 else -1)
        else:
            candidate = road_c
        endpoint = .5*(_eval(candidate[0], target_y)+_eval(candidate[1], target_y))
        if source == 'road':
            # The agreed no-lane target is the actual run midpoint at y=50%,
            # not the midpoint of a polynomial approximation to jagged road edges.
            endpoint = .5*(road_left[target_y]+road_right[target_y])
            shift = endpoint-.5*(_eval(candidate[0], target_y)+_eval(candidate[1], target_y))
            candidate[0][0] += shift
            candidate[1][0] += shift
        mid = .5*(candidate[0]+candidate[1])
        span = start[1]-target_y
        k = float(np.clip(mid[2]*(span/90.)**2,
                          -.9*abs(endpoint-start[0]), .9*abs(endpoint-start[0])))
        # x(t)=xs+(xe-xs-k)t+k*t^2, t=(ys-y)/span. One quadratic, no S.
        def make_path(target, curvature):
            t = (start[1]-y)/span
            x = start[0]+(target-start[0]-curvature)*t+curvature*t*t
            return _fit(y, x)
        path_c = make_path(endpoint, k)
        # Use ONE interpolation fraction for both edges AND the path. Their
        # target midpoint identity then survives temporal smoothing exactly.
        limited = False
        old_corridor=self.corridor if self.corridor is not None else self.display_corridor
        old_path=(self.display_path_coefficient if self.display_y is not None and
                  np.isclose(target_y,self.display_y[0]) else None)
        if old_corridor is not None:
            alpha = 1-np.exp(-dt/cfg.smoothing_seconds)
            changes=[(old_corridor[0],candidate[0],cfg.boundary_rate),
                     (old_corridor[1],candidate[1],cfg.boundary_rate)]
            if old_path is not None:
                changes.append((old_path,path_c,cfg.path_rate))
            for old,new,rate in changes:
                delta = _peak(new-old, y)
                alpha = min(alpha, rate*dt/max(delta, 1e-9))
                limited |= delta > rate*dt
            candidate = [old+alpha*(new-old) for old,new in zip(old_corridor,candidate)]
            if old_path is not None:
                path_c=old_path+alpha*(path_c-old_path)
            else:
                middle=.5*(candidate[0]+candidate[1])
                endpoint=float(_eval(middle,target_y))
                k=float(np.clip(middle[2]*(span/90.)**2,-.9*abs(endpoint-start[0]),.9*abs(endpoint-start[0])))
                path_c=make_path(endpoint,k)
        left, right = [_eval(c, y) for c in candidate]
        if np.any(right-left < 12) or np.any(left < 0) or np.any(right > 319):
            return invalid('invalid_corridor_geometry')
        if (np.any(left[known] < road_left[rows][known]-(8 if real[0] else 3)) or
                np.any(right[known] > road_right[rows][known]+(8 if real[1] else 3))):
            return invalid('corridor_outside_road')
        x = _eval(path_c, y)
        # Every candidate and temporal blend is checked; never clip samples,
        # because pointwise clipping destroys the single quadratic model.
        interior = known & (y < start[1]-8)
        if (np.any(x < 0) or np.any(x > 319) or
                np.any(x[interior] < np.maximum(left[interior], road_left[rows][interior])) or
                np.any(x[interior] > np.minimum(right[interior], road_right[rows][interior]))):
            return invalid('path_outside_corridor')
        self.corridor = [c.copy() for c in candidate]
        self.path_coefficient = path_c.copy()
        self.last_source = source
        self.target_y = target_y
        self.display_corridor=[c.copy() for c in candidate]
        self.display_y=y.copy()
        self.display_path_coefficient=path_c.copy()
        if all(real):
            learned = real_c[1]-real_c[0]
            if np.all(_eval(learned, y) >= 12):
                self.width = learned.copy() if self.width is None else .95*self.width+.05*learned
        # Convert normalized-y coefficients into image-y polynomial coefficients.
        c0, c1, c2 = path_c
        pixel_c = np.array([c0-c1+c2, (c1-2*c2)/90., c2/8100.])
        points = np.column_stack((x, y))[::-1]
        return GuidanceEstimate(True, 'ok', source, np.column_stack((left, y)),
                                np.column_stack((right, y)), points, pixel_c,
                                points[-1].copy(), real, width_source, limited)


def draw_guidance(frame, estimate, mask_size=(320, 180), projection=None,show_boundaries=True):
    """BGR: white actual boundary, orange dashed virtual boundary, cyan path."""
    if not estimate.boundaries_available:
        return
    from lane_boundaries import draw_polyline, transform
    scale = np.array([frame.shape[1]/mask_size[0], frame.shape[0]/mask_size[1]])
    def mapped(p):
        return p*scale if projection is None else transform(p, np.linalg.inv(projection))
    for p, real in (zip((estimate.left, estimate.right), estimate.real) if show_boundaries else ()):
        color=(255,255,255) if real else ((0,150,255) if estimate.road_supported else (0,90,160))
        draw_polyline(frame, mapped(p), color,
                      thickness=2, dashed=not real)
    path=estimate.visible_path
    if path is None:
        return
    color=(255,255,0) if estimate.valid else (180,180,0)
    draw_polyline(frame, mapped(path), color, thickness=2,dashed=not estimate.valid)
    for p in (path[0],path[-1]):
        q = mapped(np.array([p]))[0]
        if np.isfinite(q).all():
            x, y = np.rint(q).astype(int)
            if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                frame[max(0, y-4):y+5, max(0, x-4):x+5] = color
