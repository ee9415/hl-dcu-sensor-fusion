"""Manual intersection preview in uncalibrated BEV units; NEVER vehicle control.

No intersection detection, exit-lane inference or ego-motion is implied.
Paths have x(s), y(s) samples rather than polynomial y(x) coefficients.
"""
import json
import os
from pathlib import Path
import time
import numpy as np
from centerline import _draw_polyline

COMMAND_DIR = Path('/app/turn_runtime')


def road_samples(estimate):
    x = np.linspace(*estimate.forward_range_m, 1000)
    y = np.polynomial.polynomial.polyval(x, estimate.coefficients)
    distance = np.r_[0., np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    if distance[-1] < 12:
        raise ValueError('visible centerline too short')
    s = np.linspace(0, min(24., distance[-1]), 241)
    return np.column_stack((np.interp(s, distance, x), np.interp(s, distance, y)))


def turn_samples(estimate, direction):
    """Preserve the first sixth of the road path and match tangent/curvature.

    Heading is a cubic Hermite function of arc length: its initial derivative
    equals road curvature, final derivative is zero, final heading is +/-90 deg.
    Integrating cos/sin makes the path parametric, not a cubic y(x).
    """
    base = road_samples(estimate)
    join = 40
    x = base[join, 0]
    c = estimate.coefficients
    slope = c[1] + 2*c[2]*x + 3*c[3]*x*x
    curvature = (2*c[2]+6*c[3]*x)/(1+slope*slope)**1.5
    length = float(np.sum(np.linalg.norm(np.diff(base[join:], axis=0), axis=1)))
    u = np.linspace(0, 1, 2001)
    theta = (np.arctan(slope) + curvature*length*u*(1-u)**2
             + (-1 if direction == 'left' else 1)*np.pi/2*(3*u*u-2*u**3))
    tangents = np.column_stack((np.cos(theta), np.sin(theta)))
    tail = np.vstack((base[join], base[join] + np.cumsum(
        (tangents[1:]+tangents[:-1])*.5*length/2000, axis=0)))
    result = base.copy()
    result[join:] = tail[::10]
    return base, result


class TurnPreview:
    def __init__(self):
        self.mode = 'road'
        self.points = None
        self.target = None
        self.projection = None
        self.last_t = None
        self.status = 'road'

    def request(self, mode, estimate):
        if mode not in ('road', 'left', 'right'):
            raise ValueError('unknown mode')
        if mode == 'road':
            if self.mode != 'road':
                self.mode = 'returning'
            return
        if self.mode != 'road':
            raise ValueError('return to road before choosing another turn')
        if not estimate.valid or estimate.projection is None:
            raise ValueError('fresh valid road centerline required')
        self.points, self.target = turn_samples(estimate, mode)
        self.projection = estimate.projection.copy()
        self.mode = mode
        self.status = 'transition'

    @staticmethod
    def curvature(points):
        a, b = np.diff(points, axis=0)[:-1], np.diff(points, axis=0)[1:]
        denominator = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)*np.linalg.norm(a+b, axis=1)
        return 2*(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0])/np.maximum(denominator, 1e-12)

    @staticmethod
    def step(previous, target, dt):
        delta = target-previous
        gain = min(1., .5*min(dt, .1)/max(np.max(np.linalg.norm(delta, axis=1)), 1e-12))
        old = np.diff(previous, axis=0)
        # Avoid angular jumps or degenerate segments while blending sampled paths.
        for _ in range(30):
            candidate = previous+gain*delta
            new = np.diff(candidate, axis=0)
            angle = np.arctan2(old[:, 0]*new[:, 1]-old[:, 1]*new[:, 0], (old*new).sum(axis=1))
            curvature_change = np.max(np.abs(TurnPreview.curvature(candidate)-TurnPreview.curvature(previous)))
            if (np.min(np.linalg.norm(new, axis=1)) > 1e-6
                    and np.max(np.abs(angle)) <= .05*min(dt, .1)+1e-10
                    and curvature_change <= .005*min(dt, .1)+1e-10):
                return candidate
            gain *= .5
        return previous.copy()

    def update(self, estimate, timestamp_s):
        if not np.isfinite(timestamp_s) or (self.last_t is not None and timestamp_s <= self.last_t):
            return None
        dt = .1 if self.last_t is None else timestamp_s-self.last_t
        self.last_t = timestamp_s
        if self.mode == 'road':
            self.status = 'road'
            return None
        if not estimate.valid:
            self.status = 'waiting_observation'
            return None
        if self.mode == 'returning':
            try:
                self.target = road_samples(estimate)
            except ValueError:
                self.status = 'waiting_observation'
                return None
        self.points = self.step(self.points, self.target, dt)
        error = float(np.max(np.linalg.norm(self.points-self.target, axis=1)))
        self.status = 'transition' if error > 1e-9 else 'preview_only'
        if self.mode == 'returning' and error < 1e-9:
            self.mode, self.status = 'road', 'road'
            return None
        return self.points


class TurnMailbox:
    """Command access is through authenticated oakctl app exec, not an open port."""
    def __init__(self, directory=COMMAND_DIR):
        self.directory = Path(directory)
        self.directory.mkdir(exist_ok=True, parents=True)
        # A process restart never replays a stale turn command.
        self.seen = {p.name for p in self.directory.glob('*.request')}

    def poll(self, planner, estimate):
        for path in sorted(self.directory.glob('*.request')):
            if path.name in self.seen:
                continue
            self.seen.add(path.name)
            response = {'accepted': False}
            try:
                request = json.loads(path.read_text())
                if not 0 <= time.time()-request['created'] <= 10:
                    raise ValueError('expired command')
                planner.request(request['mode'], estimate)
                response.update(accepted=True, mode=planner.mode)
            except (ValueError, KeyError, TypeError) as exc:
                response['error'] = str(exc)
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(response))
            os.replace(temporary, path.with_suffix('.response'))


def draw_turn(frame, points, projection=None, mask_size=(320, 180)):
    """Purple preview, with bounded rasterization even near a projection horizon."""
    height, width = frame.shape[:2]
    if projection is None:
        # Fixed preview scale avoids auto-zoom-induced apparent jumps.
        scale = min((width-40)/60, (height-40)/50)
        pixels = np.column_stack((width/2+points[:, 1]*scale,
                                  height-20-points[:, 0]*scale))
    else:
        bev = np.column_stack(((points[:, 1]/12+.5)*639, (1-points[:, 0]/40)*479, np.ones(len(points))))
        homogeneous = bev @ projection.T
        with np.errstate(divide='ignore', invalid='ignore'):
            pixels = homogeneous[:, :2]/homogeneous[:, 2:3]
        pixels *= (width/mask_size[0], height/mask_size[1])
        # Homographies are scale-invariant: a negative denominator alone does
        # not mean behind-camera. Reject only the horizon singularity.
        pixels[np.abs(homogeneous[:, 2]) <= 1e-6] = np.nan
    for a, b in zip(pixels[:-1], pixels[1:]):
        if np.all(np.isfinite([a, b])) and np.all(np.array([a, b]) >= 0) and np.all(np.array([a, b]) < [width, height]):
            _draw_polyline(frame, np.array([a, b]), (255, 80, 220), thickness=3)
