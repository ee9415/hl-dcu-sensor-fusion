"""Observed far corridor goal, not an inferred lane or obstacle-safe route.

All distances use the uncalibrated development projection.
"""
from dataclasses import replace
import numpy as np
from corridor import CorridorTracker
from temporal_centerline import CenterlineTemporalFilter
from conservative_centerline import ConservativeCenterline
from centerline import _transform, _draw_polyline


def guide_to_far_corridor(ordered, observed, previous=None):
    if not observed.valid or observed.projection is None:
        return ordered
    if not ordered.valid:
        ordered = observed
    left, right = observed.bev_left, observed.bev_right
    if len(left) < 12 or len(left) != len(right):
        return ordered
    forward = (1-left[:, 1]/479)*40
    # Search a supported band in the far quarter, not a single extremal row.
    # Prefer a narrow connected passage over a widened junction/shoulder.
    far = np.argsort(forward)[-max(8, len(forward)//4):]
    bands = [far[i:i+8] for i in range(len(far)-7)]
    def band_width(indices):
        width = right[indices, 0]-left[indices, 0]
        return float(np.median(width)+.2*np.std(width))
    indices = min(bands, key=band_width)
    gx = float(np.median(forward[indices]))
    centers = ((left[indices, 0]+right[indices, 0])/2/639-.5)*12
    gy = float(np.median(centers))
    half_width = float(np.median((right[indices, 0]-left[indices, 0])/639*6))
    lo = observed.forward_range_m[0]
    if gx-lo < 8 or half_width <= .05 or not np.all(np.isfinite([gx, gy, half_width])):
        return ordered
    x = np.linspace(lo, gx, 80)
    lower = CorridorTracker.boundary(left, x)
    upper = CorridorTracker.boundary(right, x)
    reference = CenterlineTemporalFilter._values(ordered, x)
    anchor = reference[0]
    if previous is not None and np.all(np.isfinite(previous)):
        old_anchor = np.polynomial.polynomial.polyval(lo, previous)
        if lower[0] <= old_anchor <= upper[0]:
            anchor = .7*old_anchor+.3*anchor
    anchor = float(np.clip(anchor, lower[0], upper[0]))
    distance = gx-lo
    # Search bounded quadratics anchored near the vehicle. Solve the linear
    # term from the far goal first; only use curvature when it improves fit.
    best, score = None, float('inf')
    for q in np.linspace(-ConservativeCenterline.curvature_limit/2,
                         ConservativeCenterline.curvature_limit/2, 121):
        b = (gy-anchor-q*distance**2)/distance
        # Global derivative limits on 0..40, expressed in local z=x-lo.
        low_b = max(-.2+2*q*lo, -.2-2*q*(40-lo))
        high_b = min(.2+2*q*lo, .2-2*q*(40-lo))
        b = float(np.clip(b, low_b, high_b))
        coef = np.array([anchor-b*lo+q*lo*lo, b-2*q*lo, q, 0.])
        values = np.polynomial.polynomial.polyval(x, coef)
        outside = np.maximum(lower-values, 0)+np.maximum(values-upper, 0)
        goal_error = values[-1]-gy
        cost = (20*goal_error**2 + 30*np.mean(outside**2)
                + .15*np.mean((values-reference)**2) + .03*(q/.003)**2)
        if cost < score:
            best, score = coef, cost
    result = CenterlineTemporalFilter._curve(ordered, best, (lo, gx), 'goal_guided', 0., False)
    goal_bev = np.array([[(gy/12+.5)*639, (1-gx/40)*479]])
    goal_image = tuple(float(v) for v in _transform(goal_bev, observed.projection)[0])
    return replace(result, model_degree=1 if abs(best[2]) < 1e-12 else 2,
                   target_degree=1 if abs(best[2]) < 1e-12 else 2,
                   goal_position=(gx, gy), goal_image=goal_image,
                   goal_half_width=half_width)


def draw_goal(frame, estimate, mask_size):
    if estimate.goal_image is None:
        return
    x, y = np.array(estimate.goal_image)*[frame.shape[1]/mask_size[0], frame.shape[0]/mask_size[1]]
    if not np.all(np.isfinite([x, y])) or not (6 <= x < frame.shape[1]-6 and 6 <= y < frame.shape[0]-6):
        return
    _draw_polyline(frame, np.array([[x-5, y], [x+5, y]]), (255, 255, 255), 1)
    _draw_polyline(frame, np.array([[x, y-5], [x, y+5]]), (255, 255, 255), 1)
