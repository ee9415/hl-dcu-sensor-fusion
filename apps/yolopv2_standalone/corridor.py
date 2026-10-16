"""Development-BEV corridor association before temporal smoothing.

Distances use the existing uncalibrated 12 m / 40 m BEV scale, not measured metres.
"""
from dataclasses import replace
import numpy as np
from temporal_centerline import CenterlineTemporalFilter


class CorridorTracker:
    def __init__(self, width_tolerance=0.5, stable_tolerance=0.3, max_inferred_s=3.0):
        self.width_tolerance = width_tolerance
        self.stable_tolerance = stable_tolerance
        self.max_inferred_s = max_inferred_s
        self.reference = None
        self.last_t = None
        self.inferred_since = None

    @staticmethod
    def boundary(points, x):
        forward = (1 - points[:, 1] / 479) * 40
        lateral = (points[:, 0] / 639 - .5) * 12
        order = np.argsort(forward)
        return np.interp(x, forward[order], lateral[order])

    def update(self, measured, *, timestamp_s):
        if not np.isfinite(timestamp_s):
            raise ValueError("timestamp must be finite")
        if self.last_t is not None and timestamp_s <= self.last_t:
            return replace(measured, valid=False, corridor_state="out_of_order")
        if self.last_t is not None and timestamp_s - self.last_t > .5:
            self.reference = None
            self.inferred_since = None
        self.last_t = timestamp_s
        if not measured.valid:
            return replace(measured, corridor_state="missing")
        if self.reference is None:
            self.reference = measured
            return measured
        old = self.reference
        lo = max(old.forward_range_m[0], measured.forward_range_m[0])
        hi = min(old.forward_range_m[1], measured.forward_range_m[1])
        if hi - lo < 5:
            return replace(measured, valid=False, corridor_state="insufficient_overlap")
        x = np.linspace(lo, hi, 80)
        left = self.boundary(measured.bev_left, x)
        right = self.boundary(measured.bev_right, x)
        old_left = self.boundary(old.bev_left, x)
        old_right = self.boundary(old.bev_right, x)
        dl, dr = left - old_left, right - old_right
        width_change = (right - left) - (old_right - old_left)
        changed = np.abs(width_change) > self.width_tolerance
        if not np.any(changed):
            # Both boundaries move together on a bend: accept the new corridor.
            self.reference = measured
            self.inferred_since = None
            return measured
        left_stable = np.abs(dl) <= self.stable_tolerance
        right_stable = np.abs(dr) <= self.stable_tolerance
        expanded = width_change > self.width_tolerance
        infer_left = changed & expanded & left_stable & ~right_stable
        infer_right = changed & expanded & right_stable & ~left_stable
        ambiguous = changed & ~(infer_left | infer_right)
        if np.any(ambiguous):
            # Narrowing/obstacles or both boundaries lost: do not invent free space.
            return replace(measured, valid=False, corridor_state="ambiguous")
        if self.inferred_since is None:
            self.inferred_since = timestamp_s
        if timestamp_s - self.inferred_since > self.max_inferred_s:
            return replace(measured, valid=False, corridor_state="inference_expired")
        center = (left + right) / 2
        width = old_right - old_left
        center[infer_left] = left[infer_left] + width[infer_left] / 2
        center[infer_right] = right[infer_right] - width[infer_right] / 2
        coefficients = np.polynomial.Polynomial.fit(x, center, 3).convert().coef
        fitted = np.polynomial.polynomial.polyval(x, coefficients)
        residual = float(np.sqrt(np.mean((fitted - center) ** 2)))
        if residual > .75 or np.any(fitted < left) or np.any(fitted > right):
            return replace(measured, valid=False, corridor_state="unsafe_fit")
        result = CenterlineTemporalFilter._curve(
            measured, coefficients, (lo, hi), "measured", 0., False)
        # Never learn the expanded width, even when it persists for many frames.
        return replace(result, residual_m=residual, corridor_state="single_boundary")
