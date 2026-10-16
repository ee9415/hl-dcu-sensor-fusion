"""Follow the current measured cubic; briefly suppress unilateral expansion only.

Threshold distances use the existing development BEV, not calibrated road metres.
"""
from dataclasses import replace
import numpy as np
from temporal_centerline import CenterlineTemporalFilter


class CorridorTracker:
    def __init__(self, width_tolerance=.6, stable_tolerance=.25, release_s=.8):
        self.width_tolerance = width_tolerance
        self.stable_tolerance = stable_tolerance
        self.release_s = release_s
        self.reference = None
        self.last_t = None
        self.anchor = None
        self.guard_since = None
        self.side = None

    @staticmethod
    def boundary(points, x):
        forward = (1 - points[:, 1] / 479) * 40
        order = np.argsort(forward)
        return np.interp(x, forward[order], (points[order, 0] / 639 - .5) * 12)

    def clear_guard(self):
        self.anchor = None
        self.guard_since = None
        self.side = None

    def expansion(self, current, previous):
        lo = max(current.forward_range_m[0], previous.forward_range_m[0])
        hi = min(current.forward_range_m[1], previous.forward_range_m[1])
        if hi - lo < 5:
            return None
        x = np.linspace(lo, hi, 80)
        dl = self.boundary(current.bev_left, x) - self.boundary(previous.bev_left, x)
        dr = self.boundary(current.bev_right, x) - self.boundary(previous.bev_right, x)
        left = (dl < -self.width_tolerance) & (np.abs(dr) <= self.stable_tolerance)
        right = (dr > self.width_tolerance) & (np.abs(dl) <= self.stable_tolerance)
        # Require a coherent section, not an isolated noisy row.
        for side, selected in (("left", left), ("right", right)):
            if np.mean(selected) >= .35 and np.ptp(x[selected]) >= 5:
                return side
        return None

    def update(self, measured, *, timestamp_s):
        if not np.isfinite(timestamp_s):
            raise ValueError("timestamp must be finite")
        if self.last_t is not None and timestamp_s <= self.last_t:
            return replace(measured, valid=False, corridor_state="out_of_order")
        if self.last_t is not None and timestamp_s - self.last_t > .5:
            self.reference = None
            self.clear_guard()
        self.last_t = timestamp_s
        if not measured.valid:
            self.reference = None
            self.clear_guard()
            return replace(measured, corridor_state="missing")
        previous = self.reference
        self.reference = measured  # Always advance the raw reference, never freeze width.
        if previous is None:
            return measured
        if self.anchor is not None:
            if self.expansion(measured, self.anchor) != self.side:
                self.clear_guard()  # Return/bend: immediately follow the current dotted line.
        elif self.expansion(measured, previous) is not None:
            self.anchor = previous
            self.side = self.expansion(measured, previous)
            self.guard_since = timestamp_s
        if self.anchor is None:
            return measured
        elapsed = timestamp_s - self.guard_since
        if elapsed >= self.release_s:
            self.clear_guard()
            return measured
        anchor = self.anchor
        # Never extrapolate the previous cubic into new visibility.
        if (anchor.forward_range_m[0] > measured.forward_range_m[0] + .1
                or anchor.forward_range_m[1] < measured.forward_range_m[1] - .1):
            self.clear_guard()
            return measured
        fraction = .15 + .85 * min(1., elapsed / self.release_s)
        coefficients = (1 - fraction) * np.asarray(anchor.coefficients) + fraction * np.asarray(measured.coefficients)
        x = np.linspace(*measured.forward_range_m, 80)
        y = np.polynomial.polynomial.polyval(x, coefficients)
        if np.mean((y >= self.boundary(measured.bev_left, x)) &
                   (y <= self.boundary(measured.bev_right, x))) < .95:
            self.clear_guard()
            return measured
        result = CenterlineTemporalFilter._curve(measured, coefficients,
                    measured.forward_range_m, "measured", 0., False)
        return replace(result, corridor_state="expansion_guard")
