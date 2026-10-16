"""Prefer linear geometry; promote one degree at a time with time hysteresis.

Fit errors use the existing uncalibrated development-BEV scale.
"""
from dataclasses import replace
import numpy as np
from corridor import CorridorTracker
from temporal_centerline import CenterlineTemporalFilter


class AdaptiveOrderSelector:
    def __init__(self):
        self.degree = 1
        self.last_t = None
        self.pending = None
        self.pending_s = 0.

    def update(self, measured, *, timestamp_s):
        if not np.isfinite(timestamp_s):
            raise ValueError("timestamp must be finite")
        if self.last_t is not None and timestamp_s <= self.last_t:
            return replace(measured, valid=False, tracking_state="out_of_order")
        dt = .1 if self.last_t is None else timestamp_s - self.last_t
        if dt > .5:
            self.degree, self.pending, self.pending_s = 1, None, 0.
            dt = .1
        self.last_t = timestamp_s
        if not measured.valid or measured.projection is None:
            self.pending, self.pending_s = None, 0.
            return replace(measured, model_degree=self.degree, target_degree=self.degree)
        x = np.linspace(*measured.forward_range_m, 80)
        target = np.polynomial.polynomial.polyval(x, measured.coefficients)
        fits, errors = {}, {}
        for degree in (1, 2, 3):
            p = np.polynomial.Polynomial.fit(x, target, degree).convert()
            fits[degree] = np.pad(p.coef, (0, 4-len(p.coef)))
            errors[degree] = float(np.sqrt(np.mean((p(x) - target)**2)))
        desired, duration = self.degree, 0.
        if self.degree > 1 and errors[self.degree-1] < .12:
            desired, duration = self.degree-1, 1.
        elif self.degree < 3:
            threshold = .25 if self.degree == 1 else .35
            improvement = errors[self.degree] - errors[self.degree+1]
            if self.degree == 1:
                # An S bend may have no useful quadratic term. Still enter degree 2
                # before confirming degree 3; never get stuck at degree 1 for symmetry.
                improvement = max(improvement, errors[1] - errors[3])
            if errors[self.degree] > threshold and improvement > .10:
                desired = self.degree+1
                duration = .4 if desired == 2 else .8
        if desired == self.degree:
            self.pending, self.pending_s = None, 0.
        else:
            self.pending_s = self.pending_s + dt if self.pending == desired else dt
            self.pending = desired
            if self.pending_s + 1e-9 >= duration:
                self.degree = desired
                self.pending, self.pending_s = None, 0.
        coefficients = fits[self.degree]
        fitted = np.polynomial.polynomial.polyval(x, coefficients)
        inside = ((fitted >= CorridorTracker.boundary(measured.bev_left, x)) &
                  (fitted <= CorridorTracker.boundary(measured.bev_right, x)))
        result = CenterlineTemporalFilter._curve(measured, coefficients,
                    measured.forward_range_m, "measured", 0., False)
        return replace(result, valid=bool(np.mean(inside) >= .9),
                       model_degree=self.degree, target_degree=self.degree,
                       order_fit_error=errors[self.degree])
