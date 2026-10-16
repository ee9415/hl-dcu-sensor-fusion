"""Final display-path envelope. Units are UNCALIBRATED development BEV units.

Not a vehicle steering controller. No fitting may follow this final limiter.
"""
from dataclasses import replace
import numpy as np
from corridor import CorridorTracker
from temporal_centerline import CenterlineTemporalFilter


class ConservativeCenterline:
    lateral_rate = .5
    heading_rate = .05
    curvature_rate = .005
    curvature_limit = .006
    slope_limit = .20

    def __init__(self):
        self.previous = None
        self.last_t = None
        self.metrics = {}

    @classmethod
    def bounds(cls, delta):
        # Exact extrema over the fixed 0..40 domain, independent of visible range.
        x = [0., 40.]
        if abs(delta[2]) > 1e-15:
            vertex = -delta[1] / (2 * delta[2])
            if 0 < vertex < 40:
                x.append(vertex)
        lateral = np.max(np.abs(np.polynomial.polynomial.polyval(x, delta)))
        slope = max(abs(delta[1]), abs(delta[1] + 80 * delta[2]))
        # atan is 1-Lipschitz. For quadratic curvature, the following is a
        # conservative bound on change, including the slope denominator.
        curvature = abs(2 * delta[2]) + 3 * cls.curvature_limit * slope
        return np.array([lateral, slope, curvature])

    def update(self, measured, *, timestamp_s):
        if not np.isfinite(timestamp_s):
            raise ValueError('timestamp must be finite')
        self.metrics = {'compared': False, 'limited': False, 'conflict': False}
        if self.last_t is not None and timestamp_s <= self.last_t:
            return replace(measured, valid=False, tracking_state='out_of_order')
        dt = .1 if self.last_t is None else min(timestamp_s - self.last_t, .1)
        # Keep the previous path through missing frames and long gaps; never
        # allow a large elapsed time or temporal-filter reset to bypass limits.
        self.last_t = timestamp_s
        if not measured.valid or measured.projection is None:
            return measured
        if not np.all(np.isfinite(measured.coefficients)):
            return replace(measured, valid=False)
        x = np.linspace(*measured.forward_range_m, 80)
        p = np.polynomial.Polynomial.fit(x, np.polynomial.polynomial.polyval(
            x, measured.coefficients), min(2, measured.model_degree)).convert()
        coef = np.pad(p.coef, (0, 4-len(p.coef)))
        anchor = np.polynomial.polynomial.polyval(x[0], coef)
        coef[2] = np.clip(coef[2], -self.curvature_limit/2, self.curvature_limit/2)
        slope = max(abs(coef[1]), abs(coef[1] + 80 * coef[2]))
        coef[1:3] *= min(1., self.slope_limit / max(slope, 1e-12))
        coef[0] = anchor - coef[1]*x[0] - coef[2]*x[0]**2
        if self.previous is not None:
            delta = coef - self.previous
            changes = self.bounds(delta)
            limits = np.array([self.lateral_rate, self.heading_rate, self.curvature_rate])
            gain = min(1., float(np.min(limits * dt / np.maximum(changes, 1e-15))))
            coef = self.previous + gain * delta
            self.metrics.update(compared=True, limited=gain < 1.,
                                input_rate_bounds=(changes/dt).tolist(),
                                output_rate_bounds=(self.bounds(coef-self.previous)/dt).tolist())
        self.previous = coef.copy()
        fitted = np.polynomial.polynomial.polyval(x, coef)
        inside = ((fitted >= CorridorTracker.boundary(measured.bev_left, x)) &
                  (fitted <= CorridorTracker.boundary(measured.bev_right, x)))
        valid = bool(np.mean(inside) >= .9)
        self.metrics['conflict'] = not valid
        result = CenterlineTemporalFilter._curve(measured, coef, measured.forward_range_m,
                    measured.tracking_state if valid else 'constraint_conflict',
                    measured.measurement_age_s, measured.motion_compensated)
        return replace(result, valid=valid, model_degree=2 if abs(coef[2]) > 1e-12 else 1)
