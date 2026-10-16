"""Time-based cubic tracking on the default development BEV (x forward, y right)."""
from dataclasses import replace
import numpy as np


class CenterlineTemporalFilter:
    def __init__(self, *, time_constant_s=0.35, jump_m=0.65,
                 max_lateral_rate_m_s=1.0, hold_s=0.3, reacquire_s=0.3):
        self.tau = time_constant_s
        self.jump = jump_m
        self.rate = max_lateral_rate_m_s
        self.hold = hold_s
        self.reacquire = reacquire_s
        self.previous = None
        self.last_t = None
        self.age = 0.0
        self.candidate = None
        self.candidate_time = 0.0

    @staticmethod
    def _values(estimate, x):
        return np.polynomial.polynomial.polyval(x, estimate.coefficients)

    @staticmethod
    def _curve(template, coefficients, bounds, state, age, motion):
        x = np.linspace(*bounds, 80)
        y = np.polynomial.polynomial.polyval(x, coefficients)
        bev = np.column_stack(((y / 12.0 + 0.5) * 639, (1 - x / 40) * 479))
        projected = np.column_stack((bev, np.ones(len(bev)))) @ template.projection.T
        image = projected[:, :2] / projected[:, 2:3]
        return replace(template, valid=True, coefficients=tuple(coefficients),
                       forward_range_m=bounds, bev_curve=bev.astype(np.float32),
                       image_curve=image.astype(np.float32), tracking_state=state,
                       measurement_age_s=age, motion_compensated=motion)

    def update(self, measured, *, timestamp_s, speed_m_s=None, yaw_rate_rad_s=None):
        """Yaw positive toward +y (right); motion values must cover this frame interval.

        Missing motion uses a stationary prior, never invents vehicle velocity.
        A gap >0.5 s resets tracking; rejected/stale measurements cannot remain valid.
        """
        if not np.isfinite(timestamp_s):
            raise ValueError("timestamp must be finite")
        if self.last_t is not None and timestamp_s <= self.last_t:
            return replace(measured, valid=False, tracking_state="out_of_order")
        dt = 0.1 if self.last_t is None else timestamp_s - self.last_t
        self.last_t = timestamp_s
        if dt > 0.5:
            self.previous = None
            self.candidate = None
            self.candidate_time = 0.0
            dt = 0.1
        self.age += dt
        usable = (measured.valid and measured.projection is not None
                  and np.all(np.isfinite(measured.coefficients))
                  and np.isfinite(measured.residual_m) and measured.residual_m <= 0.75)
        prior = self.previous
        motion = (speed_m_s is not None and yaw_rate_rad_s is not None
                  and np.isfinite(speed_m_s) and np.isfinite(yaw_rate_rad_s))
        if prior is not None and motion:
            angle = yaw_rate_rad_s * dt
            distance = speed_m_s * dt
            dx = distance if abs(angle) < 1e-8 else distance * np.sin(angle) / angle
            dy = 0.0 if abs(angle) < 1e-8 else distance * (1 - np.cos(angle)) / angle
            x = np.linspace(*prior.forward_range_m, 80)
            y = self._values(prior, x)
            nx = np.cos(angle) * (x - dx) + np.sin(angle) * (y - dy)
            ny = -np.sin(angle) * (x - dx) + np.cos(angle) * (y - dy)
            if np.any(np.diff(nx) <= 0) or nx[-1] - max(0, nx[0]) < 5:
                prior = None
            else:
                coef = np.polynomial.Polynomial.fit(nx, ny, 3).convert().coef
                prior = self._curve(prior, coef, (max(0, nx[0]), min(40, nx[-1])),
                                    "predicted", self.age, True)
        if not usable:
            self.candidate = None
            self.candidate_time = 0.0
            self.previous = prior
            if prior is not None and self.age <= self.hold + 1e-9:
                return replace(prior, tracking_state="predicted" if motion else "held",
                               measurement_age_s=self.age, motion_compensated=bool(motion))
            return replace(measured, valid=False, tracking_state="lost", measurement_age_s=self.age)

        # Compare only points representing the same forward distances.
        bounds = measured.forward_range_m
        if prior is not None:
            bounds = (max(bounds[0], prior.forward_range_m[0]),
                      min(bounds[1], prior.forward_range_m[1]))
        overlap = bounds[1] - bounds[0] >= 5
        x = np.linspace(*bounds, 80) if overlap else np.array([])
        jump = (float(np.max(np.abs(self._values(measured, x) - self._values(prior, x))))
                if prior is not None and overlap else float("inf"))
        needs_confirmation = prior is None or not overlap or jump > self.jump or self.age > self.hold + dt
        if needs_confirmation:
            consistent = False
            if self.candidate is not None:
                lo = max(measured.forward_range_m[0], self.candidate.forward_range_m[0])
                hi = min(measured.forward_range_m[1], self.candidate.forward_range_m[1])
                if hi - lo >= 5:
                    grid = np.linspace(lo, hi, 40)
                    consistent = np.max(np.abs(self._values(measured, grid) - self._values(self.candidate, grid))) < self.jump
            self.candidate_time = self.candidate_time + dt if consistent else dt
            self.candidate = measured
            if self.candidate_time + 1e-9 < self.reacquire:
                self.previous = prior
                if prior is not None and self.age <= self.hold + 1e-9:
                    return replace(prior, tracking_state="held", measurement_age_s=self.age,
                                   motion_compensated=bool(motion))
                return replace(measured, valid=False, tracking_state="acquiring", measurement_age_s=self.age)
        if prior is None or not overlap:
            result = replace(measured, tracking_state="reinitialized", motion_compensated=False)
        else:
            quality = 1 / (1 + (measured.residual_m / 0.25) ** 2)
            gain = (1 - np.exp(-dt / self.tau)) * quality
            # One global gain bounds displacement of the entire cubic without clipping its shape.
            gain = min(gain, self.rate * dt / max(jump, 1e-9))
            coef = np.asarray(prior.coefficients) + gain * (np.asarray(measured.coefficients) - prior.coefficients)
            result = self._curve(measured, coef, bounds,
                                 "reacquiring" if needs_confirmation else "tracking", 0.0, bool(motion))
        self.previous = result
        self.age = 0.0
        self.candidate = None
        self.candidate_time = 0.0
        return result
