"""Development-BEV corridor association before temporal smoothing.

Distances use the existing uncalibrated 12 m / 40 m BEV scale, not measured metres.
"""
from dataclasses import replace
import numpy as np
from temporal_centerline import CenterlineTemporalFilter


class CorridorTracker:
    def __init__(self, width_tolerance=0.5, stable_tolerance=0.3):
        self.width_tolerance = width_tolerance
        self.stable_tolerance = stable_tolerance
        self.reference = None
        self.last_t = None
        self.candidate = None
        self.candidate_since = None

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
            self.candidate = None
        self.last_t = timestamp_s
        if not measured.valid:
            self.candidate = None
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
            self.candidate = None
            return measured
        left_stable = np.abs(dl) <= self.stable_tolerance
        right_stable = np.abs(dr) <= self.stable_tolerance
        expanded = width_change > self.width_tolerance
        infer_left = changed & expanded & left_stable & ~right_stable
        infer_right = changed & expanded & right_stable & ~left_stable
        ambiguous = changed & ~(infer_left | infer_right)
        old_center = (old_left + old_right) / 2
        contained = (old_center > left + .15) & (old_center < right - .15)
        retained = ambiguous & contained
        reliable = ~ambiguous | retained
        if np.count_nonzero(reliable) < 32 or np.ptp(x[reliable]) < 5:
            # A changed scene must establish BOTH boundaries again, not lock out forever.
            consistent = False
            if self.candidate is not None:
                c = self.candidate
                covered = c.forward_range_m[0] <= lo + 1 and c.forward_range_m[1] >= hi - 1
                consistent = covered and all(np.percentile(np.abs(current - self.boundary(points, x)), 80) < .5
                    for current, points in ((left, c.bev_left), (right, c.bev_right)))
            if not consistent:
                self.candidate = measured
                self.candidate_since = timestamp_s
            elif timestamp_s - self.candidate_since >= .3:
                self.reference = measured
                self.candidate = None
                return replace(measured, corridor_state="reacquired")
            return replace(measured, valid=False, corridor_state="ambiguous")
        self.candidate = None
        # Containment is checked against THIS frame's road mask, not a blind time hold.
        # Missing masks still return invalid above and expire in the temporal filter.
        center = (left + right) / 2
        center[retained] = old_center[retained]
        width = old_right - old_left
        center[infer_left] = left[infer_left] + width[infer_left] / 2
        center[infer_right] = right[infer_right] - width[infer_right] / 2
        # Fit only supported distances; an isolated bad boundary must not veto the frame.
        fit_x, fit_y = x[reliable], center[reliable]
        weights = np.ones(len(fit_x))
        for _ in range(4):
            polynomial = np.polynomial.Polynomial.fit(fit_x, fit_y, 3, w=weights)
            errors = np.abs(polynomial(fit_x) - fit_y)
            weights = np.sqrt(np.minimum(1., .3 / np.maximum(errors, 1e-6)))
        coefficients = polynomial.convert().coef
        fitted = np.polynomial.polynomial.polyval(x, coefficients)
        residual = float(np.sqrt(np.average((fitted[reliable] - center[reliable]) ** 2, weights=weights**2)))
        inside_fit = reliable & (fitted >= left) & (fitted <= right)
        runs = np.split(np.flatnonzero(inside_fit), np.flatnonzero(np.diff(np.flatnonzero(inside_fit)) > 1) + 1)
        supported = max(runs, key=len)
        if residual > .75 or len(supported) < 16 or np.ptp(x[supported]) < 5:
            raw = np.polynomial.polynomial.polyval(x, measured.coefficients)
            # A failed correction must not discard an otherwise supported current
            # observation. The temporal stage still bounds its motion.
            if measured.residual_m <= .5 and np.mean((raw >= left) & (raw <= right)) >= .95:
                self.reference = measured
                return replace(measured, corridor_state="measurement_fallback")
            return replace(measured, valid=False, corridor_state="unsafe_fit")
        bounds = (float(x[supported].min()), float(x[supported].max()))
        result = CenterlineTemporalFilter._curve(
            measured, coefficients, bounds, "measured", 0., False)
        # Advance supported boundaries, but preserve width on expanded samples.
        ref_left, ref_right = left.copy(), right.copy()
        ref_right[infer_left] = left[infer_left] + width[infer_left]
        ref_left[infer_right] = right[infer_right] - width[infer_right]
        ref_left[ambiguous], ref_right[ambiguous] = old_left[ambiguous], old_right[ambiguous]
        # Keep the current full support: intersecting reference ranges forever collapses
        # the visible curve to a short distant fragment after repeated mask dropouts.
        reference_left = measured.bev_left.copy()
        reference_right = measured.bev_right.copy()
        for points, corrected in ((reference_left, ref_left), (reference_right, ref_right)):
            distances = (1 - points[:, 1] / 479) * 40
            inside = (distances >= lo) & (distances <= hi)
            points[inside, 0] = (np.interp(distances[inside], x, corrected) / 12 + .5) * 639
        self.reference = replace(measured, bev_left=reference_left, bev_right=reference_right)
        # Never learn the expanded width, even when it persists for many frames.
        state = ("contained_prior" if np.mean(retained) > .8 else
                 "single_boundary" if np.any(infer_left | infer_right) else "partial_boundaries")
        return replace(result, residual_m=residual, corridor_state=state)
