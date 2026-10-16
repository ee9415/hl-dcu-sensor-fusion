import unittest
from dataclasses import replace
import numpy as np
from centerline import estimate_centerline, CenterlineTemporalFilter
from corridor import CorridorTracker
from test_centerline import trapezoid_mask


class CorridorTests(unittest.TestCase):
    def setUp(self):
        self.base = estimate_centerline(trapezoid_mask())
        self.tracker = CorridorTracker()
        self.tracker.update(self.base, timestamp_s=0.)

    def changed(self, left, right):
        l, r = self.base.bev_left.copy(), self.base.bev_right.copy()
        l[:, 0] += left * 639 / 12
        r[:, 0] += right * 639 / 12
        coef = np.array(self.base.coefficients)
        coef[0] += (left + right) / 2
        return replace(self.base, bev_left=l, bev_right=r, coefficients=tuple(coef))

    def test_either_expansion_is_suppressed_initially_but_not_frozen(self):
        for l, r in ((0, 2), (-2, 0)):
            tracker = CorridorTracker()
            tracker.update(self.base, timestamp_s=0.)
            raw = self.changed(l, r)
            first = tracker.update(raw, timestamp_s=.1)
            self.assertEqual(first.corridor_state, 'expansion_guard')
            self.assertAlmostEqual(first.coefficients[0] - self.base.coefficients[0], (l+r)*.075)
            for i in range(2, 21):
                result = tracker.update(raw, timestamp_s=i/10)
            np.testing.assert_allclose(result.coefficients, raw.coefficients)
            self.assertEqual(result.corridor_state, 'measured')

    def test_bend_bilateral_expansion_and_narrowing_pass_through(self):
        for l, r in ((1, 1), (-1, 1), (1, -1)):
            tracker = CorridorTracker()
            tracker.update(self.base, timestamp_s=0.)
            raw = self.changed(l, r)
            result = tracker.update(raw, timestamp_s=.1)
            np.testing.assert_allclose(result.coefficients, raw.coefficients)
            self.assertEqual(result.corridor_state, 'measured')

    def test_return_cancels_guard(self):
        self.tracker.update(self.changed(0, 2), timestamp_s=.1)
        result = self.tracker.update(self.base, timestamp_s=.2)
        self.assertEqual(result.corridor_state, 'measured')
        np.testing.assert_allclose(result.coefficients, self.base.coefficients)

    def test_small_width_change_is_not_guarded(self):
        raw = self.changed(0, .2)
        self.assertEqual(self.tracker.update(raw, timestamp_s=.1).corridor_state, 'measured')

    def test_gradual_expansion_is_not_a_sudden_jump(self):
        for i in range(1, 20):
            raw = self.changed(0, i * .1)
            result = self.tracker.update(raw, timestamp_s=i/10)
            self.assertEqual(result.corridor_state, 'measured')
            np.testing.assert_allclose(result.coefficients, raw.coefficients)

    def test_pipeline_rejoins_raw_after_guard(self):
        tracker = CenterlineTemporalFilter(measurement_following=True, reacquire_s=.1)
        tracker.update(self.base, timestamp_s=0.)
        raw = self.changed(0, 2)
        for i in range(1, 16):
            corrected = self.tracker.update(raw, timestamp_s=i/10)
            result = tracker.update(corrected, timestamp_s=i/10)
            if i == 1:
                self.assertLess(abs(result.coefficients[0] - self.base.coefficients[0]), .11)
        self.assertLess(abs(result.coefficients[0] - raw.coefficients[0]), .03)

    def test_gap_missing_and_order(self):
        self.assertFalse(self.tracker.update(self.base, timestamp_s=0).valid)
        self.assertFalse(self.tracker.update(replace(self.base, valid=False), timestamp_s=.1).valid)
        raw = self.changed(0, 2)
        self.assertEqual(self.tracker.update(raw, timestamp_s=1).corridor_state, 'measured')

    def test_following_filter_catches_current_curve_quickly(self):
        tracker = CenterlineTemporalFilter(measurement_following=True, reacquire_s=.1)
        tracker.update(self.base, timestamp_s=0.)
        target = self.changed(1, 1)
        for i in range(1, 6):
            result = tracker.update(target, timestamp_s=i/10)
        self.assertLess(abs(result.coefficients[0] - target.coefficients[0]), .03)

    def test_curved_road_passes_through(self):
        raw = estimate_centerline(trapezoid_mask(curve_strength=10))
        result = self.tracker.update(raw, timestamp_s=.1)
        np.testing.assert_allclose(result.coefficients, raw.coefficients)

    def test_follow_mode_uses_same_validity_as_dotted_line(self):
        tracker = CenterlineTemporalFilter(measurement_following=True, reacquire_s=.1)
        raw = replace(self.base, residual_m=1.)
        self.assertTrue(tracker.update(raw, timestamp_s=0).valid)
        for i in range(1, 6):
            result = tracker.update(replace(raw, valid=False), timestamp_s=i/10)
        self.assertFalse(result.valid)


if __name__ == '__main__':
    unittest.main()
