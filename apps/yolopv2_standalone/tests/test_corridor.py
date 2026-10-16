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

    def test_either_side_expansion_does_not_pull_center(self):
        for left, right in ((0, 2), (-2, 0)):
            tracker = CorridorTracker()
            tracker.update(self.base, timestamp_s=0.)
            for i in range(1, 21):
                result = tracker.update(self.changed(left, right), timestamp_s=i / 10)
                self.assertTrue(result.valid)
                self.assertEqual(result.corridor_state, 'single_boundary')
                x = np.linspace(*result.forward_range_m, 80)
                delta = CenterlineTemporalFilter._values(result, x) - CenterlineTemporalFilter._values(self.base, x)
                self.assertLess(np.max(np.abs(delta)), .03)

    def test_coherent_boundary_motion_is_accepted(self):
        result = self.tracker.update(self.changed(.8, .8), timestamp_s=.1)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.coefficients[0], self.base.coefficients[0] + .8)

    def test_curved_road_is_accepted(self):
        curved = estimate_centerline(trapezoid_mask(curve_strength=10))
        result = self.tracker.update(curved, timestamp_s=.1)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.coefficients, curved.coefficients)

    def test_ambiguous_expansion_and_narrowing_are_invalid(self):
        for left, right in ((-2, 2), (1, -1)):
            self.assertFalse(self.tracker.update(self.changed(left, right),
                                                timestamp_s=.1 if left < 0 else .2).valid)

    def test_inference_expires_and_normal_boundaries_recover(self):
        for i in range(1, 34):
            result = self.tracker.update(self.changed(0, 2), timestamp_s=i / 10)
        self.assertFalse(result.valid)
        self.assertEqual(result.corridor_state, 'inference_expired')
        self.assertTrue(self.tracker.update(self.base, timestamp_s=3.4).valid)

    def test_gap_resets_and_old_timestamp_is_rejected(self):
        self.assertFalse(self.tracker.update(self.base, timestamp_s=0).valid)
        result = self.tracker.update(self.changed(0, 2), timestamp_s=2)
        self.assertEqual(result.corridor_state, 'measured')


if __name__ == '__main__':
    unittest.main()
