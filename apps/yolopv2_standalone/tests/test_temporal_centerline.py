import unittest
from dataclasses import replace
import numpy as np
from centerline import CenterlineTemporalFilter, estimate_centerline
from test_centerline import trapezoid_mask


class TemporalTests(unittest.TestCase):
    def setUp(self):
        self.line = estimate_centerline(trapezoid_mask())
        self.tracker = CenterlineTemporalFilter()
        for t in (0., .1, .2):
            self.result = self.tracker.update(self.line, timestamp_s=t)

    def shifted(self, distance):
        c = list(self.line.coefficients)
        c[0] += distance
        return replace(self.line, coefficients=tuple(c))

    def test_persistent_jump_is_rate_limited(self):
        previous = self.result
        x = np.linspace(2, 35, 50)
        for t in np.arange(.3, 2, .1):
            current = self.tracker.update(self.shifted(2), timestamp_s=float(t))
            if current.valid and previous.valid:
                delta = self.tracker._values(current, x) - self.tracker._values(previous, x)
                self.assertLessEqual(np.max(np.abs(delta)), .100001)
            previous = current

    def test_dropout_expires_and_reacquisition_requires_confirmation(self):
        blank = replace(self.line, valid=False)
        self.assertEqual(self.tracker.update(blank, timestamp_s=.3).tracking_state, 'held')
        self.tracker.update(blank, timestamp_s=.4)
        self.tracker.update(blank, timestamp_s=.5)
        self.assertFalse(self.tracker.update(blank, timestamp_s=.6).valid)
        self.assertFalse(self.tracker.update(self.shifted(2), timestamp_s=.7).valid)

    def test_alternating_outliers_do_not_reacquire(self):
        for i in range(3, 15):
            r = self.tracker.update(self.shifted(2 if i % 2 else -2), timestamp_s=i / 10)
        self.assertFalse(r.valid)

    def test_same_polynomial_different_support_does_not_jump(self):
        r = self.tracker.update(replace(self.line, forward_range_m=(10, 30)), timestamp_s=.3)
        np.testing.assert_allclose(r.coefficients, self.line.coefficients, atol=1e-8)
        x = (1 - r.bev_curve[:, 1] / 479) * 40
        y = (r.bev_curve[:, 0] / 639 - .5) * 12
        np.testing.assert_allclose(y, self.tracker._values(r, x), atol=1e-5)

    def test_out_of_order_and_gap(self):
        self.assertFalse(self.tracker.update(self.line, timestamp_s=.1).valid)
        r = self.tracker.update(self.line, timestamp_s=2)
        self.assertEqual(r.tracking_state, 'acquiring')

    def test_motion_translation(self):
        # Sloping line y=0.1*x becomes y=0.1*x+0.1 after moving forward 1 m.
        tracker = CenterlineTemporalFilter(reacquire_s=.1)
        line = replace(self.line, coefficients=(0., .1, 0., 0.))
        tracker.update(line, timestamp_s=0)
        r = tracker.update(replace(line, valid=False), timestamp_s=.1,
                           speed_m_s=10., yaw_rate_rad_s=0.)
        np.testing.assert_allclose(r.coefficients[:2], (.1, .1), atol=1e-6)
        self.assertTrue(r.motion_compensated)

    def test_motion_yaw(self):
        tracker = CenterlineTemporalFilter(reacquire_s=.1)
        line = replace(self.line, coefficients=(0., 0., 0., 0.))
        tracker.update(line, timestamp_s=0)
        r = tracker.update(replace(line, valid=False), timestamp_s=.1,
                           speed_m_s=0., yaw_rate_rad_s=.2)
        self.assertAlmostEqual(r.coefficients[1], -np.tan(.02), places=6)

    def test_no_motion_input_is_explicit(self):
        self.assertFalse(self.result.motion_compensated)


if __name__ == '__main__':
    unittest.main()
