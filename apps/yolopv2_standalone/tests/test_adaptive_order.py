import unittest
from dataclasses import replace
import numpy as np
from adaptive_order import AdaptiveOrderSelector
from centerline import estimate_centerline, CenterlineTemporalFilter
from test_centerline import trapezoid_mask


class AdaptiveOrderTests(unittest.TestCase):
    def setUp(self):
        self.base = estimate_centerline(trapezoid_mask())
        self.selector = AdaptiveOrderSelector()
        self.x = np.linspace(*self.base.forward_range_m, 80)

    def line(self, degree):
        z = np.polynomial.Polynomial([-1., .05])
        p = {1: .2 + .4*z, 2: 2*z*z - .5, 3: 2*(2.5*z**3 - 1.5*z)}[degree]
        coef = np.pad(p.coef, (0, 4-len(p.coef)))
        return CenterlineTemporalFilter._curve(self.base, coef, self.base.forward_range_m,
                                               'measured', 0., False)

    def test_straight_stays_first_order_with_exact_zero_high_terms(self):
        for i in range(20):
            result = self.selector.update(self.line(1), timestamp_s=i/10)
            self.assertTrue(result.valid)
            self.assertEqual(result.model_degree, 1)
            self.assertEqual(result.coefficients[2:], (0., 0.))

    def test_quadratic_requires_persistent_need(self):
        degrees = [self.selector.update(self.line(2), timestamp_s=i/10).model_degree for i in range(20)]
        self.assertEqual(degrees[0], 1)
        self.assertEqual(degrees[-1], 2)
        self.assertNotIn(3, degrees)

    def test_s_bend_promotes_sequentially_to_cubic(self):
        degrees = [self.selector.update(self.line(3), timestamp_s=i/10).model_degree for i in range(30)]
        self.assertEqual(degrees[0], 1)
        self.assertIn(2, degrees)
        self.assertEqual(degrees[-1], 3)
        self.assertLess(degrees.index(2), degrees.index(3))

    def test_one_frame_curve_does_not_promote(self):
        self.selector.update(self.line(1), timestamp_s=0.)
        self.selector.update(self.line(3), timestamp_s=.1)
        result = self.selector.update(self.line(1), timestamp_s=.2)
        self.assertEqual(result.model_degree, 1)

    def test_returns_to_linear_and_temporal_terms_decay_to_zero(self):
        temporal = CenterlineTemporalFilter(measurement_following=True, reacquire_s=.1)
        degrees = []
        previous = None
        max_step = 0.
        for i in range(70):
            target = self.line(3 if i < 25 else 1)
            ordered = self.selector.update(target, timestamp_s=i/10)
            result = temporal.update(ordered, timestamp_s=i/10)
            degrees.append(ordered.model_degree)
            if previous is not None and result.valid and previous.valid:
                max_step = max(max_step, np.max(np.abs(temporal._values(result, self.x) - temporal._values(previous, self.x))))
            previous = result
        self.assertEqual(degrees[-1], 1)
        self.assertEqual(result.model_degree, 1)
        self.assertEqual(result.coefficients[2:], (0., 0.))
        self.assertLess(max_step, 2.)

    def test_gap_resets_to_linear_and_missing_does_not_promote(self):
        for i in range(20):
            self.selector.update(self.line(3), timestamp_s=i/10)
        self.assertEqual(self.selector.update(self.line(3), timestamp_s=3.).model_degree, 1)
        self.assertFalse(self.selector.update(replace(self.base, valid=False), timestamp_s=3.1).valid)
        self.assertFalse(self.selector.update(self.base, timestamp_s=3.).valid)

    def test_order_transition_does_not_lock_following_indefinitely(self):
        temporal = CenterlineTemporalFilter(measurement_following=True, reacquire_s=.1)
        first = replace(self.line(1), model_degree=1, target_degree=1)
        temporal.update(first, timestamp_s=0.)
        target = replace(self.line(2), model_degree=2, target_degree=2)
        for i in range(1, 20):
            result = temporal.update(target, timestamp_s=i/10)
        self.assertFalse(result.order_transition)
        self.assertLess(np.max(np.abs(temporal._values(result, self.x) - temporal._values(target, self.x))), .02)


if __name__ == '__main__':
    unittest.main()
