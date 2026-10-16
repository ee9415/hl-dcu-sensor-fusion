import unittest
from dataclasses import replace
import numpy as np
from conservative_centerline import ConservativeCenterline
from centerline import estimate_centerline
from test_centerline import trapezoid_mask


class ConservativeTests(unittest.TestCase):
    def test_rates_cubic_and_changing_visible_range(self):
        base = estimate_centerline(trapezoid_mask())
        limiter = ConservativeCenterline()
        rng = np.random.default_rng(14)
        previous = None
        t = 0.
        for i in range(250):
            dt = float(rng.uniform(.02, .3))
            t += dt
            candidate = replace(base, coefficients=tuple(rng.normal(size=4)*[1,.1,.01,.001]),
                                forward_range_m=(float(rng.uniform(0,10)), 40.))
            result = limiter.update(candidate, timestamp_s=t)
            self.assertEqual(result.coefficients[3], 0.)
            self.assertLessEqual(abs(2*result.coefficients[2]), limiter.curvature_limit+1e-10)
            if previous is not None:
                rates = limiter.bounds(np.array(result.coefficients)-previous)/dt
                self.assertTrue(np.all(rates <= np.array([.5,.05,.005])+1e-9))
                grid = np.linspace(0, 40, 1001)
                current = np.array(result.coefficients)
                slope = current[1] + 2*current[2]*grid
                old_slope = previous[1] + 2*previous[2]*grid
                heading_rate = np.max(np.abs(np.arctan(slope)-np.arctan(old_slope)))/dt
                curvature_rate = np.max(np.abs(2*current[2]/(1+slope*slope)**1.5
                                               - 2*previous[2]/(1+old_slope*old_slope)**1.5))/dt
                self.assertLessEqual(heading_rate, .05+1e-9)
                self.assertLessEqual(curvature_rate, .005+1e-9)
                self.assertLessEqual(np.max(np.abs(slope)), limiter.slope_limit+1e-9)
            previous = np.array(result.coefficients)

    def test_gap_and_invalid_do_not_reset_rate_limit(self):
        base = estimate_centerline(trapezoid_mask())
        limiter = ConservativeCenterline()
        first = limiter.update(base, timestamp_s=0.)
        self.assertFalse(limiter.update(replace(base, valid=False), timestamp_s=.1).valid)
        result = limiter.update(replace(base, coefficients=(4.,.1,.01,0.)), timestamp_s=10.)
        self.assertLessEqual(limiter.bounds(np.array(result.coefficients)-first.coefficients)[0], .05+1e-9)
        self.assertFalse(limiter.update(base, timestamp_s=9.).valid)

    def test_straight_remains_straight(self):
        base = replace(estimate_centerline(trapezoid_mask()), coefficients=(0.,0.,0.,0.), model_degree=1)
        result = ConservativeCenterline().update(base, timestamp_s=0.)
        self.assertEqual(result.coefficients, (0.,0.,0.,0.))


if __name__ == '__main__':
    unittest.main()
