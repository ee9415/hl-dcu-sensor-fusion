import unittest
from dataclasses import replace
import numpy as np
from centerline import estimate_centerline
from far_corridor import guide_to_far_corridor
from conservative_centerline import ConservativeCenterline


def distant_road():
    mask = np.zeros((180, 320), np.uint8)
    for row in range(72, 177):
        t = (row-72)/104
        center = 136+30*t
        half = 9+110*t
        mask[row, int(center-half):int(center+half)+1] = 255
    return mask


class FarCorridorTests(unittest.TestCase):
    def test_far_goal_above_previous_roi(self):
        mask = distant_road()
        old = estimate_centerline(mask)
        extended = estimate_centerline(mask, roi_top_fraction=.38)
        guided = guide_to_far_corridor(extended, extended)
        self.assertTrue(guided.valid)
        self.assertLess(guided.goal_image[1], 85)
        self.assertGreaterEqual(old.image_curve[:, 1].min(), 89)
        self.assertLess(abs(np.polynomial.polynomial.polyval(guided.goal_position[0], guided.coefficients)
                            -guided.goal_position[1]), .15)
        self.assertEqual(guided.coefficients[3], 0.)

    def test_widened_near_region_does_not_move_far_goal(self):
        mask = distant_road()
        first = estimate_centerline(mask, roi_top_fraction=.38)
        mask[130:177, 230:300] = 255
        widened = estimate_centerline(mask, roi_top_fraction=.38)
        a, b = guide_to_far_corridor(first, first), guide_to_far_corridor(widened, widened)
        np.testing.assert_allclose(a.goal_image, b.goal_image)

    def test_disconnected_upper_region_not_claimed_as_goal(self):
        mask = distant_road()
        mask[82:95] = 0
        observed = estimate_centerline(mask, roi_top_fraction=.38)
        guided = guide_to_far_corridor(observed, observed)
        self.assertGreater(guided.goal_image[1], 94)

    def test_wide_far_patch_does_not_replace_narrow_passage(self):
        mask = distant_road()
        mask[72:81, 60:260] = 255
        observed = estimate_centerline(mask, roi_top_fraction=.38)
        guided = guide_to_far_corridor(observed, observed)
        self.assertTrue(observed.valid)
        self.assertGreater(guided.goal_image[1], 81)
        self.assertLess(guided.goal_image[0], 145)

    def test_rate_limits_survive_goal_change(self):
        observed = estimate_centerline(distant_road(), roi_top_fraction=.38)
        limiter = ConservativeCenterline()
        previous = None
        for i in range(250):
            if i == 50:
                observed = estimate_centerline(np.roll(distant_road(), 20, axis=1), roi_top_fraction=.38)
            target = guide_to_far_corridor(observed, observed, limiter.previous)
            result = limiter.update(target, timestamp_s=i*.1)
            if previous is not None:
                rate = limiter.bounds(np.array(result.coefficients)-previous)/.1
                self.assertTrue(np.all(rate <= np.array([.5,.05,.005])+1e-8))
            previous = np.array(result.coefficients)
        gx, gy = target.goal_position
        self.assertLess(abs(np.polynomial.polynomial.polyval(gx, result.coefficients)-gy), .15)

    def test_invalid_input_and_straight(self):
        observed = estimate_centerline(distant_road(), roi_top_fraction=.38)
        invalid = replace(observed, valid=False)
        self.assertIsNone(guide_to_far_corridor(invalid, invalid).goal_position)
        with self.assertRaises(ValueError):
            estimate_centerline(distant_road(), roi_top_fraction=.1)


if __name__ == '__main__':
    unittest.main()
