import unittest

import numpy as np

from centerline import (
    CenterlineTemporalFilter,
    estimate_centerline,
    render_bev,
)


def trapezoid_mask(curve_strength: float = 0.0) -> np.ndarray:
    height, width = 180, 320
    mask = np.zeros((height, width), dtype=np.uint8)
    for y in range(90, 177):
        progress = (y - 90) / (176 - 90)
        half_width = 30 + 110 * progress
        bend = curve_strength * (1.0 - progress) ** 2
        center = width * 0.5 + bend
        left = int(round(center - half_width))
        right = int(round(center + half_width))
        mask[y, max(left, 0) : min(right + 1, width)] = 255
    return mask


class CenterlineTests(unittest.TestCase):
    def test_straight_drivable_area_produces_centerline(self) -> None:
        estimate = estimate_centerline(trapezoid_mask())

        self.assertTrue(estimate.valid)
        self.assertLess(abs(estimate.coefficients[0]), 0.1)
        self.assertLess(estimate.residual_m, 0.1)
        self.assertGreater(estimate.forward_range_m[1], 35.0)

    def test_curved_drivable_area_produces_nonzero_polynomial(self) -> None:
        estimate = estimate_centerline(trapezoid_mask(curve_strength=18.0))

        self.assertTrue(estimate.valid)
        self.assertGreater(np.max(np.abs(estimate.bev_curve[:, 0] - 319.5)), 20.0)
        self.assertTrue(np.all(np.isfinite(estimate.coefficients)))

    def test_blank_mask_is_invalid(self) -> None:
        estimate = estimate_centerline(np.zeros((180, 320), dtype=np.uint8))

        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.bev_curve.shape, (0, 2))

    def test_bev_render_contains_fitted_curve(self) -> None:
        estimate = estimate_centerline(trapezoid_mask())
        frame = render_bev(estimate)

        self.assertEqual(frame.shape, (480, 640, 3))
        cyan = np.all(frame == np.array((255, 255, 0), dtype=np.uint8), axis=2)
        self.assertGreater(int(np.count_nonzero(cyan)), 100)



if __name__ == "__main__":
    unittest.main()
