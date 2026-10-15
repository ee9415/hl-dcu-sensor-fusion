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

    def test_temporal_filter_smooths_ordinary_curve_change(self) -> None:
        straight = estimate_centerline(trapezoid_mask())
        curved = estimate_centerline(trapezoid_mask(curve_strength=8.0))
        tracker = CenterlineTemporalFilter(alpha=0.25, max_curve_jump_px=200.0)

        first = tracker.update(straight)
        smoothed = tracker.update(curved)

        expected = first.bev_curve[:, 0] * 0.75 + curved.bev_curve[:, 0] * 0.25
        np.testing.assert_allclose(smoothed.bev_curve[:, 0], expected, atol=1e-4)

    def test_temporal_filter_rejects_short_large_jump(self) -> None:
        straight = estimate_centerline(trapezoid_mask())
        curved = estimate_centerline(trapezoid_mask(curve_strength=18.0))
        tracker = CenterlineTemporalFilter(max_curve_jump_px=5.0, hold_frames=2)

        tracker.update(straight)
        first_rejected = tracker.update(curved)
        second_rejected = tracker.update(curved)
        accepted = tracker.update(curved)

        np.testing.assert_array_equal(first_rejected.bev_curve, straight.bev_curve)
        np.testing.assert_array_equal(second_rejected.bev_curve, straight.bev_curve)
        np.testing.assert_array_equal(accepted.bev_curve, curved.bev_curve)

    def test_temporal_filter_holds_brief_invalid_measurement(self) -> None:
        straight = estimate_centerline(trapezoid_mask())
        invalid = estimate_centerline(np.zeros((180, 320), dtype=np.uint8))
        tracker = CenterlineTemporalFilter(hold_frames=2)

        tracker.update(straight)

        self.assertTrue(tracker.update(invalid).valid)
        self.assertTrue(tracker.update(invalid).valid)
        self.assertFalse(tracker.update(invalid).valid)


if __name__ == "__main__":
    unittest.main()
