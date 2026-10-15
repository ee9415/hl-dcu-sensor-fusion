import unittest

import numpy as np

from postprocess import estimate_spatial_from_depth


class SpatialMeasurementTests(unittest.TestCase):
    def setUp(self) -> None:
        self.intrinsics = np.array(
            (
                (100.0, 0.0, 99.5),
                (0.0, 100.0, 49.5),
                (0.0, 0.0, 1.0),
            ),
            dtype=np.float64,
        )

    def test_centered_constant_depth_returns_forward_distance(self) -> None:
        depth = np.full((100, 200), 12_000, dtype=np.uint16)

        result = estimate_spatial_from_depth(
            depth,
            (0.4, 0.4, 0.6, 0.6),
            self.intrinsics,
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.x_mm, 0.0)
        self.assertAlmostEqual(result.y_mm, 0.0)
        self.assertAlmostEqual(result.z_mm, 12_000.0)
        self.assertEqual(result.roi, (90, 45, 111, 56))

    def test_sparse_depth_is_rejected(self) -> None:
        depth = np.zeros((100, 200), dtype=np.uint16)
        depth[48:50, 98:103] = 5_000

        result = estimate_spatial_from_depth(
            depth,
            (0.4, 0.4, 0.6, 0.6),
            self.intrinsics,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.valid_pixels, 10)
        self.assertTrue(np.isnan(result.z_mm))

    def test_outliers_do_not_move_median_depth(self) -> None:
        depth = np.full((100, 200), 8_000, dtype=np.uint16)
        depth[45:48, 90:95] = 45_000
        depth[48:51, 95:100] = 0

        result = estimate_spatial_from_depth(
            depth,
            (0.4, 0.4, 0.6, 0.6),
            self.intrinsics,
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.z_mm, 8_000.0)

    def test_off_center_box_returns_horizontal_coordinate(self) -> None:
        depth = np.full((100, 200), 10_000, dtype=np.uint16)

        result = estimate_spatial_from_depth(
            depth,
            (0.7, 0.4, 0.8, 0.6),
            self.intrinsics,
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.x_mm, 4_975.0)
        self.assertAlmostEqual(result.y_mm, 0.0)
        self.assertAlmostEqual(result.z_mm, 10_000.0)


if __name__ == "__main__":
    unittest.main()
