"""postprocess.estimate_spatial_from_depth(박스 영역 깊이로 3D 위치 추정) 테스트."""
import unittest

import numpy as np

from postprocess import estimate_spatial_from_depth


class SpatialMeasurementTests(unittest.TestCase):
    def setUp(self) -> None:
        """200x100 영상 기준 카메라 내부 파라미터(fx=fy=100, 주점은 영상 중앙)."""
        self.intrinsics = np.array(
            (
                (100.0, 0.0, 99.5),
                (0.0, 100.0, 49.5),
                (0.0, 0.0, 1.0),
            ),
            dtype=np.float64,
        )

    def test_centered_constant_depth_returns_forward_distance(self) -> None:
        """중앙 박스 + 균일 깊이 → x,y=0, z=깊이 값이고 ROI는 박스 중앙부."""
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
        """유효 깊이 픽셀이 너무 적으면 결과를 무효(z=NaN)로 처리한다."""
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
        """이상치(먼 값, 0)가 섞여도 중앙값 기반이라 깊이 추정이 흔들리지 않는다."""
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
        """오른쪽에 치우친 박스는 핀홀 모델대로 x 좌표(약 4975mm)를 계산한다."""
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
