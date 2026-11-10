"""depth_color.colorize_depth(깊이 mm → 컬러 시각화) 테스트."""
import unittest
import numpy as np
from depth_color import colorize_depth


class DepthColorTests(unittest.TestCase):
    def test_invalid_black_and_range_endpoints(self):
        """0/NaN/inf/음수는 검정, 최소 거리 이하는 빨강(BGR 0,0,255), 최대 거리 이상은 파랑으로 고정된다."""
        depth = np.array([[0, np.nan, np.inf, -1, 300, 25000, 50000]])
        result = colorize_depth(depth)
        np.testing.assert_array_equal(result[0, :4], np.zeros((4, 3), dtype=np.uint8))
        np.testing.assert_array_equal(result[0, 4], [0, 0, 255])
        np.testing.assert_array_equal(result[0, 5:], [[255, 0, 0], [255, 0, 0]])

    def test_fixed_scale_preserves_input_and_alignment(self):
        """고정 스케일이라 다운샘플한 영상에서도 같은 깊이는 같은 색이고, 입력 배열은 변경되지 않는다."""
        raw = np.zeros((1080, 1920), np.uint16)
        raw[300:600, 600:900] = 1000
        before = raw.copy()
        result = colorize_depth(raw[::3, ::3])
        self.assertEqual(result.shape, (360, 640, 3))
        self.assertTrue(result.flags.c_contiguous)
        self.assertEqual(result.dtype, np.uint8)
        self.assertTrue(np.any(result[100:200, 200:300]))
        self.assertFalse(np.any(result[:100]))
        np.testing.assert_array_equal(raw, before)
        np.testing.assert_array_equal(result[150, 250], colorize_depth(np.array([[1000]]))[0, 0])

    def test_distinguishes_ten_to_twenty_five_meters(self):
        """10~25m 구간의 깊이도 서로 다른 색으로 구분된다."""
        colors = colorize_depth(np.array([[10000, 15000, 20000, 25000]], dtype=np.uint16))[0]
        self.assertEqual(len(np.unique(colors, axis=0)), 4)
        np.testing.assert_array_equal(colors[-1], [255, 0, 0])

    def test_rejects_bad_shape(self):
        """빈 배열이나 2차원이 아닌 입력은 거부한다."""
        for raw in (np.array([]), np.zeros((0, 3)), np.zeros((3, 3, 3))):
            with self.assertRaises(ValueError):
                colorize_depth(raw)
