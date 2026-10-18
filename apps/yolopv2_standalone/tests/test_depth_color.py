import unittest
import numpy as np
from depth_color import colorize_depth


class DepthColorTests(unittest.TestCase):
    def test_invalid_black_and_range_endpoints(self):
        depth = np.array([[0, np.nan, np.inf, -1, 300, 25000, 50000]])
        result = colorize_depth(depth)
        np.testing.assert_array_equal(result[0, :4], np.zeros((4, 3), dtype=np.uint8))
        np.testing.assert_array_equal(result[0, 4], [0, 0, 255])
        np.testing.assert_array_equal(result[0, 5:], [[255, 0, 0], [255, 0, 0]])

    def test_fixed_scale_preserves_input_and_alignment(self):
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
        colors = colorize_depth(np.array([[10000, 15000, 20000, 25000]], dtype=np.uint16))[0]
        self.assertEqual(len(np.unique(colors, axis=0)), 4)
        np.testing.assert_array_equal(colors[-1], [255, 0, 0])

    def test_rejects_bad_shape(self):
        for raw in (np.array([]), np.zeros((0, 3)), np.zeros((3, 3, 3))):
            with self.assertRaises(ValueError):
                colorize_depth(raw)
