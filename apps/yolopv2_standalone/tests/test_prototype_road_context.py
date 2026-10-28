import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from prototype_road_context import road_context


class RoadContextTest(unittest.TestCase):
    def setUp(self):
        self.road = np.zeros((180, 320), np.uint8)
        self.road[:, 50:220] = 1
        self.y = np.arange(100, 160, 2, dtype=float)
        self.support = np.ones(len(self.y), dtype=bool)

    def curve(self, x):
        return np.column_stack((np.full_like(self.y, x), self.y))

    def test_left_and_right_transition_orientation(self):
        right = road_context(self.road, self.curve(219), self.support, 'right')
        left = road_context(self.road, self.curve(50), self.support, 'left')
        self.assertEqual(right['inside_to_outside_fraction'], 1.)
        self.assertEqual(left['inside_to_outside_fraction'], 1.)

    def test_interior_line_is_not_automatically_invalid(self):
        result = road_context(self.road, self.curve(150), self.support, 'right')
        self.assertEqual(result['both_road_fraction'], 1.)
        self.assertEqual(result['inside_to_outside_fraction'], 0.)

    def test_out_of_frame_samples_are_reported_missing(self):
        result = road_context(self.road, self.curve(317), self.support, 'right')
        self.assertEqual(result['rows'], 0)
        self.assertIsNone(result['inside_to_outside_fraction'])

    def test_wrong_shape_and_side_raise(self):
        with self.assertRaises(ValueError):
            road_context(np.zeros((5, 5)), self.curve(100), self.support, 'right')
        with self.assertRaises(ValueError):
            road_context(self.road, self.curve(100), self.support, 'center')


if __name__ == '__main__':
    unittest.main()
