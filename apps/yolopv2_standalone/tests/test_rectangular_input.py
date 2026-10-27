import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from postprocess import map_letterboxed_box_to_source, segmentation_masks


class RectangularInputTest(unittest.TestCase):
    def test_square_default_matches_original_geometry(self):
        rng = np.random.default_rng(42)
        road = rng.random((1, 2, 320, 320), dtype=np.float32)
        lane = rng.random((1, 1, 320, 320), dtype=np.float32)
        actual_road, actual_lane = segmentation_masks(road, lane, (1920, 1080))
        np.testing.assert_array_equal(actual_road, (np.argmax(road, axis=1)[0] * 255).astype(np.uint8)[70:250])
        np.testing.assert_array_equal(actual_lane, ((lane[0, 0] >= .5) * 255).astype(np.uint8)[70:250])
        box = np.array([32, 88, 288, 232], dtype=np.float32)
        np.testing.assert_array_equal(map_letterboxed_box_to_source(box, (1920, 1080)),
                                      np.array([.1, .1, .9, .9], dtype=np.float32))

    def test_rectangular_crop_and_box_mapping(self):
        road = np.zeros((1, 2, 192, 320), dtype=np.float32)
        road[:, 1, 6:186] = 1
        lane = np.zeros((1, 1, 192, 320), dtype=np.float32)
        lane[:, :, 6:186, 100] = 1
        r, l = segmentation_masks(road, lane, (1920, 1080), (320, 192))
        self.assertEqual(r.shape, (180, 320))
        self.assertTrue(np.all(r == 255))
        self.assertTrue(np.all(l[:, 100] == 255))
        box = np.array([32, 24, 288, 168], dtype=np.float32)
        np.testing.assert_allclose(map_letterboxed_box_to_source(box, (1920, 1080), (320, 192)),
                                   [.1, .1, .9, .9], rtol=0, atol=1e-7)

    def test_wrong_output_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            segmentation_masks(np.zeros((1, 2, 192, 320)), np.zeros((1, 1, 192, 320)),
                               (1920, 1080))


if __name__ == '__main__':
    unittest.main()
