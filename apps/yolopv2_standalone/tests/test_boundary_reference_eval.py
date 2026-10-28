import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from evaluate_boundary_references import boundary_x, evaluate


class BoundaryReferenceTest(unittest.TestCase):
    def test_interpolates_without_extrapolating_or_bridging_large_gaps(self):
        points = [[3, 10], [5, 12], [20, 30]]
        self.assertEqual(boundary_x(points, 11), 4)
        self.assertIsNone(boundary_x(points, 9))
        self.assertIsNone(boundary_x(points, 20))

    def test_missing_prediction_is_not_a_zero_error(self):
        labels = {'frames': [{'source_frame_index': 1, 'source_media_epoch': 0,
                              'left': [[20, 10]], 'right': [[30, 10]]}]}
        predictions = [{'input_source': 'pc', 'source_frame_index': 1, 'source_media_epoch': 0,
                        'left_boundary_image': [[18, 10], [20, 12]], 'right_boundary_image': []}]
        rows = evaluate(labels, predictions)
        self.assertEqual(rows[0]['abs_error_px'], 2)
        self.assertIsNone(rows[1]['abs_error_px'])

    def test_duplicate_source_identity_is_rejected(self):
        labels = {'frames': [{'source_frame_index': 1, 'source_media_epoch': 0, 'left': [[20, 10]]}]}
        p = {'input_source': 'pc', 'source_frame_index': 1, 'source_media_epoch': 0}
        with self.assertRaises(ValueError):
            evaluate(labels, [p, p])


if __name__ == '__main__':
    unittest.main()
