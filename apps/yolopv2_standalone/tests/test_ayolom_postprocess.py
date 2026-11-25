"""Verify YOLOv8 DFL geometry, added class IDs and two-class mask handling."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ayolom_postprocess import decode_prediction, segmentation_masks, segmentation_scores
from postprocess import non_max_suppression


class AyolomPostprocessTest(unittest.TestCase):
    def heads(self):
        return {f'{prefix}_stride_{stride}': np.full((1, channels, 384//stride, 672//stride), -100., np.float32)
                for prefix, channels in (('det', 65), ('person_tl', 68)) for stride in (8, 16, 32)}

    def test_uniform_dfl_has_analytic_geometry_without_objectness(self):
        heads = self.heads()
        heads['det_stride_8'][0, :64, 0, 0] = 0
        heads['det_stride_8'][0, 64, 0, 0] = 10
        prediction = decode_prediction(heads, (672, 384))
        # Mean of uniform bins 0..15 is 7.5, so each side is 7.5*8 pixels.
        np.testing.assert_allclose(prediction[0, :5], [4, 4, 120, 120, 1])
        decoded = non_max_suppression(prediction)
        self.assertEqual(decoded.shape, (1, 6))
        np.testing.assert_allclose(decoded[0, :4], [-56, -56, 64, 64])
        self.assertEqual(decoded[0, 5], 0)

    def test_added_local_class_maps_to_global_id_and_nhwc_matches(self):
        heads = self.heads()
        heads['person_tl_stride_16'][0, 64+2, 4, 7] = 10
        prediction = decode_prediction(heads, (672, 384))
        decoded = non_max_suppression(prediction)
        self.assertEqual(decoded.shape, (1, 6))
        self.assertEqual(decoded[0, 5], 3)  # Added local 2 = tl_yellow.
        alternate = {n:t.transpose(0, 2, 3, 1) for n,t in heads.items()}
        np.testing.assert_array_equal(decode_prediction(alternate, (672, 384)), prediction)

    def test_segmentation_removes_three_padding_rows_and_keeps_ros_geometry(self):
        logits = np.zeros((1, 2, 384, 672), np.float32)
        logits[:, 1, :3] = 100
        logits[:, 1, -3:] = 100
        road, lane = segmentation_masks(logits, logits, (1920, 1080), (672, 384))
        self.assertEqual(lane.shape, (180, 320))
        self.assertFalse(lane.any())  # Padding is removed, ties remain background.
        logits[:, 1, 3:381] = 3
        road, lane = segmentation_masks(logits, logits, (1920, 1080), (672, 384))
        self.assertTrue(np.all(road == 255))
        self.assertTrue(np.all(lane == 255))
        self.assertTrue(np.all(segmentation_scores(logits, (1920,1080), (672,384)) > .95))

    def test_nonfinite_and_wrong_shapes_rejected(self):
        heads = self.heads()
        heads['det_stride_8'][0, 0, 0, 0] = np.nan
        with self.assertRaises(ValueError):
            decode_prediction(heads, (672,384))
        with self.assertRaises(ValueError):
            segmentation_scores(np.zeros((1,1,384,672)), (1920,1080), (672,384))


if __name__ == '__main__':
    unittest.main()
