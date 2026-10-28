import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from prototype_directional_evidence import inspect_candidate, synthetic


class DirectionalEvidenceTest(unittest.TestCase):
    def test_synthetic_transverse_and_longitudinal_paint_differ(self):
        data = synthetic()
        self.assertGreater(data['longitudinal_paint']['longitudinal_fraction'], .8)
        self.assertEqual(data['transverse_paint']['longitudinal_fraction'], 0.)

    def test_identical_geometry_cannot_prove_semantics(self):
        data = synthetic()
        self.assertEqual(data['longitudinal_paint'],
                         data['visually_identical_wrong_longitudinal_paint'])

    def test_too_few_observed_rows_is_not_confident(self):
        points = np.column_stack((np.full(7, 100.), np.arange(100, 107)))
        result = inspect_candidate(np.zeros((180, 320)), points, np.ones(7, bool))
        self.assertIsNone(result['longitudinal_fraction'])

    def test_rejects_bad_mask_shape(self):
        with self.assertRaises(ValueError):
            inspect_candidate(np.zeros((10, 10)), np.zeros((10, 2)), np.ones(10, bool))


if __name__ == '__main__':
    unittest.main()
