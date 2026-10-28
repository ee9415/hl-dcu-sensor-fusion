import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from prototype_paint_support_gate import support_fraction, synthetic


class PrototypePaintSupportTest(unittest.TestCase):
    def test_supported_curve_is_counted_and_missing_paint_is_not(self):
        y = np.linspace(80, 170, 46)
        points = np.column_stack((np.full_like(y, 200), y))
        obs = (points, .8, np.ones(len(y), bool))
        paint = np.zeros((180, 320), dtype=np.uint8)
        fraction, count = support_fraction(paint, obs)
        self.assertEqual((fraction, count), (0., 46))
        paint[np.rint(y).astype(int), 200] = 1
        fraction, count = support_fraction(paint, obs)
        self.assertEqual((fraction, count), (1., 46))

    def test_unpainted_false_pair_cannot_use_fast_pair_rebase(self):
        baseline, _ = synthetic(False, False)
        gated, _ = synthetic(True, False)
        self.assertAlmostEqual(baseline[-1], 245.)
        self.assertAlmostEqual(gated[-1], 224.)

    def test_painted_false_pair_exposes_remaining_limitation(self):
        gated, _ = synthetic(True, True)
        self.assertAlmostEqual(gated[-1], 245.)


if __name__ == '__main__':
    unittest.main()
