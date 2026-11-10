"""prototype_paint_support_gate(페인트 지지율 게이트 시험 스크립트) 테스트."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from prototype_paint_support_gate import support_fraction, synthetic


class PrototypePaintSupportTest(unittest.TestCase):
    def test_supported_curve_is_counted_and_missing_paint_is_not(self):
        """곡선 위 페인트 비율을 센다. 페인트가 없으면 0, 전부 있으면 1."""
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
        """페인트 없는 거짓 쌍은 게이트가 켜져 있으면 빠른 재설정에 쓰이지 못한다(245 → 224 유지)."""
        baseline, _ = synthetic(False, False)
        gated, _ = synthetic(True, False)
        self.assertAlmostEqual(baseline[-1], 245.)
        self.assertAlmostEqual(gated[-1], 224.)

    def test_painted_false_pair_exposes_remaining_limitation(self):
        """페인트가 있는 거짓 쌍은 게이트로도 막지 못한다는 남은 한계를 기록한다."""
        gated, _ = synthetic(True, True)
        self.assertAlmostEqual(gated[-1], 245.)


if __name__ == '__main__':
    unittest.main()
