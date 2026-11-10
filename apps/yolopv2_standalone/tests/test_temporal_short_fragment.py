"""ImageBoundaryTracker._temporal_fragments(짧은 차선 조각의 시간적 누적 복구) 테스트."""
import unittest

import numpy as np

from image_boundaries import ImageBoundaryTracker


class TemporalShortFragmentTests(unittest.TestCase):
    def setUp(self):
        """기울기 0.2인 이전(prior) 경계선을 준비한다."""
        self.tracker = ImageBoundaryTracker()
        self.prior = np.column_stack((80 + .2 * (self.tracker.rows - 125), self.tracker.rows))

    def fragment(self, offset=2, slope=.2):
        """y=138~148 구간에 prior와 비슷한 짧은 조각 하나를 영역 세그먼트로 넣는다."""
        y = np.arange(138, 150, 2)
        self.tracker.region_segments[0] = [np.column_stack((80 + slope * (y - 125) + offset, y))]

    def test_repeated_fragment_is_local_low_confidence_support(self):
        """같은 조각이 3프레임 반복되면 해당 구간에만 낮은 신뢰도(0.45)의 지지점으로 복구한다."""
        self.fragment()
        for _ in range(2):
            self.assertIsNone(self.tracker._temporal_fragments(None, 0, self.prior))
        curve, confidence, support, temporal = self.tracker._temporal_fragments(None, 0, self.prior)
        self.assertEqual(confidence, .45)
        self.assertTrue(temporal)
        self.assertEqual(support.sum(), 6)
        self.assertTrue(np.isnan(curve[~support, 0]).all())
        np.testing.assert_allclose(curve[support, 0] - self.prior[support, 0], 2)

    def test_missing_changed_or_wrong_direction_does_not_recover(self):
        """방향이 다르거나, 위치가 매번 바뀌거나, 조각이 없으면 복구하지 않는다."""
        for _ in range(4):
            self.fragment(slope=1.3)
            self.assertIsNone(self.tracker._temporal_fragments(None, 0, self.prior))
        for offset in (1, 5, 1, 5):
            self.fragment(offset=offset)
            self.assertIsNone(self.tracker._temporal_fragments(None, 0, self.prior))
        self.tracker.region_segments[0] = []
        self.assertIsNone(self.tracker._temporal_fragments(None, 0, self.prior))


if __name__ == '__main__':
    unittest.main()
