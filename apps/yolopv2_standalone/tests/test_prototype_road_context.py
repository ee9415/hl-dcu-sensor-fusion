"""prototype_road_context(경계선 양쪽 도로 영역 문맥 분석 시험 스크립트) 테스트."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from prototype_road_context import road_context


class RoadContextTest(unittest.TestCase):
    def setUp(self):
        """x=50~219 구간만 도로인 마스크와 세로 경계선 샘플 행을 준비한다."""
        self.road = np.zeros((180, 320), np.uint8)
        self.road[:, 50:220] = 1
        self.y = np.arange(100, 160, 2, dtype=float)
        self.support = np.ones(len(self.y), dtype=bool)

    def curve(self, x):
        """x 위치 고정 세로 곡선을 만든다."""
        return np.column_stack((np.full_like(self.y, x), self.y))

    def test_left_and_right_transition_orientation(self):
        """좌/우 경계선 모두 도로 안쪽에서 바깥쪽으로 바뀌는 방향을 올바르게 판정한다."""
        right = road_context(self.road, self.curve(219), self.support, 'right')
        left = road_context(self.road, self.curve(50), self.support, 'left')
        self.assertEqual(right['inside_to_outside_fraction'], 1.)
        self.assertEqual(left['inside_to_outside_fraction'], 1.)

    def test_interior_line_is_not_automatically_invalid(self):
        """도로 내부의 선은 양쪽 모두 도로로 보고될 뿐 자동으로 무효 처리되지 않는다."""
        result = road_context(self.road, self.curve(150), self.support, 'right')
        self.assertEqual(result['both_road_fraction'], 1.)
        self.assertEqual(result['inside_to_outside_fraction'], 0.)

    def test_out_of_frame_samples_are_reported_missing(self):
        """화면 밖으로 나가는 샘플은 누락으로 보고한다."""
        result = road_context(self.road, self.curve(317), self.support, 'right')
        self.assertEqual(result['rows'], 0)
        self.assertIsNone(result['inside_to_outside_fraction'])

    def test_wrong_shape_and_side_raise(self):
        """잘못된 마스크 크기나 side 값은 거부한다."""
        with self.assertRaises(ValueError):
            road_context(np.zeros((5, 5)), self.curve(100), self.support, 'right')
        with self.assertRaises(ValueError):
            road_context(self.road, self.curve(100), self.support, 'center')


if __name__ == '__main__':
    unittest.main()
