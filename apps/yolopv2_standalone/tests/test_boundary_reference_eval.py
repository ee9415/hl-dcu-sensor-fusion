"""evaluate_boundary_references(수동 라벨 대비 경계선 예측 오차 평가 스크립트) 단위 테스트."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from evaluate_boundary_references import boundary_x, evaluate


class BoundaryReferenceTest(unittest.TestCase):
    def test_interpolates_without_extrapolating_or_bridging_large_gaps(self):
        """라벨 점 사이는 선형 보간하지만, 범위 밖 외삽이나 큰 간격(여기선 y=12~30) 메우기는 하지 않는다."""
        points = [[3, 10], [5, 12], [20, 30]]
        # y=11은 (3,10)~(5,12) 사이 → x=4로 보간
        self.assertEqual(boundary_x(points, 11), 4)
        self.assertIsNone(boundary_x(points, 9))
        self.assertIsNone(boundary_x(points, 20))

    def test_missing_prediction_is_not_a_zero_error(self):
        """예측이 없는 쪽(right)은 오차 0이 아니라 None으로 기록되어야 한다."""
        labels = {'frames': [{'source_frame_index': 1, 'source_media_epoch': 0,
                              'left': [[20, 10]], 'right': [[30, 10]]}]}
        predictions = [{'input_source': 'pc', 'source_frame_index': 1, 'source_media_epoch': 0,
                        'left_boundary_image': [[18, 10], [20, 12]], 'right_boundary_image': []}]
        rows = evaluate(labels, predictions)
        self.assertEqual(rows[0]['abs_error_px'], 2)
        self.assertIsNone(rows[1]['abs_error_px'])

    def test_duplicate_source_identity_is_rejected(self):
        """같은 (소스, 프레임 인덱스, epoch) 예측이 중복되면 평가를 거부한다."""
        labels = {'frames': [{'source_frame_index': 1, 'source_media_epoch': 0, 'left': [[20, 10]]}]}
        p = {'input_source': 'pc', 'source_frame_index': 1, 'source_media_epoch': 0}
        with self.assertRaises(ValueError):
            evaluate(labels, [p, p])


if __name__ == '__main__':
    unittest.main()
