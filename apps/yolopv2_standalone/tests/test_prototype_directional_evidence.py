"""prototype_directional_evidence(차선 페인트 방향성 근거 시험 스크립트) 테스트."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/hl_camera_bringup/scripts'))
from prototype_directional_evidence import inspect_candidate, synthetic


class DirectionalEvidenceTest(unittest.TestCase):
    def test_synthetic_transverse_and_longitudinal_paint_differ(self):
        """합성 데이터에서 종방향 페인트와 횡방향 페인트의 종방향 비율이 확연히 다르다."""
        data = synthetic()
        self.assertGreater(data['longitudinal_paint']['longitudinal_fraction'], .8)
        self.assertEqual(data['transverse_paint']['longitudinal_fraction'], 0.)

    def test_identical_geometry_cannot_prove_semantics(self):
        """기하가 같으면 의미(올바른/잘못된 차선)가 달라도 같은 결과가 나온다 → 기하만으로는 의미를 증명할 수 없다는 한계를 기록."""
        data = synthetic()
        self.assertEqual(data['longitudinal_paint'],
                         data['visually_identical_wrong_longitudinal_paint'])

    def test_too_few_observed_rows_is_not_confident(self):
        """관측 행이 너무 적으면 비율을 내지 않는다(None)."""
        points = np.column_stack((np.full(7, 100.), np.arange(100, 107)))
        result = inspect_candidate(np.zeros((180, 320)), points, np.ones(7, bool))
        self.assertIsNone(result['longitudinal_fraction'])

    def test_rejects_bad_mask_shape(self):
        """마스크 크기가 예상(180x320)과 다르면 거부한다."""
        with self.assertRaises(ValueError):
            inspect_candidate(np.zeros((10, 10)), np.zeros((10, 2)), np.ones(10, bool))


if __name__ == '__main__':
    unittest.main()
