import unittest

import numpy as np
from image_boundaries import ImageBoundaryTracker


class FitEvidenceTraceTest(unittest.TestCase):
    def stripe(self):
        paint = np.zeros((180, 320), dtype=bool)
        for y in range(80, 169, 2):
            if not 120 <= y <= 132:
                paint[y, 100] = True
        return paint

    def test_opt_in_evidence_separates_fit_inliers_from_curve_gap(self):
        paint = self.stripe()
        baseline = ImageBoundaryTracker()
        expected = baseline._detect_full(paint, 0, None)
        self.assertNotIn('fit_observed_points', baseline.detection_diagnostics[0])
        traced = ImageBoundaryTracker()
        traced.trace_fit_evidence = True
        actual = traced._detect_full(paint, 0, None)
        np.testing.assert_array_equal(actual[0], expected[0])
        self.assertEqual(actual[1], expected[1])
        stats = traced.detection_diagnostics[0]
        inliers = stats['fit_observed_points']
        self.assertTrue(all(paint[y, int(x)] for x, y in inliers))
        self.assertTrue(any(120 <= y <= 132 for y in stats['fit_curve_rows']))
        self.assertFalse(any(120 <= y <= 132 for _, y in inliers))

    def test_missing_candidate_has_no_inliers(self):
        tracker = ImageBoundaryTracker()
        tracker.trace_fit_evidence = True
        self.assertIsNone(tracker._detect_full(np.zeros((180, 320), bool), 0, None))
        self.assertNotIn('fit_observed_points', tracker.detection_diagnostics[0])


if __name__ == '__main__':
    unittest.main()
