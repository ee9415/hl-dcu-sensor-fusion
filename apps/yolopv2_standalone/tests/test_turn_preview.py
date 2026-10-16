import json
from dataclasses import replace
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from centerline import estimate_centerline
from test_centerline import trapezoid_mask
from turn_preview import TurnPreview, TurnMailbox, road_samples, turn_samples, draw_turn


class TurnTests(unittest.TestCase):
    def setUp(self):
        self.base = replace(estimate_centerline(trapezoid_mask()), coefficients=(0., 0., 0., 0.))

    def test_default_road_unchanged(self):
        planner = TurnPreview()
        self.assertIsNone(planner.update(self.base, 0.))
        self.assertEqual(planner.mode, 'road')

    def test_left_right_endpoint_and_mirror(self):
        base, left = turn_samples(self.base, 'left')
        _, right = turn_samples(self.base, 'right')
        np.testing.assert_allclose(left[:41], base[:41])
        np.testing.assert_allclose(left[:, 0], right[:, 0])
        np.testing.assert_allclose(left[:, 1], -right[:, 1])
        for points, sign in ((left, -1), (right, 1)):
            tangent = points[-1]-points[-2]
            self.assertAlmostEqual(np.arctan2(tangent[1], tangent[0]), sign*np.pi/2, delta=.001)
            self.assertLess(np.max(np.abs(TurnPreview.curvature(points)[38:42])), .005)

    def test_curved_entry_matches_heading_and_curvature(self):
        measured = replace(self.base, coefficients=(.3, .04, .002, 0.))
        base, target = turn_samples(measured, 'right')
        np.testing.assert_allclose(base[:41], target[:41])
        x = base[40, 0]
        slope = .04+.004*x
        expected = .004/(1+slope*slope)**1.5
        self.assertAlmostEqual(TurnPreview.curvature(target)[39], expected, delta=.002)

    def test_per_frame_limits_and_return(self):
        planner = TurnPreview()
        planner.request('left', self.base)
        previous = planner.points.copy()
        for i in range(1700):
            result = planner.update(self.base, i*.1)
            self.assertIsNotNone(result)
            self.assertLessEqual(np.max(np.linalg.norm(result-previous, axis=1)), .05+1e-9)
            self.assertLessEqual(np.max(np.abs(planner.curvature(result)-planner.curvature(previous))), .0005+1e-9)
            old, new = np.diff(previous, axis=0), np.diff(result, axis=0)
            angle = np.arctan2(old[:, 0]*new[:, 1]-old[:, 1]*new[:, 0], (old*new).sum(axis=1))
            self.assertLessEqual(np.max(np.abs(angle)), .005+1e-9)
            previous = result.copy()
        self.assertEqual(planner.status, 'preview_only')
        planner.request('road', self.base)
        for i in range(1700, 3400):
            planner.update(self.base, i*.1)
        self.assertEqual(planner.mode, 'road')

    def test_invalid_and_repeated_command(self):
        planner = TurnPreview()
        with self.assertRaises(ValueError):
            planner.request('left', replace(self.base, valid=False))
        planner.request('left', self.base)
        with self.assertRaises(ValueError):
            planner.request('right', self.base)
        self.assertIsNone(planner.update(replace(self.base, valid=False), 0.))
        self.assertIsNone(planner.update(self.base, 0.))

    def test_mailbox_ack_expiry_and_restart(self):
        with tempfile.TemporaryDirectory(dir='.') as directory:
            root = Path(directory)
            (root/'old.request').write_text('{}')
            mailbox = TurnMailbox(root)
            (root/'new.request').write_text(json.dumps({'mode': 'right', 'created': time.time()}))
            (root/'expired.request').write_text(json.dumps({'mode': 'left', 'created': time.time()-30}))
            planner = TurnPreview()
            mailbox.poll(planner, self.base)
            self.assertFalse((root/'old.response').exists())
            self.assertTrue(json.loads((root/'new.response').read_text())['accepted'])
            self.assertFalse(json.loads((root/'expired.response').read_text())['accepted'])

    def test_projection_horizon_is_bounded(self):
        _, points = turn_samples(self.base, 'right')
        frame = np.zeros((540, 960, 3), np.uint8)
        draw_turn(frame, points, np.zeros((3, 3)))
        self.assertEqual(frame.sum(), 0)
        draw_turn(frame, points)
        self.assertGreater(frame.sum(), 0)

    def test_homography_sign_does_not_change_overlay(self):
        points = road_samples(self.base)
        positive = np.zeros((540, 960, 3), np.uint8)
        negative = positive.copy()
        draw_turn(positive, points, self.base.projection)
        draw_turn(negative, points, -self.base.projection)
        np.testing.assert_array_equal(positive, negative)
        self.assertGreater(positive.sum(), 0)


if __name__ == '__main__':
    unittest.main()
