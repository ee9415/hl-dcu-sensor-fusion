import json
from pathlib import Path
import unittest

import numpy as np

from lane_boundaries import (BoundaryConfig, BoundaryTracker, draw_boundaries,
                             projection_for, render_boundary_bev, transform)


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.forward = np.linspace(.5, 25, 60)
        self.lateral = np.linspace(-6, 6, 241)
        self.tracker = BoundaryTracker()

    def masks(self, left=-1.2, right=1.2, markings=True):
        left = np.broadcast_to(left, (60,))
        right = np.broadcast_to(right, (60,))
        x = self.lateral[None, :]
        road = (x >= left[:, None]-.001) & (x <= right[:, None]+.001)
        lane = np.zeros_like(road)
        if markings:
            lane[np.arange(60), np.argmin(abs(x-left[:, None]), axis=1)] = True
            lane[np.arange(60), np.argmin(abs(x-right[:, None]), axis=1)] = True
        return road, lane

    def update(self, masks, t=0):
        return self.tracker.update_bev(self.forward, self.lateral, *masks, t)

    def test_width_settings_asymmetric_and_validation(self):
        self.tracker = BoundaryTracker(BoundaryConfig(2., .2, .5))
        result = self.update(self.masks(-5, 5, False))
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.left, -1.2, atol=1e-5)
        np.testing.assert_allclose(result.right, 1.5, atol=1e-5)
        self.assertEqual(result.state, 'inferred')
        for kw in ({'vehicle_width_m': 0}, {'left_margin_m': -1},
                   {'right_margin_m': float('nan')}, {'max_forward_m': 41},
                   {'vehicle_width_m': True}):
            with self.assertRaises(ValueError):
                BoundaryConfig(**kw)
        loaded = BoundaryConfig.load(Path(__file__).resolve().parents[1]/'lane_boundaries.json')
        self.assertAlmostEqual(loaded.width_m, 2.4)

    def test_one_and_two_sided_widening_does_not_expand(self):
        first = self.update(self.masks())
        for index, masks in enumerate((self.masks(-1.2, 5, False), self.masks(-5, 5, False))):
            result = self.update(masks, .1*(index+1))
            self.assertTrue(result.valid)
            np.testing.assert_allclose(result.right-result.left, 2.4, atol=1e-6)
            np.testing.assert_allclose(result.left, first.left, atol=.02)

    def test_curve_and_rate_with_missing_right_marking(self):
        self.update(self.masks(-4, 4, False))
        bend = .35*(self.forward/25)**2
        road, lane = self.masks(-1.2+bend, 1.2+bend)
        road[:] = True
        lane[:, self.lateral > 0] = False
        previous = self.tracker.previous
        for n in range(1, 15):
            result = self.update((road, lane), n*.1)
            self.assertTrue(result.valid)
            self.assertLessEqual(np.max(abs(result.left-previous.left)), .100001)
            np.testing.assert_allclose(result.right-result.left, 2.4, atol=1e-6)
            previous = result
        self.assertGreater(result.left[-1]-result.left[0], .2)
        self.assertEqual(result.right_state, 'inferred')

    def test_narrow_road_invalid_and_far_constriction_truncates(self):
        self.assertEqual(self.update(self.masks(-.8, .8)).state, 'insufficient_width')
        road, lane = self.masks()
        road[40:, abs(self.lateral) > .8] = False
        result = self.update((road, lane), .1)
        self.assertTrue(result.valid)
        self.assertEqual(len(result.forward), 40)

    def test_loss_expires_and_does_not_resurrect_stale_pair(self):
        self.update(self.masks())
        empty = np.zeros((60, 241), bool)
        held = self.update((empty, empty), .1)
        self.assertTrue(held.valid)
        self.assertEqual(held.state, 'held')
        self.assertFalse(self.update((empty, empty), .7).valid)
        self.assertFalse(self.update((empty, empty), .8).valid)
        self.assertEqual(self.update(self.masks(), .8).state, 'nonmonotonic_timestamp')

    def test_disconnected_road_not_bridged(self):
        road, lane = self.masks(-4, 4, False)
        road[:, abs(self.lateral) < .4] = False
        self.assertFalse(self.update((road, lane)).valid)

    def test_visible_shifted_road_does_not_hold_old_pair(self):
        self.update(self.masks())
        result = self.update(self.masks(2., 5., False), .1)
        self.assertFalse(result.valid)
        self.assertEqual(result.state, 'boundary_conflict')

    def test_mask_projection_render_and_json(self):
        h, w = 180, 320
        projection = projection_for((h, w))
        source = np.array([[.04*(w-1), .98*(h-1)], [.62*(w-1), .38*(h-1)]])
        np.testing.assert_allclose(transform(source, np.linalg.inv(projection)), [[0,479],[639,0]], atol=1e-7)
        road = np.full((h, w), 255, np.uint8)
        result = self.tracker.update(road, np.zeros_like(road), 0.)
        self.assertTrue(result.valid)
        bev = render_boundary_bev(result)
        # Exactly two boundary colors, no center curve at the vehicle axis.
        self.assertTrue(np.any(np.all(bev == (0,220,255), axis=2)))
        self.assertTrue(np.any(np.all(bev == (255,180,0), axis=2)))
        self.assertFalse(np.any(np.all(bev[:,320] == (0,220,255), axis=1)))
        rgb = np.zeros((540,960,3), np.uint8)
        draw_boundaries(rgb, result, (320,180))
        self.assertGreater(np.count_nonzero(rgb), 0)
        payload = result.telemetry(self.tracker.config)
        self.assertEqual(len(payload['left_boundary_dev']), len(payload['right_boundary_dev']))
        json.dumps(payload, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
