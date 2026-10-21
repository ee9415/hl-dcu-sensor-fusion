import json
from pathlib import Path
import unittest

import numpy as np

from lane_boundaries import (BoundaryConfig, BoundaryTracker, draw_boundaries,
                             projection_for, render_boundary_bev, transform, white_lane_evidence)


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

    def test_width_settings_are_clearance_only(self):
        self.tracker = BoundaryTracker(BoundaryConfig(2., .2, .5))
        result = self.update(self.masks(-2.5, 3.5))
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.left, -2.5, atol=.03)
        np.testing.assert_allclose(result.right, 3.5, atol=.03)
        self.assertTrue(result.telemetry(self.tracker.config)['clearance_sufficient'])
        for kw in ({'vehicle_width_m': 0}, {'left_margin_m': -1},
                   {'right_margin_m': float('nan')}, {'max_forward_m': 41}):
            with self.assertRaises(ValueError):
                BoundaryConfig(**kw)
        self.assertFalse(BoundaryTracker().update_bev(self.forward,self.lateral,
                         *self.masks(-5,5,False),0).valid)

    def test_one_and_two_sided_widening_does_not_expand(self):
        first = self.update(self.masks(-2.5,3.5))
        for index, masks in enumerate((self.masks(-2.5, 5, False), self.masks(-5, 5, False))):
            result = self.update(masks, .1*(index+1))
            self.assertTrue(result.valid)
            np.testing.assert_allclose(result.left, first.left, atol=.02)
            np.testing.assert_allclose(result.right, first.right, atol=.02)

    def test_curves_fit_independently_and_preserve_observed_width_when_one_missing(self):
        bend = .4*(self.forward/25)**2
        first = self.update(self.masks(-2.5+bend, 3.+2*bend))
        self.assertTrue(first.valid)
        np.testing.assert_allclose(first.left, -2.5+bend, atol=.04)
        np.testing.assert_allclose(first.right, 3.+2*bend, atol=.04)
        self.assertGreater(first.right[-1]-first.left[-1], first.right[0]-first.left[0]+.2)
        road,lane = self.masks(-2.4+bend,3.1+2*bend)
        road[:]=True
        lane[:,self.lateral>0]=False
        result=self.update((road,lane),.1)
        self.assertTrue(result.valid)
        self.assertEqual(result.right_state,'inferred')
        self.assertLessEqual(np.max(abs(result.left-first.left)),.100001)
        np.testing.assert_allclose(result.right-result.left, first.right-first.left, atol=1e-6)

    def test_vehicle_width_does_not_move_narrow_observed_lines(self):
        result = self.update(self.masks(-.8,.8))
        self.assertTrue(result.valid)
        self.assertEqual(result.state,'insufficient_width')
        np.testing.assert_allclose(result.right-result.left,1.6,atol=.03)
        self.assertFalse(result.telemetry(self.tracker.config)['clearance_sufficient'])
        self.tracker=BoundaryTracker()
        road,lane=self.masks(-2.5,3.)
        road[40:,abs(self.lateral)>.8]=False
        result=self.update((road,lane))
        self.assertTrue(result.valid)
        self.assertEqual(len(result.forward),40)

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
        self.assertIn(result.state, ('boundary_conflict','lane_unavailable'))

    def test_mask_projection_render_and_json(self):
        h, w = 180, 320
        projection = projection_for((h, w))
        source = np.array([[.04*(w-1), .98*(h-1)], [.62*(w-1), .38*(h-1)]])
        np.testing.assert_allclose(transform(source, np.linalg.inv(projection)), [[0,479],[639,0]], atol=1e-7)
        road = np.full((h, w), 255, np.uint8)
        lane = np.zeros_like(road)
        for side in (-2.5,3.):
            points = transform(np.column_stack(((np.full(60,side)/12+.5)*639,
                                               (1-self.forward/40)*479)),projection)
            from lane_boundaries import draw_polyline
            rgb_mask = np.zeros((h,w,3),np.uint8)
            draw_polyline(rgb_mask,points,(255,255,255),thickness=1)
            lane |= rgb_mask[:,:,0]
        result = self.tracker.update(road, lane, 0.)
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

    def test_lane_supported_thin_road_hole_is_not_a_constriction(self):
        road = np.full((180, 320), 255, np.uint8)
        lane = np.zeros_like(road)
        road[:, 158:162] = 0
        lane[:, 158:162] = 255
        sampled=self.tracker._sample(road,lane)[2]
        self.assertTrue(np.all(sampled))
        self.assertFalse(np.all(self.tracker._sample(road,np.zeros_like(lane))[2]))
        road[:,145:175]=0
        lane[:,145:175]=255
        self.assertFalse(np.all(self.tracker._sample(road,lane)[2]))

    def test_white_paint_refinement_rejects_green_and_matches_stripe(self):
        rgb=np.full((180,320,3),60,np.uint8)
        rgb[:,90:93]=230
        rgb[:,240:243]=(40,180,40)
        lane=np.zeros((180,320),np.uint8)
        lane[:,87:96]=255
        lane[:,237:246]=255
        paint=white_lane_evidence(rgb,lane)
        self.assertTrue(np.all(paint[:,90:93]))
        self.assertFalse(np.any(paint[:,240:243]))
        self.assertFalse(np.any(paint[:,:85]))

    def test_missing_nearest_road_and_paint_does_not_discard_visible_lines(self):
        road,lane=self.masks(-2.5,3.)
        road[:4]=False
        lane[:4]=False
        result=self.update((road,lane))
        self.assertTrue(result.valid)
        self.assertEqual(result.forward[0],self.forward[4])
        np.testing.assert_allclose(result.left,-2.5,atol=.03)
        np.testing.assert_allclose(result.right,3.,atol=.03)

    def test_lane_gap_expires_and_remote_new_stripes_do_not_pull_pair_outward(self):
        first=self.update(self.masks(-2.5,3.))
        result=self.update(self.masks(-5.,5.),.1)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.left,first.left)
        np.testing.assert_allclose(result.right,first.right)
        for n in range(2,23):
            result=self.update(self.masks(-5.,5.),n*.1)
        self.assertFalse(result.valid)

    def test_returning_shifted_pair_reacquires_after_gap(self):
        self.update(self.masks(-2.5,3.))
        for n in range(1,24):
            self.update(self.masks(-5,5,False),n*.1)
        for n in range(24,27):
            result=self.update(self.masks(-3.5,2.),n*.1)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.left,-3.5,atol=.03)
        np.testing.assert_allclose(result.right,2.,atol=.03)


if __name__ == '__main__':
    unittest.main()
