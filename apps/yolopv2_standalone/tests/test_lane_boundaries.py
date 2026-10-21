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

    def test_segmentation_change_does_not_erase_historical_pair(self):
        first=self.update(self.masks())
        result = self.update(self.masks(2., 5., False), .1)
        self.assertTrue(result.pair_valid)
        np.testing.assert_allclose(result.left,first.left)
        np.testing.assert_allclose(result.right,first.right)

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

    def test_rgb_detects_paint_without_nn_and_rejects_broad_bright_edge(self):
        rgb=np.full((180,320,3),60,np.uint8)
        rgb[:,90:93]=220
        rgb[:,180:240]=230
        lane=np.zeros((180,320),np.uint8)
        paint=white_lane_evidence(rgb,lane)
        self.assertTrue(np.all(paint[:,90:93]))
        self.assertFalse(np.any(paint[:,180:240]))

    def test_one_side_reacquires_far_from_history_without_other_side(self):
        self.update(self.masks(-2.5,3.))
        road,lane=self.masks(-1.,4.5)
        road[:]=True
        lane[:,self.lateral>0]=False
        for n in range(1,31):
            result=self.update((road,lane),n*.1)
        self.assertEqual(result.left_state,'observed')
        np.testing.assert_allclose(result.left,-1.,atol=.03)
        self.assertEqual(result.right_state,'inferred')

    def test_long_white_stripe_beats_short_inner_marking(self):
        road,lane=self.masks(-2.5,3.)
        road[:]=True
        lane[:12,np.argmin(abs(self.lateral+.8))]=True
        lane[:12,np.argmin(abs(self.lateral-.8))]=True
        result=self.update((road,lane))
        self.assertTrue(result.pair_valid)
        np.testing.assert_allclose(result.left,-2.5,atol=.03)
        np.testing.assert_allclose(result.right,3.,atol=.03)

    def test_short_wrong_pair_does_not_lock_out_long_actual_white_lines(self):
        road,lane=self.masks(-.8,.8)
        road[:]=True
        lane[16:]=False
        self.update((road,lane))
        for n in range(1,41):
            result=self.update(self.masks(-2.5,3.),n*.1)
        self.assertTrue(result.pair_valid)
        np.testing.assert_allclose(result.left,-2.5,atol=.03)
        np.testing.assert_allclose(result.right,3.,atol=.03)

    def test_rgb_only_boundary_pipeline_with_empty_nn_lane_mask(self):
        from lane_boundaries import draw_polyline
        rgb=np.full((180,320,3),60,np.uint8)
        projection=projection_for((180,320))
        for side in (-2.5,3.):
            points=transform(np.column_stack(((np.full(60,side)/12+.5)*639,
                                             (1-self.forward/40)*479)),projection)
            draw_polyline(rgb,points,(230,230,230),thickness=1)
        result=self.tracker.update(np.full((180,320),255,np.uint8),
                                   np.zeros((180,320),np.uint8),0.,rgb=rgb)
        self.assertTrue(result.pair_valid)
        np.testing.assert_allclose(result.left,-2.5,atol=.3)
        np.testing.assert_allclose(result.right,3.,atol=.3)

    def test_missing_nearest_road_and_paint_does_not_discard_visible_lines(self):
        road,lane=self.masks(-2.5,3.)
        road[:4]=False
        lane[:4]=False
        result=self.update((road,lane))
        self.assertTrue(result.valid)
        self.assertEqual(result.forward[0],self.forward[4])
        np.testing.assert_allclose(result.left,-2.5,atol=.03)
        np.testing.assert_allclose(result.right,3.,atol=.03)

    def test_long_lane_gap_and_remote_new_stripes_do_not_pull_pair_outward(self):
        first=self.update(self.masks(-2.5,3.))
        result=self.update(self.masks(-5.,5.),.1)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.left,first.left)
        np.testing.assert_allclose(result.right,first.right)
        for n in range(2,602):
            result=self.update(self.masks(-5.,5.),n*.1)
            self.assertTrue(result.pair_valid)
            np.testing.assert_allclose(result.left,first.left)
            np.testing.assert_allclose(result.right,first.right)

    def test_returning_shifted_pair_reacquires_after_gap(self):
        self.update(self.masks(-2.5,3.))
        for n in range(1,24):
            self.update(self.masks(-5,5,False),n*.1)
        for n in range(24,27):
            result=self.update(self.masks(-3.5,2.),n*.1)
        self.assertTrue(result.valid)
        self.assertLessEqual(np.max(abs(result.left+2.5)),.100001)
        for n in range(27,40):
            result=self.update(self.masks(-3.5,2.),n*.1)
        np.testing.assert_allclose(result.left,-3.5,atol=.03)
        np.testing.assert_allclose(result.right,2.,atol=.03)

    def test_single_side_startup_renders_without_inventing_other_side(self):
        for visible in (0,1):
            self.tracker=BoundaryTracker()
            road,lane=self.masks(-2.5,3.)
            lane[:,self.lateral>0 if visible==0 else self.lateral<0]=False
            result=self.update((road,lane))
            self.assertTrue(result.valid)
            self.assertEqual(result.state,'partial')
            self.assertEqual(result.left_valid,visible==0)
            self.assertEqual(result.right_valid,visible==1)
            self.assertFalse(result.pair_valid)
            payload=result.telemetry(self.tracker.config)
            missing='right_boundary_dev' if visible==0 else 'left_boundary_dev'
            self.assertTrue(all(v is None for v in payload[missing]))
            self.assertIsNone(payload['boundary_width_min_dev'])
            self.assertIsNone(payload['clearance_sufficient'])
            json.dumps(payload,allow_nan=False)
            bev=render_boundary_bev(result)
            visible_color=(0,220,255) if visible==0 else (255,180,0)
            missing_color=(255,180,0) if visible==0 else (0,220,255)
            self.assertTrue(np.any(np.all(bev==visible_color,axis=2)))
            self.assertFalse(np.any(np.all(bev==missing_color,axis=2)))

    def test_occluded_side_continues_long_term_with_visible_side(self):
        for visible in (0,1):
            self.tracker=BoundaryTracker()
            first=self.update(self.masks(-2.5,3.))
            for n in range(1,302):
                shift=.15*np.sin(n*.1)
                road,lane=self.masks(-2.5+shift,3.+shift)
                road[:]=True
                lane[:,self.lateral>0 if visible==0 else self.lateral<0]=False
                result=self.update((road,lane),n*.1)
                self.assertTrue(result.valid)
                self.assertTrue(result.left_valid if visible==0 else result.right_valid)
                self.assertEqual(result.left_state if visible==0 else result.right_state,'observed')
                self.assertEqual(result.right_state if visible==0 else result.left_state,'inferred')
                np.testing.assert_allclose(result.right-result.left,first.right-first.left,atol=1e-6)

    def test_left_and_right_have_independent_visible_ranges(self):
        road,lane=self.masks(-2.5,3.)
        lane[:25,self.lateral>0]=False
        lane[40:,self.lateral<0]=False
        result=self.update((road,lane))
        self.assertTrue(result.left_valid)
        self.assertTrue(result.right_valid)
        self.assertTrue(np.all(np.isnan(result.right[:25])))
        self.assertTrue(np.all(np.isnan(result.left[40:])))
        self.assertTrue(result.pair_valid)

    def test_conflicting_road_at_hidden_side_does_not_hide_visible_edge(self):
        self.update(self.masks(-2.5,3.))
        road,lane=self.masks(-2.5,3.)
        road[:,self.lateral>1.5]=False
        lane[:,self.lateral>0]=False
        result=self.update((road,lane),.1)
        self.assertTrue(result.valid)
        self.assertTrue(result.left_valid)
        self.assertTrue(result.right_valid)
        self.assertEqual(result.right_state,'inferred')

    def test_long_unmarked_widening_and_segmentation_loss_preserve_curvature(self):
        bend=.4*(self.forward/25)**2
        first=self.update(self.masks(-2.5+bend,3.+bend))
        for n in range(1,602):
            road,lane=self.masks(-5,5,False)
            if n>300:
                road[:]=False
            result=self.update((road,lane),n*.1)
            self.assertTrue(result.pair_valid)
            np.testing.assert_allclose(result.left,first.left)
            np.testing.assert_allclose(result.right,first.right)
        self.assertEqual(result.state,'held')

    def test_short_visible_range_does_not_warp_long_occluded_boundary(self):
        first=self.update(self.masks(-2.5,3.))
        bend=.1*(self.forward/8.)**2
        road,lane=self.masks(-2.5,3.+bend)
        road[:]=True
        lane[:,self.lateral<0]=False
        lane[22:]=False
        result=self.update((road,lane),.1)
        self.assertTrue(result.left_valid)
        self.assertEqual(result.left_state,'inferred')
        self.assertLessEqual(np.nanmax(abs(result.left-first.left)),.100001)
        self.assertLessEqual(abs(result.left[-1]-result.left[22]),.000001)

    def test_return_outside_old_pavement_is_rate_limited_until_aligned(self):
        previous=self.update(self.masks(-2.5,3.))
        for n in range(1,61):
            previous=self.update(self.masks(-5,5,False),n*.1)
        # Old left edge is more than .45 outside the new pavement. Previously
        # this bypassed smoothing and jumped directly to the returned paint.
        for n in range(61,91):
            result=self.update(self.masks(-1.,4.5),n*.1)
            self.assertTrue(result.pair_valid)
            self.assertLessEqual(np.max(abs(result.left-previous.left)),.100001)
            self.assertLessEqual(np.max(abs(result.right-previous.right)),.100001)
            previous=result
        np.testing.assert_allclose(result.left,-1.,atol=.03)
        np.testing.assert_allclose(result.right,4.5,atol=.03)


if __name__ == '__main__':
    unittest.main()
