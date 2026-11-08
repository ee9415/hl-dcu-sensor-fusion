"""Sequence contracts for the mask-fed common boundary tracker."""
import unittest
import json
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from image_boundaries import ImageBoundaryTracker
from lane_boundaries import BoundaryConfig
from ego_lane_quadratic import tracked_quadratics


def scene(shift=0., left=True, right=True, curve=0., wrong_right=0., reverse=False):
    rows = np.arange(82., 176.)
    bend = curve*((rows-125.)/50.)**2
    curves = []
    for visible, sign, offset in ((left, -1, 0.), (right, 1, wrong_right)):
        if visible:
            x = 160+sign*.9*(rows-65.)+shift+bend+offset
            curves.append(np.column_stack((x, rows, np.ones(len(rows)))))
    return SimpleNamespace(group_curves=curves[::-1] if reverse else curves)


class ContourTemporalTests(unittest.TestCase):
    def test_faster_profile_reaches_retained_target_sooner_at_two_frame_rates(self):
        slow = replace(BoundaryConfig.load(), skeleton_observed_tau_s=.12,
                       skeleton_observed_fine_tau_s=.25, skeleton_observed_max_rate_px_s=80.,
                       skeleton_retained_tau_s=.4, skeleton_retained_max_rate_px_s=20.)
        def response(config, fps):
            tracker = ImageBoundaryTracker(config)
            for i in range(fps):
                before = tracker.update_contours(scene(),(180,320),i/fps)
            initial = before.image_left.copy()
            good = np.isfinite(initial[:,0])
            for i in range(1,4*fps):
                result = tracker.update_contours(scene(left=False,shift=12.),(180,320),
                                                 (fps-1+i)/fps)
                if np.max(abs(result.image_left[good,0]-initial[good,0]-12.)) <= 1.2:
                    return i/fps
            self.fail('Retained boundary did not converge')
        for fps in (10,20):
            self.assertLess(response(BoundaryConfig.load(),fps),response(slow,fps)-.1)

    def test_confirmed_aligning_observation_already_guides_missing_side(self):
        previous = self.warm()
        guided = False
        for i in range(1,16):
            result = self.update(.95+i*.05,left=False,shift=25.)
            if result.right_state == 'aligning' and result.right_support.any():
                guided |= self.displacement(previous,result) > 1e-6
            if not result.right_support.any():
                self.assertLess(self.displacement(previous,result),1e-8)
            previous = result
        self.assertTrue(guided)

    def test_response_telemetry_reports_configured_limits(self):
        result = self.warm()
        profile = result.telemetry(self.tracker.config)['boundary_response']
        self.assertEqual(profile['retained_max_rate_px_s'],40.)
        self.assertEqual(profile['observed_max_rate_px_s'],120.)
        self.assertEqual(profile['retained_tau_s'],.2)

    def test_first_turn_at_loss_is_not_omitted_from_retained_target(self):
        before = self.warm()
        for i in range(1,61):
            result = self.update(.95+i*.05,left=False,shift=12.)
        good = np.isfinite(before.image_left[:,0])
        error = np.max(abs(result.image_left[good,0]-before.image_left[good,0]-12.))
        self.assertLess(error,1.2)

    def setUp(self):
        self.tracker = ImageBoundaryTracker()

    def update(self, time, **kwargs):
        return self.tracker.update_contours(scene(**kwargs), (180,320), time)

    def warm(self, fps=20):
        for i in range(fps):
            result = self.update(i/fps)
        return result

    def displacement(self, before, after, side='left'):
        a, b = getattr(before,'image_'+side), getattr(after,'image_'+side)
        common = np.isfinite(a[:,0]) & np.isfinite(b[:,0])
        self.assertGreater(common.sum(), 8)
        return float(np.max(abs(a[common,0]-b[common,0])))

    def test_loss_keeps_previous_left_instead_of_image_width_prior(self):
        before = self.warm()
        after = self.update(1.,left=False)
        self.assertLess(self.displacement(before,after), 1e-8)
        self.assertEqual(after.left_state, 'inferred')
        self.assertLess(after.left_confidence,before.left_confidence)
        self.assertFalse(after.left_support.any())
        self.assertTrue(tracked_quadratics(after,(180,320)).left.is_virtual)

    def test_missing_left_accumulates_bounded_opposite_motion_and_no_width_learning(self):
        previous = self.warm()
        width = self.tracker.width.copy()
        count = self.tracker.width_samples.copy()
        initial = previous
        for i in range(1,11):
            result = self.update(.95+i*.05, left=False,shift=i*.8)
            self.assertLessEqual(self.displacement(previous,result),self.tracker.config.skeleton_retained_max_rate_px_s*.05+1e-7)
            previous = result
        self.assertGreater(self.displacement(initial,result),.5)
        np.testing.assert_array_equal(self.tracker.width,width)
        np.testing.assert_array_equal(self.tracker.width_samples,count)

    def test_one_frame_outlier_cannot_pull_opposite(self):
        before = self.warm()
        after = self.update(1.,left=False,wrong_right=65.)
        self.assertLess(self.displacement(before,after),1e-8)
        self.assertFalse(after.right_support.any())
        recovered = self.update(1.05)
        self.assertLess(self.displacement(before,recovered),1e-8)

    def test_reacquisition_is_rate_limited_even_after_confirmation(self):
        previous = self.warm()
        for i in range(1,16):
            result = self.update(.95+i*.05,shift=35.)
            self.assertLessEqual(self.displacement(previous,result),self.tracker.config.skeleton_observed_max_rate_px_s*.05+1e-6)
            previous = result
        self.assertEqual(result.left_state,'observed')

    def test_long_loss_retains_skeleton_without_virtual_self_confirmation(self):
        before = self.warm()
        width = self.tracker.width.copy()
        counts = self.tracker.width_samples.copy()
        for i in range(1,121):
            result = self.update(.95+i*.05,left=False)
        self.assertTrue(result.left_valid)
        self.assertTrue(result.right_valid)
        self.assertLess(self.displacement(before,result),1e-8)
        self.assertFalse(result.left_support.any())
        self.assertLess(result.left_confidence,.001)
        self.assertAlmostEqual(self.tracker._contour_seen[0],.95)
        np.testing.assert_array_equal(width,self.tracker.width)
        np.testing.assert_array_equal(counts,self.tracker.width_samples)
        before = result
        for i in range(1,121):
            result = self.update(6.95+i*.05,left=False,right=False)
        self.assertTrue(result.pair_valid)
        self.assertLess(self.displacement(before,result),1e-8)

    def test_opposite_curvature_is_accumulated_after_three_second_width_timeout(self):
        previous = self.warm()
        initial = previous
        middle = None
        for i in range(1,121):
            result = self.update(.95+i*.05,left=False,curve=i*.08)
            self.assertTrue(result.pair_valid)
            self.assertFalse(result.left_support.any())
            self.assertLessEqual(self.displacement(previous,result),self.tracker.config.skeleton_retained_max_rate_px_s*.05+1e-6)
            if i == 70:
                middle = result
            previous = result
        self.assertGreater(self.displacement(middle,result),.4)
        self.assertGreater(self.displacement(initial,result),2.)
        self.assertEqual(result.left_state,'inferred')
        self.assertGreater(result.detection_diagnostics[0]['missing_s'],5.)

    def test_loss_preserves_distinct_original_shape_instead_of_copying_other_curve(self):
        curves = scene().group_curves
        curves[0][:,0] += 2.*((curves[0][:,1]-125.)/50.)**2
        curves[1][:,0] -= 3.*((curves[1][:,1]-125.)/50.)**2
        for i in range(8):
            before = self.tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),i*.05)
        for i in range(1,61):
            after = self.tracker.update_contours(SimpleNamespace(group_curves=[curves[1]]),(180,320),.35+i*.05)
        self.assertLess(self.displacement(before,after),1e-8)

    def test_skeleton_details_survive_without_polynomial_refitting(self):
        curves = scene().group_curves
        for curve in curves:
            curve[:,0] += 1.4*np.sin(curve[:,1]/7.)
        result = self.tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),0.)
        points = result.image_left
        good = np.isfinite(points[:,0])
        expected = np.interp(points[good,1],curves[0][:,1],curves[0][:,0])
        np.testing.assert_allclose(points[good,0],expected,atol=1e-8)
        fit = np.polyfit(points[good,1],points[good,0],2)
        self.assertGreater(np.max(abs(np.polyval(fit,points[good,1])-expected)),.5)
        self.assertIsNone(result.telemetry()['image_curve_degree'])
        self.assertEqual(result.telemetry()['image_geometry'],'tracked_skeleton_samples')
        after = self.update(.1,left=False,right=False)
        self.assertLess(self.displacement(result,after),1e-8)

    def test_renderer_draws_retained_skeleton_samples_and_keeps_unseen_top_black(self):
        from lane_overlay import overlay_tracked_skeleton
        result = self.warm()
        image = np.zeros((180,320,3),np.uint8)
        overlay_tracked_skeleton(image,result,(180,320))
        points = result.image_left[np.isfinite(result.image_left).all(axis=1)]
        x,y = np.rint(points).astype(int).T
        self.assertTrue(np.all(image[y,x] == (255,0,255)))
        retained = self.update(1.,left=False)
        image[:] = 0
        overlay_tracked_skeleton(image,retained,(180,320))
        self.assertGreater(np.all(image[:,:160] == (0,255,255),axis=2).sum(),10)
        valid = np.ones((180,320),bool);valid[160:]=False
        overlay_tracked_skeleton(image,retained,(180,320),np.eye(3),valid)
        self.assertFalse(image[~valid].any())

    def test_reacquisition_after_long_retention_still_has_no_jump(self):
        previous = self.warm()
        for i in range(1,81):
            previous = self.update(.95+i*.05,left=False)
        for i in range(1,21):
            result = self.update(4.95+i*.05,shift=35.)
            self.assertLessEqual(self.displacement(previous,result),self.tracker.config.skeleton_observed_max_rate_px_s*.05+1e-6)
            previous = result
        self.assertEqual(result.left_state,'observed')

    def test_opposite_changes_do_not_require_bilateral_width_training(self):
        self.update(0.,right=False)
        initial = self.update(.05,left=False)
        self.assertIsNone(self.tracker.width)
        for i in range(1,61):
            result = self.update(.05+i*.05,left=False,curve=i*.1)
        self.assertTrue(result.left_valid)
        self.assertGreater(self.displacement(initial,result),1.)
        self.assertIsNone(self.tracker.width)

    def test_no_unobserved_side_is_initialized_from_default_width(self):
        result = self.update(0.,left=False)
        self.assertFalse(result.left_valid)
        self.assertTrue(result.right_valid)
        self.assertIsNone(self.tracker.width)

    def test_wide_valid_initial_pair_is_not_cheaper_to_drop(self):
        rows = np.arange(110.,176.)
        curves = [np.column_stack((x(rows),rows,np.ones(len(rows)))) for x in
                  (lambda y: 16.-1.3*(y-153), lambda y: 264.+1.1*(y-153))]
        result = self.tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),0.)
        self.assertTrue(result.pair_valid)

    def test_existing_boundary_can_cross_image_center_without_relabeling(self):
        # Boundary ownership follows temporal association, not image half.
        self.update(0.,shift=70.)
        for i in range(1,61):
            result = self.update(i*.05,shift=70.+i*.6)
        self.assertTrue(result.left_valid)
        self.assertGreater(np.interp(153,result.image_left[::-1,1],result.image_left[::-1,0]),160.)

    def test_candidate_order_and_nearer_distractor_do_not_change_owned_boundary(self):
        before = self.warm()
        candidates = scene(reverse=True)
        distractor = scene(shift=30.,right=False).group_curves[0]
        candidates.group_curves.insert(0,distractor)
        after = self.tracker.update_contours(candidates,(180,320),1.)
        self.assertLess(self.displacement(before,after),1e-8)

    def test_two_right_candidates_cannot_impersonate_missing_left(self):
        before = self.warm()
        candidates = scene(left=False)
        outer = scene(left=False,wrong_right=45.).group_curves[0]
        candidates.group_curves.append(outer)
        for i in range(1,10):
            after = self.tracker.update_contours(candidates,(180,320),.95+i*.05)
            self.assertFalse(after.left_support.any())
            self.assertTrue(after.right_support.any())
            self.assertLess(self.displacement(before,after),1e-8)

    def test_display_and_telemetry_share_identical_polynomial(self):
        estimate = self.warm()
        estimate = self.update(1.,left=False,shift=3.)
        display = tracked_quadratics(estimate,(180,320))
        payload = estimate.telemetry()
        for side in ('left','right'):
            lane = getattr(display,side)
            points = np.array(payload[side+'_boundary_image'])
            self.assertLess(np.max(abs(np.polyval(lane.coefficients,points[:,1])-points[:,0])),1e-7)

    def test_live_telemetry_is_json_serializable_after_rgb_reset_and_association(self):
        for i in range(20):
            estimate = self.tracker.update_contours(scene(left=i<10),(180,320),i*.05,
                rgb=np.full((180,320,3),60,np.uint8))
            # The real server uses this strict conversion; a mock publisher
            # alone does not expose NumPy bool diagnostics on frame two.
            payload = json.loads(json.dumps(estimate.telemetry(),allow_nan=False))
            self.assertIs(type(payload['boundary_detection_diagnostics'][0]['reset']),bool)

    def test_time_and_shape_and_scene_reset_do_not_leak_history(self):
        self.warm()
        self.assertFalse(self.update(.2,left=False,right=False).valid)
        self.warm()
        self.assertFalse(self.update(3.,left=False,right=False).valid)
        self.warm()
        result = self.tracker.update_contours(scene(left=False,right=False),(360,640),1.)
        self.assertFalse(result.valid)
        self.tracker.update_contours(scene(),(180,320),1.1,rgb=np.zeros((180,320,3),np.uint8))
        result = self.tracker.update_contours(scene(left=False,right=False),(180,320),1.2,
                                              rgb=np.full((180,320,3),255,np.uint8))
        self.assertFalse(result.valid)

    def test_fps_independence_for_constant_observation_and_loss(self):
        endpoints = []
        for fps in (10,20,30):
            self.tracker = ImageBoundaryTracker()
            self.warm(fps)
            # Identical physical interval, not an identical frame count.
            for i in range(1,fps+1):
                result = self.update((fps-1)/fps+i/fps,shift=6.)
            endpoints.append(result.image_left[:,0])
        self.assertLess(np.nanmax(np.ptp(np.array(endpoints),axis=0)),.5)


if __name__ == '__main__':
    unittest.main()
