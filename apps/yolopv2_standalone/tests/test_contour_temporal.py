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
    """윤곽 추출 결과(group_curves)를 흉내 낸 합성 장면을 만든다.

    shift: 좌우 경계선 x 이동, left/right: 각 쪽 표시 여부, curve: 곡률 크기,
    wrong_right: 오른쪽만 추가로 어긋나게, reverse: 후보 순서 뒤집기.
    """
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
        """기본 응답 프로파일은 느린 프로파일보다 유지(retained) 경계선이 목표 위치에 더 빨리 수렴한다(10/20fps 모두)."""
        slow = replace(BoundaryConfig.load(), skeleton_observed_tau_s=.12,
                       skeleton_observed_fine_tau_s=.25, skeleton_observed_max_rate_px_s=80.,
                       skeleton_retained_tau_s=.4, skeleton_retained_max_rate_px_s=20.)
        def response(config, fps):
            """왼쪽을 잃은 뒤 오른쪽이 12px 이동했을 때, 유지된 왼쪽이 따라와 수렴하기까지 걸린 시간(초)."""
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
        """반대쪽이 aligning(재정렬 중) 상태라도 실제 지지점이 있으면 사라진 쪽을 이끌고, 지지점이 없으면 움직이지 않는다."""
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
        """텔레메트리에 설정된 응답 한계값(최대 속도, 시정수)이 그대로 보고된다."""
        result = self.warm()
        profile = result.telemetry(self.tracker.config)['boundary_response']
        self.assertEqual(profile['retained_max_rate_px_s'],40.)
        self.assertEqual(profile['observed_max_rate_px_s'],120.)
        self.assertEqual(profile['retained_tau_s'],.2)

    def test_first_turn_at_loss_is_not_omitted_from_retained_target(self):
        """놓친 직후의 첫 이동도 유지 목표에 반영되어 최종적으로 12px 이동을 따라간다."""
        before = self.warm()
        for i in range(1,61):
            result = self.update(.95+i*.05,left=False,shift=12.)
        good = np.isfinite(before.image_left[:,0])
        error = np.max(abs(result.image_left[good,0]-before.image_left[good,0]-12.))
        self.assertLess(error,1.2)

    def setUp(self):
        """테스트마다 새 추적기를 만든다."""
        self.tracker = ImageBoundaryTracker()

    def update(self, time, **kwargs):
        """scene(**kwargs) 장면으로 추적기를 한 프레임 갱신한다."""
        return self.tracker.update_contours(scene(**kwargs), (180,320), time)

    def warm(self, fps=20):
        """1초 동안 양쪽 경계선을 정상 관측시켜 추적기를 안정화한다."""
        for i in range(fps):
            result = self.update(i/fps)
        return result

    def displacement(self, before, after, side='left'):
        """두 결과 사이 한쪽 경계선의 최대 x 이동량(px). 공통 유효 행이 충분해야 한다."""
        a, b = getattr(before,'image_'+side), getattr(after,'image_'+side)
        common = np.isfinite(a[:,0]) & np.isfinite(b[:,0])
        self.assertGreater(common.sum(), 8)
        return float(np.max(abs(a[common,0]-b[common,0])))

    def test_loss_keeps_previous_left_instead_of_image_width_prior(self):
        """왼쪽을 놓치면 이미지 폭 기본값으로 대체하지 않고 직전 위치를 유지하며, inferred·가상 상태로 표시한다."""
        before = self.warm()
        after = self.update(1.,left=False)
        self.assertLess(self.displacement(before,after), 1e-8)
        self.assertEqual(after.left_state, 'inferred')
        self.assertLess(after.left_confidence,before.left_confidence)
        self.assertFalse(after.left_support.any())
        self.assertTrue(tracked_quadratics(after,(180,320)).left.is_virtual)

    def test_missing_left_accumulates_bounded_opposite_motion_and_no_width_learning(self):
        """왼쪽이 없을 때 오른쪽 이동을 제한된 속도로 따라가고, 이 기간에는 차로 폭을 학습하지 않는다."""
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
        """한 프레임짜리 이상치는 반대쪽을 끌어당기지 못한다."""
        before = self.warm()
        after = self.update(1.,left=False,wrong_right=65.)
        self.assertLess(self.displacement(before,after),1e-8)
        self.assertFalse(after.right_support.any())
        recovered = self.update(1.05)
        self.assertLess(self.displacement(before,recovered),1e-8)

    def test_reacquisition_is_rate_limited_even_after_confirmation(self):
        """재획득이 확인된 뒤에도 위치 이동은 관측 최대 속도로 제한된다."""
        previous = self.warm()
        for i in range(1,16):
            result = self.update(.95+i*.05,shift=35.)
            self.assertLessEqual(self.displacement(previous,result),self.tracker.config.skeleton_observed_max_rate_px_s*.05+1e-6)
            previous = result
        self.assertEqual(result.left_state,'observed')

    def test_long_loss_retains_skeleton_without_virtual_self_confirmation(self):
        """오래 놓쳐도 skeleton 형태를 유지하되, 가상값으로 스스로 신뢰도를 올리거나 폭을 학습하지 않는다. 양쪽 다 사라져도 쌍은 유지된다."""
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
        """폭 정보 만료(3초) 이후에도 반대쪽 곡률 변화를 제한된 속도로 누적 반영한다."""
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

    def test_missing_side_matches_opposite_shape_with_learned_perspective_spacing(self):
        """놓친 쪽은 반대쪽 현재 곡률과 학습한 원근 간격으로 수렴한다."""
        curves = scene().group_curves
        curves[0][:,0] += 2.*((curves[0][:,1]-125.)/50.)**2
        curves[1][:,0] -= 3.*((curves[1][:,1]-125.)/50.)**2
        for i in range(8):
            before = self.tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),i*.05)
        for i in range(1,61):
            after = self.tracker.update_contours(SimpleNamespace(group_curves=[curves[1]]),(180,320),.35+i*.05)
        common = np.isfinite(after.image_left[:,0]) & np.isfinite(after.image_right[:,0])
        width = after.image_right[common,0]-after.image_left[common,0]
        residual = width-np.polyval(np.polyfit(after.image_left[common,1],width,1),after.image_left[common,1])
        self.assertLess(np.max(abs(residual)),.1)
        self.assertGreater(self.displacement(before,after),.2)
        self.assertTrue(after.detection_diagnostics[0]['opposite_shape_spacing'])

    def test_straight_visible_side_straightens_missing_side_symmetrically(self):
        """좌우 어느 쪽이 사라져도 직선인 반대편과 원근 간격을 따르며 폭은 재학습하지 않는다."""
        for missing in (0, 1):
            tracker = ImageBoundaryTracker()
            curves = scene().group_curves
            curves[missing][:,0] += (3. if missing == 0 else -3.)*((curves[missing][:,1]-125.)/50.)**2
            for i in range(20):
                before = tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),i*.05)
            width = tracker.width.copy()
            counts = tracker.width_samples.copy()
            for i in range(1,81):
                after = tracker.update_contours(SimpleNamespace(group_curves=[curves[1-missing]]),(180,320),.95+i*.05)
                a,b = getattr(before,'image_'+('left' if missing == 0 else 'right')), getattr(after,'image_'+('left' if missing == 0 else 'right'))
                good = np.isfinite(a[:,0]) & np.isfinite(b[:,0])
                self.assertLessEqual(np.max(abs(a[good,0]-b[good,0])),40*.05+1e-6)
                before = after
            points = getattr(after,'image_'+('left' if missing == 0 else 'right'))
            good = np.isfinite(points[:,0])
            residual = points[good,0]-np.polyval(np.polyfit(points[good,1],points[good,0],1),points[good,1])
            self.assertLess(np.max(abs(residual)),.1)
            self.assertFalse(getattr(after,('left' if missing == 0 else 'right')+'_support').any())
            np.testing.assert_array_equal(width,tracker.width)
            np.testing.assert_array_equal(counts,tracker.width_samples)

    def test_weak_partial_observation_uses_stronger_shape_without_self_confirmation(self):
        """약한 일부 관측이 이전 휨을 되살리지 않고, 강한 반대편 형상을 따르며 스스로 관측 확정하지 않는다."""
        for side in (0,1):
            tracker = ImageBoundaryTracker()
            curves = scene().group_curves
            curves[side][:,0] += (4. if side == 0 else -4.)*((curves[side][:,1]-125.)/50.)**2
            for i in range(20):
                before = tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),i*.05)
            partial = [c.copy() for c in curves]
            partial[side] = partial[side][partial[side][:,1]>=145]
            for i in range(1,81):
                after = tracker.update_contours(SimpleNamespace(group_curves=partial),(180,320),.95+i*.05)
            name = 'left' if side == 0 else 'right'
            supported = getattr(after,name+'_support')
            current = getattr(after,'image_'+name)
            old = getattr(before,'image_'+name)
            self.assertFalse(supported.any())
            self.assertAlmostEqual(tracker._contour_seen[side],.95)
            far = np.isfinite(current[:,0])
            other = getattr(after,'image_'+('right' if side == 0 else 'left'))
            gap = abs(current[far,0]-other[far,0])
            residual = gap-np.polyval(np.polyfit(current[far,1],gap,1),current[far,1])
            self.assertLess(np.max(abs(residual)),.1)
            self.assertTrue(after.detection_diagnostics[side]['partial_spacing_guidance'])
            self.assertEqual(getattr(after,name+'_state'),'inferred')

    def test_tracking_display_and_telemetry_share_one_quadratic_model(self):
        """사용자 요구에 따라 추적·표시·텔레메트리가 같은 2차 모델을 사용한다."""
        curves = scene().group_curves
        for curve in curves:
            curve[:,0] += 1.4*np.sin(curve[:,1]/7.)
        result = self.tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),0.)
        points = result.image_left
        good = np.isfinite(points[:,0])
        expected = np.interp(points[good,1],curves[0][:,1],curves[0][:,0])
        fit = np.polyfit(points[good,1],points[good,0],2)
        np.testing.assert_allclose(points[good,0],np.polyval(fit,points[good,1]),atol=1e-8)
        self.assertGreater(np.max(abs(points[good,0]-expected)),.5)
        self.assertEqual(result.telemetry()['image_curve_degree'],2)
        self.assertEqual(result.telemetry()['image_geometry'],'tracked_quadratic_model')
        after = self.update(.1,left=False,right=False)
        self.assertLess(self.displacement(result,after),1e-8)

    def test_renderer_draws_retained_skeleton_samples_and_keeps_unseen_top_black(self):
        """관측 skeleton은 분홍 실선, 유지 skeleton은 분홍 점선으로 그리고, 보이지 않는 영역은 칠하지 않는다."""
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
        self.assertGreater(np.all(image[:,:160] == (255,0,255),axis=2).sum(),10)
        valid = np.ones((180,320),bool);valid[160:]=False
        overlay_tracked_skeleton(image,retained,(180,320),np.eye(3),valid)
        self.assertFalse(image[~valid].any())

    def test_reacquisition_after_long_retention_still_has_no_jump(self):
        """오래 유지한 뒤 재획득해도 위치가 갑자기 튀지 않는다."""
        previous = self.warm()
        for i in range(1,81):
            previous = self.update(.95+i*.05,left=False)
        for i in range(1,21):
            result = self.update(4.95+i*.05,shift=35.)
            self.assertLessEqual(self.displacement(previous,result),self.tracker.config.skeleton_observed_max_rate_px_s*.05+1e-6)
            previous = result
        self.assertEqual(result.left_state,'observed')

    def test_without_reliable_bilateral_spacing_missing_side_is_held(self):
        """양쪽 동시 관측으로 간격을 학습하지 않았다면 임의 간격으로 변형하지 않는다."""
        self.update(0.,right=False)
        initial = self.update(.05,left=False)
        self.assertIsNone(self.tracker.width)
        for i in range(1,61):
            result = self.update(.05+i*.05,left=False,curve=i*.1)
        self.assertTrue(result.left_valid)
        self.assertLess(self.displacement(initial,result),1e-8)
        self.assertIsNone(self.tracker.width)

    def test_no_unobserved_side_is_initialized_from_default_width(self):
        """한 번도 관측되지 않은 쪽은 기본 폭으로 만들어 내지 않는다."""
        result = self.update(0.,left=False)
        self.assertFalse(result.left_valid)
        self.assertTrue(result.right_valid)
        self.assertIsNone(self.tracker.width)

    def test_wide_valid_initial_pair_is_not_cheaper_to_drop(self):
        """넓지만 유효한 초기 쌍을 버리지 않는다."""
        rows = np.arange(110.,176.)
        curves = [np.column_stack((x(rows),rows,np.ones(len(rows)))) for x in
                  (lambda y: 16.-1.3*(y-153), lambda y: 264.+1.1*(y-153))]
        result = self.tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),0.)
        self.assertTrue(result.pair_valid)

    def test_existing_boundary_can_cross_image_center_without_relabeling(self):
        # Boundary ownership follows temporal association, not image half.
        """기존 경계선이 화면 중앙을 넘어가도 좌/우 라벨이 바뀌지 않는다."""
        self.update(0.,shift=70.)
        for i in range(1,61):
            result = self.update(i*.05,shift=70.+i*.6)
        self.assertTrue(result.left_valid)
        self.assertGreater(np.interp(153,result.image_left[::-1,1],result.image_left[::-1,0]),160.)

    def test_candidate_order_and_nearer_distractor_do_not_change_owned_boundary(self):
        """후보 순서나 더 가까운 방해 후보가 있어도 이미 추적 중인 경계선은 바뀌지 않는다."""
        before = self.warm()
        candidates = scene(reverse=True)
        distractor = scene(shift=30.,right=False).group_curves[0]
        candidates.group_curves.insert(0,distractor)
        after = self.tracker.update_contours(candidates,(180,320),1.)
        self.assertLess(self.displacement(before,after),1e-8)

    def test_two_right_candidates_cannot_impersonate_missing_left(self):
        """오른쪽 후보가 둘 있어도 그중 하나가 사라진 왼쪽으로 둔갑하지 않는다."""
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
        """화면 표시용 2차식과 텔레메트리 점들이 같은 다항식을 쓴다."""
        estimate = self.warm()
        estimate = self.update(1.,left=False,shift=3.)
        display = tracked_quadratics(estimate,(180,320))
        payload = estimate.telemetry()
        for side in ('left','right'):
            lane = getattr(display,side)
            points = np.array(payload[side+'_boundary_image'])
            self.assertLess(np.max(abs(np.polyval(lane.coefficients,points[:,1])-points[:,0])),1e-7)

    def test_live_telemetry_is_json_serializable_after_rgb_reset_and_association(self):
        """RGB 리셋·재연결 후에도 텔레메트리가 엄격한 JSON으로 직렬화된다(NumPy bool 등 없음)."""
        for i in range(20):
            estimate = self.tracker.update_contours(scene(left=i<10),(180,320),i*.05,
                rgb=np.full((180,320,3),60,np.uint8))
            # The real server uses this strict conversion; a mock publisher
            # alone does not expose NumPy bool diagnostics on frame two.
            payload = json.loads(json.dumps(estimate.telemetry(),allow_nan=False))
            self.assertIs(type(payload['boundary_detection_diagnostics'][0]['reset']),bool)

    def test_time_and_shape_and_scene_reset_do_not_leak_history(self):
        """시간 역행/큰 시간 간격, 영상 크기 변경, 장면 밝기 급변 시 이전 이력을 버린다."""
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
        """같은 물리 시간 동안의 결과가 10/20/30fps에 관계없이 거의 같다(0.5px 이내)."""
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
