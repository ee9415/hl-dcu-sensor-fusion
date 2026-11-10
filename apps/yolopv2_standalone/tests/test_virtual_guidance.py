"""virtual_guidance(도로/경계선 기반 가상 주행 경로 2차식) 테스트."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from virtual_guidance import (GuidanceConfig, VirtualGuidance, draw_guidance,
                              road_corridor, _peak)


def boundaries(left=90., right=230., confidence=(.9, .9), states=('observed', 'observed')):
    """고정 x 위치의 세로 좌우 경계선 추정 결과를 흉내 낸다(신뢰도/상태 지정 가능)."""
    y = np.linspace(172, 78, 48)
    return SimpleNamespace(image_left=np.column_stack((np.full(48, left), y)),
                           image_right=np.column_stack((np.full(48, right), y)),
                           left_state=states[0], right_state=states[1],
                           left_confidence=confidence[0], right_confidence=confidence[1],
                           left_support=np.ones(48, bool), right_support=np.ones(48, bool),
                           restored_pair=False)


class GuidanceTests(unittest.TestCase):
    def setUp(self):
        """x=60~260 구간이 도로인 마스크와 가이던스를 켠 추적기를 준비한다."""
        self.road = np.zeros((180, 320), np.uint8)
        self.road[:, 60:261] = 255
        self.tracker = VirtualGuidance(GuidanceConfig(enabled=True))

    def stable(self, b, start=0, count=3):
        """같은 입력을 count프레임 연속으로 넣어 확인 단계를 통과시키고 결과를 반환한다."""
        for i in range(count):
            result = self.tracker.update(self.road, b, (start+i)*.1)
        self.assertTrue(result.valid, result.reason)
        return result

    def assert_geometry(self, r):
        """경로 기하 조건을 확인한다: 계수와 점 일치, 시작점은 차량 위치(하단 중앙), 끝점은 목표점이자 좌우 중앙, S자 없는 순수 2차식."""
        self.assertTrue(r.valid, r.reason)
        expected = np.polynomial.polynomial.polyval(r.path[:, 1], r.coefficients)
        np.testing.assert_allclose(expected, r.path[:, 0], atol=1e-9)
        np.testing.assert_allclose(r.path[0], [159.5, 179*.995], atol=1e-8)
        np.testing.assert_allclose(r.target, r.path[-1], atol=1e-8)
        self.assertAlmostEqual(r.target[0], .5*(r.left[0, 0]+r.right[0, 0]), places=8)
        # Constant second difference: no hidden S curve or sample clipping.
        second = np.diff(r.path[:, 0], n=2)
        np.testing.assert_allclose(second, second[0], atol=1e-9)

    def test_all_four_boundary_sources_and_low_confidence_replacement(self):
        """좌우 신뢰도에 따라 기준이 both/left/right/road로 정해지고, 신뢰도 낮은 쪽은 대체값을 쓴다."""
        for confidences, source, real in (((.9, .9), 'both', (True, True)),
                                         ((.9, .1), 'left', (True, False)),
                                         ((.1, .9), 'right', (False, True)),
                                         ((.1, .1), 'road', (False, False))):
            with self.subTest(source=source):
                self.tracker.reset()
                r = self.stable(boundaries(confidence=confidences))
                self.assertEqual(r.source, source)
                self.assertEqual(r.real, real)
                self.assert_geometry(r)

    def test_rejected_low_confidence_coordinates_have_no_influence(self):
        """신뢰도가 낮아 버려진 쪽 좌표는 값이 아무리 이상해도 결과에 영향을 주지 않는다."""
        a = self.stable(boundaries(confidence=(.1, .9)))
        other = VirtualGuidance(GuidanceConfig(enabled=True))
        for i in range(3):
            b = other.update(self.road, boundaries(left=-9999, confidence=(.1, .9)), i*.1)
        np.testing.assert_allclose(a.path, b.path)
        np.testing.assert_allclose(a.left, b.left)

    def test_no_lane_target_is_connected_run_midpoint_at_half_height(self):
        """차선이 없으면 화면 절반 높이에서 연결된 도로 구간의 중앙을 목표로 삼는다."""
        r = self.stable(boundaries(confidence=(0., 0.)))
        np.testing.assert_allclose(r.target, [160, 90], atol=1e-8)

    def test_disconnected_island_is_not_averaged_into_target(self):
        """떨어져 있는 도로 섬은 도로 폭 계산에 포함하지 않는다."""
        road = self.road.copy()
        road[70:130, 280:319] = 255
        left, right = road_corridor(road > 0, 159.5)
        self.assertEqual(left[90], 60)
        self.assertEqual(right[90], 260)

    def test_no_road_fails_closed_and_clears_history(self):
        """도로가 없으면 무효 처리하고 이력을 지운다."""
        self.stable(boundaries())
        r = self.tracker.update(np.zeros_like(self.road), boundaries(), .3)
        self.assertFalse(r.valid)
        self.assertIsNone(self.tracker.corridor)
        self.assertIsNone(self.tracker.path_coefficient)

    def test_lower_target_search_when_half_height_is_missing(self):
        """절반 높이에 도로가 없으면 더 아래쪽에서 목표를 찾는다."""
        self.road[:105] = 0
        r = self.stable(boundaries(confidence=(0., 0.)))
        self.assertEqual(r.target[1], 105)

    def test_no_unbounded_search_above_or_below_allowed_height(self):
        """허용 높이 범위 밖까지 목표를 찾지 않는다."""
        self.road[:145] = 0
        r = self.tracker.update(self.road, boundaries(), 0.)
        self.assertFalse(r.valid)

    def test_valid_fallback_height_is_retained_to_prevent_flicker(self):
        """깜빡임을 막기 위해 한 번 정한 대체 목표 높이를 유지한다."""
        self.road[:110] = 0
        first = self.stable(boundaries(confidence=(0.,0.)))
        self.road[:110,60:261] = 255
        next_frame = self.tracker.update(self.road,boundaries(confidence=(0.,0.)),.3)
        self.assertTrue(next_frame.valid,next_frame.reason)
        self.assertEqual(first.target[1],next_frame.target[1])

    def test_narrow_road_tip_is_not_selected_as_the_endpoint(self):
        """좁게 튀어나온 도로 끝은 목표점으로 고르지 않는다."""
        self.road[:105] = 0
        self.road[90:105,155:166] = 255
        r = self.stable(boundaries(confidence=(0.,0.)))
        self.assertEqual(r.target[1],105)

    def test_previously_valid_road_does_not_hide_current_invalid_geometry(self):
        """이전에 유효했다고 현재의 잘못된 기하를 가리지 않는다."""
        self.stable(boundaries())
        self.road[:, :200] = 0
        r = self.tracker.update(self.road,boundaries(),.3)
        self.assertFalse(r.valid)
        self.assertIsNone(self.tracker.path_coefficient)

    def test_actual_reacquisition_needs_three_consistent_observations(self):
        """실제 경계선을 기준으로 쓰려면 3프레임 연속 일관된 관측이 필요하다."""
        self.assertEqual(self.tracker.update(self.road, boundaries(), 0).source, 'road')
        self.assertEqual(self.tracker.update(self.road, boundaries(), .1).source, 'road')
        self.assertEqual(self.tracker.update(self.road, boundaries(), .2).source, 'both')
        r = self.tracker.update(self.road, boundaries(confidence=(.1, .9)), .3)
        self.assertEqual(r.source, 'right')
        self.assertFalse(r.real[0])

    def test_predicted_or_held_is_never_a_real_reference(self):
        """예측/유지 상태의 경계선은 실제 기준으로 쓰지 않는다."""
        r = self.stable(boundaries(states=('predicted', 'held')))
        self.assertEqual(r.source, 'road')

    def test_step_change_and_source_switch_obey_boundary_and_path_rate(self):
        """경계선이 갑자기 바뀌어도 경계선·경로 변화는 프레임당 4px 이내로 제한된다."""
        previous = self.stable(boundaries())
        for i in range(3, 14):
            r = self.tracker.update(self.road, boundaries(left=120, right=250), i*.1)
            self.assert_geometry(r)
            for key in ('left', 'right', 'path'):
                change = np.max(abs(getattr(r, key)[:, 0]-getattr(previous, key)[:, 0]))
                self.assertLessEqual(change, 4.+1e-8)
            previous = r

    def test_timestamp_cut_resets_confirmation_and_disabled_is_noop(self):
        """시간이 크게 끊기면 확인 횟수를 초기화하고, 비활성 상태에서는 아무것도 하지 않는다."""
        self.stable(boundaries())
        r = self.tracker.update(self.road, boundaries(), 20.)
        self.assertEqual(r.source, 'road')
        self.assertEqual(self.tracker.hits, [1, 1])
        off = VirtualGuidance(GuidanceConfig(enabled=False))
        self.assertEqual(off.update(self.road, boundaries(), 0).reason, 'disabled')

    def test_polynomial_changes_start_only_for_three_vehicle_positions(self):
        """차량 시작 위치(start_x)를 바꾸면 경로 시작점만 달라지고 목표점은 같다."""
        results = []
        for u in (.35, .50, .65):
            tracker = VirtualGuidance(GuidanceConfig(enabled=True, start_x=u))
            r = tracker.update(self.road, boundaries(confidence=(0, 0)), 0)
            self.assertTrue(r.valid, r.reason)
            self.assertAlmostEqual(r.path[0, 0], 319*u)
            results.append(r)
        for r in results[1:]:
            np.testing.assert_allclose(r.target, results[0].target)

    def test_overlay_and_json_are_separate_from_legacy_centerline(self):
        """가이던스 오버레이/JSON은 기존 중심선 필드와 분리되어 있고 엄격한 JSON으로 직렬화된다."""
        r = self.stable(boundaries(confidence=(0., 0.)))
        frame = np.zeros((540, 960, 3), np.uint8)
        draw_guidance(frame, r)
        self.assertTrue(np.any(np.all(frame == (255, 255, 0), axis=2)))
        self.assertTrue(np.any(np.all(frame == (0, 150, 255), axis=2)))
        payload = r.telemetry()
        self.assertNotIn('centerline_valid', payload)
        json.dumps(payload, allow_nan=False)

    def test_configuration_is_validated_and_env_switch_overrides_file(self):
        """잘못된 설정값은 거부하고, 환경 변수 OAK_VIRTUAL_GUIDANCE가 설정 파일보다 우선한다."""
        for kw in ({'path_rate': 0}, {'target_y': .9}, {'confirm_frames': 2.5}, {'enabled': 1}):
            with self.assertRaises(ValueError):
                GuidanceConfig(**kw)
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)/'settings.json'
            p.write_text('{"enabled":true}')
            with patch.dict('os.environ', {'OAK_VIRTUAL_GUIDANCE': '0'}):
                self.assertFalse(GuidanceConfig.load(p).enabled)

    def test_rate_limit_includes_extremum_between_sample_rows(self):
        # In u=(y-90)/90, the extremum is u=.005 (y=90.45),
        # between the sampled endpoints; its amplitude must not be missed.
        """속도 제한 계산이 샘플 행 사이에 있는 극값까지 고려한다."""
        c=np.array([1-.005**2,.01,-1.])
        self.assertAlmostEqual(_peak(c,np.array([90.,91.])),1.,places=12)

    def test_path_failure_preserves_boundary_display_from_current_road(self):
        """경로 계산이 실패해도 현재 도로 기준 경계선 표시는 유지한다."""
        self.stable(boundaries())
        # A deformed learned width must not blank both virtual boundaries.
        self.tracker.width=np.array([1000.,0.,0.])
        b=boundaries(confidence=(.1,.9))
        self.road[:,:]=0
        self.road[:,150:261]=255
        r=self.tracker.update(self.road,b,.3)
        self.assertFalse(r.valid)
        self.assertTrue(r.boundaries_available)
        self.assertIsNone(r.path)
        self.assertEqual(r.real,(False,False))

    def test_road_loss_retains_explicit_unverified_dashed_edges_without_path(self):
        """도로를 잃으면 경로는 없애고, 마지막 경계선을 '미검증 유지' 점선으로만 보여 준다."""
        previous=self.stable(boundaries())
        blank=np.zeros_like(self.road)
        for i in range(3,10):
            r=self.tracker.update(blank,boundaries(),i*.1)
            self.assertFalse(r.valid)
            self.assertTrue(r.boundaries_available)
            self.assertEqual(r.boundary_state,'held_unverified')
            self.assertFalse(r.road_supported)
            self.assertEqual(r.real,(False,False))
            np.testing.assert_allclose(r.left,previous.left)
        frame=np.zeros((540,960,3),np.uint8)
        draw_guidance(frame,r)
        self.assertTrue(np.any(np.all(frame==(0,90,160),axis=2)))
        self.assertFalse(np.any(np.all(frame==(255,255,0),axis=2)))
        data=r.telemetry()
        self.assertTrue(data['guidance_boundaries_available'])
        self.assertFalse(data['guidance_valid'])
        self.assertEqual(data['guidance_path_image'],[])

    def test_source_reset_does_not_carry_unverified_boundaries_to_another_scene(self):
        """소스 리셋 후에는 이전 장면의 미검증 경계선을 가져오지 않는다."""
        self.stable(boundaries())
        self.tracker.reset()
        r=self.tracker.update(np.zeros_like(self.road),boundaries(),0)
        self.assertFalse(r.boundaries_available)
        self.assertFalse(r.valid)

    def test_real_to_road_fallback_does_not_jump_displayed_boundaries(self):
        """실제 경계선 기준에서 도로 기준으로 바뀌어도 표시된 경계선이 튀지 않는다."""
        previous=self.stable(boundaries())
        # Narrowing the road invalidates the retained corridor. A boundary-only
        # fallback remains visible and still follows the configured rate bound.
        road=self.road.copy();road[:,:110]=0
        r=self.tracker.update(road,boundaries(),.3)
        self.assertTrue(r.boundaries_available)
        np.testing.assert_array_less(abs(r.left[:,0]-previous.left[:,0]),4.+1e-7)
        np.testing.assert_array_less(abs(r.right[:,0]-previous.right[:,0]),4.+1e-7)

    def test_path_display_survives_road_loss_without_becoming_valid(self):
        """도로를 잃어도 표시용 경로는 '추정'으로 남지만 유효 경로로 보고하지는 않는다."""
        previous=self.stable(boundaries())
        for i in range(3,12):
            r=self.tracker.update(np.zeros_like(self.road),boundaries(),i*.1)
            self.assertFalse(r.valid)
            self.assertIsNone(r.path)
            self.assertIsNone(r.coefficients)
            self.assertIsNotNone(r.visible_path)
            np.testing.assert_allclose(r.visible_path,previous.path,atol=1e-8)
        frame=np.zeros((540,960,3),np.uint8)
        draw_guidance(frame,r)
        self.assertTrue(np.any(np.all(frame==(180,180,0),axis=2)))
        data=r.telemetry()
        self.assertEqual(data['guidance_path_display_state'],'estimated')
        self.assertTrue(data['guidance_path_display_available'])
        self.assertEqual(data['guidance_path_image'],[])

    def test_fallback_can_initialize_a_display_only_quadratic_from_road(self):
        """대체 경로는 도로만으로도 표시 전용 2차식을 처음부터 만들 수 있다."""
        left,right=road_corridor(self.road>0,159.5)
        r=self.tracker._boundary_fallback('forced_path_failure',left,right,np.array([159.5,179*.995]),.1)
        self.assertFalse(r.valid)
        p=r.visible_path
        self.assertIsNotNone(p)
        np.testing.assert_allclose(p[0],[159.5,179*.995],atol=1e-8)
        self.assertAlmostEqual(p[-1,0],.5*(r.left[0,0]+r.right[0,0]))
        second=np.diff(p[:,0],n=2)
        np.testing.assert_allclose(second,second[0],atol=1e-9)

    def test_fallback_and_recovery_share_display_path_rate_limit(self):
        """대체 상태와 복구 과정 모두 표시 경로에 같은 속도 제한을 적용한다."""
        previous=self.stable(boundaries())
        for i in range(3,16):
            road=self.road.copy()
            if i<8:road[:,:110]=0
            r=self.tracker.update(road,boundaries(),i*.1)
            self.assertIsNotNone(r.visible_path)
            p=r.visible_path
            np.testing.assert_allclose(p[0],[159.5,179*.995],atol=1e-8)
            self.assertAlmostEqual(p[-1,0],.5*(r.left[0,0]+r.right[0,0]),places=7)
            self.assertLessEqual(np.max(abs(p[:,0]-previous.visible_path[:,0])),4.+1e-8)
            previous=r

    def test_path_display_clears_at_source_reset_and_stays_off_when_disabled(self):
        """소스 리셋 시 표시 경로를 지우고, 비활성 상태에서는 표시하지 않는다."""
        self.stable(boundaries())
        self.tracker.reset()
        r=self.tracker.update(np.zeros_like(self.road),boundaries(),0)
        self.assertIsNone(r.visible_path)
        off=VirtualGuidance(GuidanceConfig(enabled=False))
        self.assertIsNone(off.update(self.road,boundaries(),0).visible_path)

    def test_boundary_visibility_only_hides_edges_keeps_current_and_estimated_path(self):
        """경계선 표시를 끄면 경계선만 숨기고 경로(현재/추정)는 그대로 그린다."""
        valid=self.stable(boundaries())
        held=self.tracker.update(np.zeros_like(self.road),boundaries(),.3)
        for r,color in ((valid,(255,255,0)),(held,(180,180,0))):
            on=np.zeros((540,960,3),np.uint8)
            off=on.copy()
            draw_guidance(on,r)
            draw_guidance(off,r,show_boundaries=False)
            np.testing.assert_array_equal(np.all(on==color,axis=2),np.all(off==color,axis=2))
            self.assertTrue(np.any(off))
            self.assertLess(np.count_nonzero(off),np.count_nonzero(on))


if __name__ == '__main__':
    unittest.main()
