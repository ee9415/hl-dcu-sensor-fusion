"""ego_lane_quadratic(자차 차로 좌우 경계선 2차식 선택·보완) 테스트.

좌우 쌍 선택, 짧은 쪽 연장, 가상 반대쪽 생성, 원근 수렴점 추정을 다룬다.
"""
import unittest
from types import SimpleNamespace

import numpy as np

from ego_lane_quadratic import (fit_quadratic, select_ego_quadratics,
                               add_virtual_side,estimate_lane_perspective,complete_partial_lanes)


class EgoQuadraticTests(unittest.TestCase):
    def test_partial_extension_uses_opposite_and_is_tangent_continuous(self):
        """짧게 관측된 왼쪽은 반대쪽을 참고해 연장하며, 연결점에서 위치와 기울기가 이어진다. 연장 부분은 가상으로 표시."""
        left=self.curve(lambda y:140-.8*(y-85))[70:]
        right=self.curve(lambda y:180+.8*(y-85))
        result=SimpleNamespace(group_curves=[left,right])
        before=select_ego_quadratics(result,(180,320))
        after=complete_partial_lanes(before,result,(180,320))
        self.assertIsNotNone(after.left.extension)
        self.assertIs(after.right,before.right)
        np.testing.assert_array_equal(after.left.points,before.left.points)
        y=after.left.points[0,1]
        a,b=after.left.coefficients,after.left.extension.coefficients
        self.assertAlmostEqual(np.polyval(a,y),np.polyval(b,y))
        self.assertAlmostEqual(2*a[0]*y+a[1],2*b[0]*y+b[1])
        self.assertTrue(after.left.extension.is_virtual)

    def test_tiny_near_fragment_anchors_extension_instead_of_width_prior(self):
        """근거리의 아주 짧은 조각이라도 있으면 폭 기본값 대신 그 조각을 기준으로 연장한다."""
        left=self.curve(lambda y:140-.8*(y-85))[81:]
        right=self.curve(lambda y:180+.8*(y-85))
        result=SimpleNamespace(group_curves=[left,right])
        before=add_virtual_side(select_ego_quadratics(result,(180,320)),result,(180,320))
        self.assertTrue(before.left.is_virtual)
        after=complete_partial_lanes(before,result,(180,320))
        self.assertFalse(after.left.is_virtual)
        self.assertEqual(after.left.support_rows,9)
        self.assertIsNotNone(after.left.extension)

    def test_complete_pair_does_not_get_partial_extension(self):
        """양쪽이 모두 충분히 관측된 쌍은 연장하지 않는다."""
        result=SimpleNamespace(group_curves=self.fragments()[:2])
        # Use equal-support pair to avoid an intentionally short right span.
        result.group_curves=[self.curve(lambda y:140-.8*(y-85)),self.curve(lambda y:180+.8*(y-85))]
        selected=select_ego_quadratics(result,(180,320))
        self.assertIs(complete_partial_lanes(selected,result,(180,320)),selected)

    def test_far_fragment_cannot_anchor_near_extension(self):
        """원거리 조각만으로는 근거리 연장의 기준이 될 수 없다."""
        left=self.curve(lambda y:140-.8*(y-85))[:9]
        right=self.curve(lambda y:180+.8*(y-85))
        result=SimpleNamespace(group_curves=[left,right])
        selected=add_virtual_side(select_ego_quadratics(result,(180,320)),result,(180,320))
        after=complete_partial_lanes(selected,result,(180,320))
        self.assertIs(after,selected)

    def test_narrow_parking_pair_is_not_extended(self):
        """주차선처럼 좁은 쌍은 연장하지 않는다."""
        left=self.curve(lambda y:140+.1*(y-85))[70:]
        right=self.curve(lambda y:170+.1*(y-85))
        result=SimpleNamespace(group_curves=[left,right])
        selected=select_ego_quadratics(result,(180,320))
        self.assertIs(complete_partial_lanes(selected,result,(180,320)),selected)
    def test_polynomial_overlay_distinguishes_virtual_side(self):
        """오버레이에서 실제 선과 가상 선(모두 BGR 255,0,255, 실선/점선)을 구분해 그린다."""
        from lane_overlay import overlay_lane_quadratics
        result=SimpleNamespace(group_curves=[self.curve(lambda y:140-.8*(y-85))])
        selected=select_ego_quadratics(result,(180,320))
        virtual=add_virtual_side(selected,result,(180,320))
        first=np.zeros((180,320,3),np.uint8)
        second=first.copy()
        overlay_lane_quadratics(first,selected,(180,320))
        overlay_lane_quadratics(second,virtual,(180,320))
        self.assertTrue(np.any(np.all(first==(255,0,255),axis=2)))
        self.assertTrue(np.all(np.any(second,axis=2)[np.any(first,axis=2)]))
        self.assertGreater(np.count_nonzero(np.any(second,axis=2)),np.count_nonzero(np.any(first,axis=2)))
        self.assertFalse(np.any(np.all(second==(0,255,255),axis=2)))
        self.assertFalse(np.any(np.all(first==(0,255,255),axis=2)))
        # The virtual curve must contain both drawn and empty center samples.
        fit=virtual.right
        xy=np.rint(fit.points).astype(int)
        colored=np.all(second[xy[:,1],xy[:,0]]==(255,0,255),axis=1)
        self.assertTrue(colored.any())
        self.assertFalse(colored.all())

    def test_polynomial_top_overlay_respects_unseen_mask(self):
        """유효 영역 마스크 밖(보이지 않는 곳)에는 그리지 않는다."""
        from lane_overlay import overlay_lane_quadratics
        selected=select_ego_quadratics(SimpleNamespace(group_curves=[self.curve(lambda y:140-.8*(y-85))]),(180,320))
        frame=np.zeros((180,320,3),np.uint8)
        valid=np.ones((180,320),bool)
        valid[150:]=False
        overlay_lane_quadratics(frame,selected,(180,320),np.eye(3),valid)
        self.assertTrue(frame[:150].any())
        self.assertFalse(frame[150:].any())

    def test_fragment_estimates_convergence_and_virtual_meets_it(self):
        """반대쪽 원거리 조각으로 원근 수렴점을 추정하고, 가상 선이 그 점에서 실제 선과 만난다. 신뢰도는 limited."""
        y=np.arange(105,175,dtype=float)
        left=np.column_stack((240-y,y,np.ones(len(y))))
        far_y=np.arange(110,120,dtype=float)
        right=np.column_stack((60+far_y,far_y,np.ones(len(far_y))))
        result=SimpleNamespace(group_curves=[left,right])
        before=select_ego_quadratics(result,(180,320))
        perspective=estimate_lane_perspective(before.left,result.group_curves,(180,320),'right')
        self.assertAlmostEqual(perspective['y'],90)
        self.assertAlmostEqual(perspective['x'],150)
        after=add_virtual_side(before,result,(180,320))
        self.assertAlmostEqual(np.polyval(after.left.coefficients,90),
                               np.polyval(after.right.coefficients,90))
        self.assertTrue(after.right.is_virtual)
        self.assertEqual(after.right.perspective['confidence'],'limited')

    def test_parallel_or_transverse_fragments_do_not_invent_convergence(self):
        """평행하거나 가로 방향인 조각으로는 수렴점을 만들어 내지 않는다."""
        y=np.arange(105,175,dtype=float)
        left=np.column_stack((240-y,y,np.ones(len(y))))
        far_y=np.arange(110,120,dtype=float)
        for x in (300-far_y,np.full(len(far_y),200)):
            result=SimpleNamespace(group_curves=[left,np.column_stack((x,far_y,np.ones(len(far_y))))])
            real=fit_quadratic(left,180,0)
            self.assertIsNone(estimate_lane_perspective(real,result.group_curves,(180,320),'right'))

    @staticmethod
    def curve(x):
        """y=85~174 범위에서 x(y) 함수로 관측 곡선 (x, y, 관측=1)을 만든다."""
        y=np.arange(85,175,dtype=float)
        return np.column_stack((x(y),y,np.ones(len(y))))

    def test_coefficients_in_pixel_coordinates(self):
        """2차식 계수는 픽셀 좌표계 기준 x = a*y^2 + b*y + c 이다."""
        curve=self.curve(lambda y:.002*y*y-.8*y+160)
        fit=fit_quadratic(curve,180)
        np.testing.assert_allclose(fit.coefficients,[.002,-.8,160],atol=1e-8)

    def test_inferred_samples_do_not_influence_fit(self):
        """추정(관측 플래그 0) 샘플은 피팅에 쓰이지 않는다."""
        curve=self.curve(lambda y:.002*y*y-.8*y+160)
        curve[20:30,0]=1000
        curve[20:30,2]=0
        fit=fit_quadratic(curve,180)
        np.testing.assert_allclose(fit.coefficients,[.002,-.8,160],atol=1e-8)

    def test_nearest_pair_excludes_outer_lanes(self):
        """여러 차선 중 자차에 가장 가까운 좌우 쌍만 고른다."""
        curves=[self.curve(lambda y:140-.8*(y-85)),
                self.curve(lambda y:180+.8*(y-85)),
                self.curve(lambda y:100-1.2*(y-85)),
                self.curve(lambda y:220+1.2*(y-85))]
        selected=select_ego_quadratics(SimpleNamespace(group_curves=curves),(180,320))
        self.assertEqual(selected.reason,'pair')
        self.assertEqual(selected.left.group_index,0)
        self.assertEqual(selected.right.group_index,1)

    def test_missing_side_is_not_fabricated(self):
        """한쪽만 있으면 다른 쪽을 만들어 내지 않고 left_only로 보고한다."""
        curves=[self.curve(lambda y:140-.8*(y-85))]
        selected=select_ego_quadratics(SimpleNamespace(group_curves=curves),(180,320))
        self.assertIsNotNone(selected.left)
        self.assertIsNone(selected.right)
        self.assertEqual(selected.reason,'left_only')

    def test_short_branch_is_not_a_lane(self):
        """너무 짧은 조각은 차선으로 피팅하지 않는다."""
        curve=self.curve(lambda y:180+.8*(y-85))[:8]
        self.assertIsNone(fit_quadratic(curve,180))

    def test_empty_scene_has_no_curves(self):
        """빈 장면에서는 좌우 모두 없다."""
        selected=select_ego_quadratics(SimpleNamespace(group_curves=[]),(180,320))
        self.assertIsNone(selected.left)
        self.assertIsNone(selected.right)

    @staticmethod
    def fragments(far_offset=0):
        """왼쪽 긴 선, 오른쪽 근거리 조각, 오른쪽 원거리 조각(far_offset만큼 어긋남)을 만든다."""
        def points(start,stop,side,offset=0):
            y=np.arange(start,stop,dtype=float)
            x=240-y if side=='left' else 60+y+offset
            return np.column_stack((x,y,np.ones(len(y))))
        return [points(90,175,'left'),points(140,175,'right'),
                points(95,112,'right',far_offset)]

    def test_opposite_support_recovers_measured_far_fragment(self):
        """반대쪽(왼쪽)을 근거로 같은 선 위의 원거리 조각을 오른쪽에 합친다."""
        curves=self.fragments()
        selected=select_ego_quadratics(SimpleNamespace(group_curves=curves),(180,320))
        self.assertEqual(selected.right.source_groups,(1,2))
        self.assertEqual(selected.right.points[0,1],95)
        np.testing.assert_allclose(selected.right.coefficients,[0,1,60],atol=1e-8)

    def test_opposite_support_rejects_neighboring_far_fragment(self):
        """옆 차로로 어긋난 원거리 조각은 합치지 않는다."""
        curves=self.fragments(far_offset=25)
        selected=select_ego_quadratics(SimpleNamespace(group_curves=curves),(180,320))
        self.assertEqual(selected.right.source_groups,(1,))

    def test_opposite_support_rejects_horizontal_far_fragment(self):
        """세로 방향이 아닌 원거리 조각은 합치지 않는다."""
        curves=self.fragments()
        curves[2][:,0]=180
        selected=select_ego_quadratics(SimpleNamespace(group_curves=curves),(180,320))
        self.assertEqual(selected.right.source_groups,(1,))

    def test_opposite_support_leaves_ambiguous_far_fragments_separate(self):
        """후보 원거리 조각이 둘 이상이면 모호하므로 합치지 않는다."""
        curves=self.fragments()
        other=curves[2].copy()
        other[:,0]+=1
        curves.append(other)
        selected=select_ego_quadratics(SimpleNamespace(group_curves=curves),(180,320))
        self.assertEqual(selected.right.source_groups,(1,))

    def test_no_opposite_boundary_means_no_guided_extension(self):
        """반대쪽이 없으면 안내 기반 연장도 하지 않는다."""
        curves=self.fragments()[1:]
        selected=select_ego_quadratics(SimpleNamespace(group_curves=curves),(180,320))
        self.assertIsNone(selected.left)
        self.assertEqual(selected.right.source_groups,(0,))

    def test_virtual_right_preserves_real_left_and_quadratic_curvature(self):
        """가상 오른쪽은 실제 왼쪽을 바꾸지 않고, 같은 곡률로 아래로 갈수록 넓어지게 만들며 지지 행은 0이다."""
        curves=[self.curve(lambda y:.001*y*y-1.1*y+220)]
        result=SimpleNamespace(group_curves=curves)
        before=select_ego_quadratics(result,(180,320))
        after=add_virtual_side(before,result,(180,320),width_at_reference=150)
        self.assertIs(after.left,before.left)
        self.assertTrue(after.right.is_virtual)
        self.assertEqual(after.right.width_source,'provided_width')
        self.assertEqual(after.left.coefficients[0],after.right.coefficients[0])
        rows=after.right.points[:,1]
        widths=after.right.points[:,0]-np.polyval(after.left.coefficients,rows)
        self.assertTrue(np.all(widths>0))
        self.assertTrue(np.all(np.diff(widths)>0))
        self.assertEqual(after.right.support_rows,0)

    def test_virtual_left_is_explicitly_virtual(self):
        """가상 왼쪽은 명시적으로 is_virtual로 표시된다."""
        result=SimpleNamespace(group_curves=[self.curve(lambda y:180+.8*(y-85))])
        before=select_ego_quadratics(result,(180,320))
        after=add_virtual_side(before,result,(180,320))
        self.assertIs(after.right,before.right)
        self.assertTrue(after.left.is_virtual)

    def test_virtual_fallback_does_not_change_real_pair(self):
        """실제 쌍이 있으면 가상 보완을 하지 않는다."""
        result=SimpleNamespace(group_curves=self.fragments()[:2])
        before=select_ego_quadratics(result,(180,320))
        self.assertIs(add_virtual_side(before,result,(180,320)),before)

    def test_virtual_fallback_does_not_create_both_missing_sides(self):
        """양쪽 다 없으면 가상 선도 만들지 않는다."""
        result=SimpleNamespace(group_curves=[])
        before=select_ego_quadratics(result,(180,320))
        self.assertIs(add_virtual_side(before,result,(180,320)),before)

    def test_virtual_fallback_rejects_invalid_prior(self):
        """음수나 NaN 폭은 거부한다."""
        result=SimpleNamespace(group_curves=[self.curve(lambda y:140-.8*(y-85))])
        before=select_ego_quadratics(result,(180,320))
        for width in (-1,float('nan')):
            with self.assertRaises(ValueError):
                add_virtual_side(before,result,(180,320),width_at_reference=width)

    def test_incompatible_short_fragment_cannot_drag_virtual_line(self):
        """맞지 않는 짧은 조각은 가상 선을 끌어당기지 못하고, 이미지 기본 폭(image_prior)을 쓴다."""
        left=self.curve(lambda y:140-.8*(y-85))
        y=np.arange(110,120,dtype=float)
        outer=np.column_stack((np.full(len(y),300),y,np.ones(len(y))))
        result=SimpleNamespace(group_curves=[left,outer])
        after=add_virtual_side(select_ego_quadratics(result,(180,320)),result,(180,320))
        self.assertEqual(after.right.width_source,'image_prior')
        self.assertEqual(after.right.source_groups,())


if __name__=='__main__':
    unittest.main()
