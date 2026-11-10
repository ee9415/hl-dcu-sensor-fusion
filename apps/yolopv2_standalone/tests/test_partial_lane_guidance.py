"""Regression for the reported almost-complete 58.5s left mask."""
import unittest
from types import SimpleNamespace
import numpy as np
from image_boundaries import ImageBoundaryTracker

class PartialLaneGuidanceTests(unittest.TestCase):
    def test_weaker_almost_complete_side_cannot_bend_the_strong_straight_side(self):
        rows=np.arange(82.,176.)
        right=np.column_stack((160+.9*(rows-65),rows,np.ones(len(rows))))
        left=np.column_stack((160-.9*(rows-65)+4*((rows-125)/50)**2,rows,np.ones(len(rows))))
        for missing in (0,1):
            tracker=ImageBoundaryTracker()
            full=[left.copy(),right.copy()]
            if missing==1:
                full[0][:,0]=160-.9*(rows-65)
                full[1][:,0]=160+.9*(rows-65)-4*((rows-125)/50)**2
            for i in range(20):tracker.update_contours(SimpleNamespace(group_curves=full),(180,320),i*.05)
            width=tracker.width.copy();counts=tracker.width_samples.copy()
            partial=[c.copy() for c in full];partial[missing]=partial[missing][partial[missing][:,1]<=158]
            for i in range(1,81):
                estimate=tracker.update_contours(SimpleNamespace(group_curves=partial),(180,320),.95+i*.05)
            name='left' if missing==0 else 'right';other='right' if missing==0 else 'left'
            self.assertEqual(getattr(estimate,name+'_state'),'inferred')
            self.assertAlmostEqual(tracker._contour_seen[missing],.95)
            actual=getattr(estimate,'image_'+other);good=np.isfinite(actual[:,0])
            self.assertLess(np.max(abs(actual[good,0]-np.polyval(np.polyfit(actual[good,1],actual[good,0],1),actual[good,1]))),.01)
            guided=getattr(estimate,'image_'+name);good &= np.isfinite(guided[:,0])
            gap=abs(actual[good,0]-guided[good,0])
            self.assertLess(np.max(abs(gap-np.polyval(np.polyfit(actual[good,1],gap,1),actual[good,1]))),.01)
            np.testing.assert_array_equal(width,tracker.width)
            np.testing.assert_array_equal(counts,tracker.width_samples)

    def test_seek_into_partial_pair_bootstraps_spacing_from_real_shared_rows(self):
        """부분 관측 장면으로 바로 탐색해도 실제 공통 관측으로 간격을 확보한다."""
        rows=np.arange(82.,176.)
        left=np.column_stack((160-.9*(rows-65)+4*((rows-125)/50)**2,rows,np.ones(len(rows))))
        right=np.column_stack((160+.9*(rows-65),rows,np.ones(len(rows))))
        left=left[left[:,1]<=158]
        tracker=ImageBoundaryTracker()
        for i in range(60):
            result=tracker.update_contours(SimpleNamespace(group_curves=[left,right]),(180,320),i*.05)
        self.assertIsNotNone(tracker.width)
        self.assertEqual(result.left_state,'inferred')
        self.assertTrue(result.detection_diagnostics[0]['partial_spacing_guidance'])
        self.assertFalse(result.left_support.any())
        self.assertLessEqual(tracker._contour_seen[0],.15)

    def test_new_supported_rows_join_filtered_curve_without_invalid_seam(self):
        """재검출로 늘어난 행이 원시 위치로 튀어 정상 관측까지 held로 되돌리지 않는다."""
        tracker=ImageBoundaryTracker();rows=np.arange(82.,176.)
        curves=[np.column_stack((160+sign*.9*(rows-65),rows,np.ones(len(rows)))) for sign in (-1,1)]
        near=[c[c[:,1]>=110].copy() for c in curves]
        for i in range(20):tracker.update_contours(SimpleNamespace(group_curves=near),(180,320),i*.05)
        moved=[c.copy() for c in curves]
        for c in moved:c[:,0]+=20.
        confirmed=False
        for i in range(1,31):
            result=tracker.update_contours(SimpleNamespace(group_curves=moved),(180,320),.95+i*.05)
            if result.detection_diagnostics[1].get('confirmed_reacquisition') or result.right_state=='observed':
                confirmed=True
                self.assertNotEqual(result.right_state,'held')
                self.assertTrue(result.right_valid)
                self.assertTrue(result.pair_valid)
        self.assertTrue(confirmed)
        self.assertEqual(result.right_state,'observed')

    def test_internal_missing_rows_follow_filtered_anchors_without_self_learning(self):
        """측정점 사이 162~164행만 옛 위치에 남아 정상 직선 전체를 탈락시키지 않는다."""
        tracker=ImageBoundaryTracker();rows=np.arange(82.,176.)
        curves=[np.column_stack((160+sign*.9*(rows-65),rows,np.ones(len(rows)))) for sign in (-1,1)]
        for i in range(20):tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),i*.05)
        moved=[c.copy() for c in curves]
        for c in moved:
            c[:,0]+=20.
            c[(c[:,1]>=160)&(c[:,1]<=166),2]=0.
        for i in range(1,31):
            result=tracker.update_contours(SimpleNamespace(group_curves=moved),(180,320),.95+i*.05)
            if result.detection_diagnostics[1].get('confirmed_reacquisition'):
                self.assertFalse(result.detection_diagnostics[1].get('pair_conflict_held',False))
        self.assertEqual(result.right_state,'observed')
        gap=(tracker.rows>=162)&(tracker.rows<=164)
        self.assertFalse(result.right_support[gap].any())
        self.assertTrue(result.pair_valid)

    def test_display_geometry_is_quadratic_while_raw_skeleton_is_preserved(self):
        """최종 표시 좌표는 2차식과 일치하고 관측 스켈레톤·지지점은 변조하지 않는다."""
        tracker=ImageBoundaryTracker();rows=np.arange(82.,176.)
        curves=[np.column_stack((160+sign*.9*(rows-65)+1.4*np.sin(rows/7),rows,np.ones(len(rows)))) for sign in (-1,1)]
        result=tracker.update_contours(SimpleNamespace(group_curves=curves),(180,320),0.)
        raw=result.image_left.copy();support=result.left_support.copy()
        payload=result.telemetry()
        self.assertEqual(payload['display_curve_degree'],2)
        display=np.array(payload['left_display_image']);coef=np.array(payload['left_display_coefficients'])
        np.testing.assert_allclose(display[:,0],np.polyval(coef,display[:,1]),atol=1e-9)
        np.testing.assert_allclose(display[:,0],raw[np.isfinite(raw[:,0]),0],atol=1e-8)
        self.assertGreater(np.max(abs(raw[np.isfinite(raw[:,0]),0]-np.interp(raw[np.isfinite(raw[:,0]),1],curves[0][:,1],curves[0][:,0]))),.5)
        np.testing.assert_array_equal(raw,result.image_left)
        np.testing.assert_array_equal(support,result.left_support)

    def test_tracked_far_lane_is_not_dropped_only_for_ending_at_row120(self):
        """기존 차선이 120행까지만 보여도 53행 실제 지지를 하단 기준 하나로 버리지 않는다."""
        tracker=ImageBoundaryTracker();near=np.arange(82.,176.)
        pair=[np.column_stack((160+sign*.9*(near-65),near,np.ones(len(near)))) for sign in (-1,1)]
        for i in range(20):tracker.update_contours(SimpleNamespace(group_curves=pair),(180,320),i*.05)
        far=np.arange(68.,121.)
        right=np.column_stack((160+.9*(far-65),far,np.ones(len(far))))
        for i in range(1,21):result=tracker.update_contours(SimpleNamespace(group_curves=[right]),(180,320),.95+i*.05)
        self.assertEqual(result.right_state,'observed')
        self.assertTrue(result.right_support.any())
        self.assertEqual(result.left_state,'inferred')
        self.assertFalse(result.left_support.any())

    def test_different_visible_domains_do_not_leave_old_nonquadratic_ends(self):
        """반대편 표시 범위가 짧아도 유지선 끝에 옛 곡률을 섞어 남기지 않는다."""
        tracker=ImageBoundaryTracker();rows=np.arange(82.,176.)
        left=np.column_stack((160-.9*(rows-65)+4*((rows-125)/50)**2,rows,np.ones(len(rows))))
        right=np.column_stack((160+.9*(rows-65),rows,np.ones(len(rows))))
        right=right[(right[:,1]>=100)&(right[:,1]<=160)]
        for i in range(20):tracker.update_contours(SimpleNamespace(group_curves=[left,right]),(180,320),i*.05)
        for i in range(1,81):
            result=tracker.update_contours(SimpleNamespace(group_curves=[right]),(180,320),.95+i*.05)
            for points in (result.image_left,result.image_right):
                good=np.isfinite(points[:,0]);coef=np.polyfit(points[good,1],points[good,0],2)
                np.testing.assert_allclose(points[good,0],np.polyval(coef,points[good,1]),atol=1e-8)
        self.assertEqual(result.left_state,'inferred')
        self.assertFalse(result.left_support.any())

if __name__=='__main__':unittest.main()
