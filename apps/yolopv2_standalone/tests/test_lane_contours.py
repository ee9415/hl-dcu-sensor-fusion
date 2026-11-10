"""lane_contours(차선 마스크 → 윤곽/skeleton/조각 연결) 테스트."""
import unittest

import cv2
import numpy as np

from lane_contours import (extract_lane_contours, longitudinal_mask,
                           LanePiece, connect_lane_pieces, lane_group_curves,
                           observed_curve_segments)


class ContourTests(unittest.TestCase):
    def test_thick_lane_skeleton_is_central_and_inside(self):
        """두꺼운 세로 차선의 skeleton은 중앙(±1px)에 있고 마스크 안에 있으며, 그룹 곡선의 관측점도 skeleton 위에 있다."""
        mask=np.zeros((180,320),np.uint8)
        mask[20:165,70:81]=1
        result=extract_lane_contours(mask)
        y,x=np.nonzero(result.skeleton_mask)
        self.assertGreater(np.ptp(y),125)
        self.assertTrue(np.all(abs(x-75)<=1))
        self.assertTrue(np.all(mask[y,x]))
        curve=result.group_curves[0]
        observed=curve[curve[:,2]>0,:2].astype(int)
        self.assertTrue(np.all(result.skeleton_mask[observed[:,1],observed[:,0]]))

    def test_curved_skeleton_and_holes_stay_inside_mask(self):
        """곡선 차선과 구멍이 있어도 skeleton과 관측점은 마스크를 벗어나지 않는다."""
        mask=np.zeros((180,320),np.uint8)
        rows=np.arange(20,170)
        points=np.column_stack((np.rint(80+.004*(rows-85)**2),rows)).astype(np.int32)
        cv2.polylines(mask,[points],False,1,7)
        mask[60:64,80:82]=0
        result=extract_lane_contours(mask)
        self.assertTrue(np.all(result.skeleton_mask<=mask))
        for curve in result.group_curves:
            observed=curve[curve[:,2]>0,:2].astype(int)
            self.assertTrue(np.all(result.skeleton_mask[observed[:,1],observed[:,0]]))
        self.assertGreater(np.ptp(np.nonzero(result.skeleton_mask)[1]),15)

    def test_observed_endpoints_do_not_make_gap_observed(self):
        """양 끝점이 관측됐다고 그 사이 빈 구간까지 관측으로 치지 않는다."""
        mask=np.zeros((20,20),np.uint8)
        mask[5,5]=mask[6,10]=1
        curve=np.array([[5,5,1],[10,6,1]],float)
        self.assertFalse(observed_curve_segments(curve,mask)[0])

    @staticmethod
    def piece(start,stop,x,slope,width=4):
        """start~stop 행에 걸친 직선 차선 조각(LanePiece)을 만든다."""
        rows=np.arange(start,stop,dtype=float)
        centers=x+slope*(rows-start)
        contour=np.column_stack((centers,rows)).astype(np.int32)[:,None,:]
        return LanePiece(contour,rows,centers,np.full(len(rows),width))

    def test_overlapping_curve_fragments_associate(self):
        """행이 겹치고 방향이 이어지는 두 조각은 하나의 그룹으로 연결된다."""
        pieces=[self.piece(80,110,80,1.1),self.piece(107,145,110,1.2)]
        groups,links=connect_lane_pieces(pieces,(180,320))
        self.assertEqual(groups,[[0,1]])
        self.assertEqual(links,[(1,0)])

    def test_parallel_neighbor_does_not_join(self):
        """옆 차로의 평행한 조각은 연결하지 않는다."""
        pieces=[self.piece(80,110,80,.8),self.piece(114,150,140,.8)]
        self.assertEqual(connect_lane_pieces(pieces,(180,320))[1],[])

    def test_opposing_distant_lanes_do_not_rejoin(self):
        """기울기 방향이 반대인 조각은 연결하지 않는다."""
        pieces=[self.piece(80,110,100,-1),self.piece(111,150,102,1)]
        self.assertEqual(connect_lane_pieces(pieces,(180,320))[1],[])

    def test_smooth_gap_is_inferred_without_changing_observations(self):
        """매끄럽게 이어지는 빈 구간(8행)은 추정값(관측 플래그 0)으로 채우고, 관측점은 바꾸지 않는다."""
        pieces=[self.piece(80,100,80,.8),self.piece(108,140,102.4,.8)]
        groups,links=connect_lane_pieces(pieces,(180,320))
        self.assertEqual(links,[(1,0)])
        curve=lane_group_curves(pieces,groups)[0]
        self.assertEqual(int((curve[:,2]==0).sum()),8)
        np.testing.assert_allclose(np.diff(curve[:,0]),.8,atol=1e-8)

    def test_intermediate_piece_cannot_be_skipped(self):
        """중간 조각을 건너뛰고 앞뒤 조각을 바로 잇지 않는다."""
        pieces=[self.piece(80,100,80,1),self.piece(102,110,102,1),
                self.piece(113,145,113,1)]
        groups,links=connect_lane_pieces(pieces,(180,320),ambiguity_margin=0.)
        self.assertNotIn((2,0),links)
    def test_parallel_dashes_stay_separate(self):
        """평행한 두 점선 차선은 각각 3조각짜리 그룹으로 분리되고, 윤곽은 마스크 안에 있다."""
        mask = np.zeros((180, 320), np.uint8)
        for x in (80, 220):
            for y in (80, 110, 140):
                mask[y:y+16, x:x+4] = 255
        before = mask.copy()
        result = extract_lane_contours(mask)
        self.assertEqual(sorted(map(len, result.groups)), [3, 3])
        np.testing.assert_array_equal(mask, before)
        for piece in result.pieces:
            xy = piece.contour[:, 0]
            self.assertTrue(np.all(mask[xy[:, 1], xy[:, 0]] > 0))

    def test_ambiguous_fork_remains_separate(self):
        """갈라지는(후보가 둘인) 경우는 어느 쪽에도 연결하지 않는다."""
        mask = np.zeros((180, 320), np.uint8)
        mask[140:160, 99:102] = 1
        mask[110:125, 96:99] = 1
        mask[110:125, 102:105] = 1
        self.assertEqual(extract_lane_contours(mask).links, [])

    def test_horizontal_and_empty(self):
        """빈 마스크는 그룹이 없고, 가로 띠는 거부된 윤곽으로 분류된다."""
        mask = np.zeros((180, 320), np.uint8)
        self.assertEqual(extract_lane_contours(mask).groups, [])
        mask[100:104, 50:150] = 1
        result = extract_lane_contours(mask)
        self.assertEqual(len(result.rejected_contours), 1)
        self.assertEqual(result.pieces, [])

    def test_sloped_dashes(self):
        """기울어진 점선도 하나의 그룹으로 연결된다."""
        mask = np.zeros((180, 320), np.uint8)
        for y in (80, 110, 140):
            cv2.line(mask, (100+y//3, y), (100+(y+16)//3, y+16), 1, 3)
        self.assertEqual(len(extract_lane_contours(mask).groups), 1)

    def test_curve_survives_transverse_branch_removal(self):
        """가로 막대를 제거하는 방향 필터를 거쳐도 곡선 차선은 95% 이상 남는다."""
        curve = np.zeros((180, 320), np.uint8)
        rows = np.arange(20, 170)
        columns = np.rint(80 + .004 * (rows-85)**2).astype(int)
        points = np.column_stack((columns, rows)).astype(np.int32)
        cv2.polylines(curve, [points], False, 1, 3)
        mask = curve.copy()
        cv2.line(mask, (20,100), (290,100), 1, 3)
        kept = longitudinal_mask(mask)
        self.assertGreater((kept & curve).sum()/curve.sum(), .95)
        # Far from the intersection the transverse bar must disappear.
        self.assertEqual(int(kept[98:103,180:280].sum()), 0)
        self.assertTrue(np.all(kept <= mask))

    def test_shallow_perspective_lane_survives(self):
        """원근 때문에 많이 누운 차선도 방향 필터에서 살아남는다."""
        mask = np.zeros((180, 320), np.uint8)
        cv2.line(mask, (30,40), (290,150), 1, 3)
        kept = longitudinal_mask(mask)
        self.assertGreater(kept.sum()/mask.sum(), .95)

    def test_multiple_crosswalk_bars_are_removed(self):
        """횡단보도 가로줄들은 모두 제거된다."""
        mask = np.zeros((180, 320), np.uint8)
        for y in (60,90,130):
            cv2.line(mask, (20,y), (290,y), 1, 3)
        self.assertFalse(longitudinal_mask(mask).any())

    def test_invalid_mask_is_rejected(self):
        """3채널 또는 NaN 마스크는 거부한다."""
        for mask in (np.zeros((3,4,3)), np.array([[np.nan]])):
            with self.assertRaises(ValueError):
                extract_lane_contours(mask)

    def test_final_contours_preserve_curved_lane(self):
        """곡선 차선은 최종적으로 하나의 조각으로 보존된다."""
        mask = np.zeros((180,320),np.uint8)
        rows = np.arange(20,170)
        points = np.column_stack((np.rint(80+.004*(rows-85)**2),rows)).astype(np.int32)
        cv2.polylines(mask,[points],False,1,3)
        result = extract_lane_contours(mask)
        self.assertEqual(len(result.pieces),1)
        self.assertGreater(result.directional_mask.sum()/mask.sum(),.95)

    def test_distant_horizontal_bridge_is_split(self):
        """원거리에서 두 차선을 잇는 가로 연결부는 끊어 두 조각으로 나눈다."""
        mask = np.zeros((180,320),np.uint8)
        cv2.line(mask,(50,170),(140,30),1,3)
        cv2.line(mask,(270,170),(180,30),1,3)
        cv2.line(mask,(140,30),(180,30),1,3)
        result = extract_lane_contours(mask)
        self.assertGreaterEqual(len(result.pieces),2)
        self.assertFalse(result.directional_mask[30,155:165].any())
        self.assertTrue(result.directional_mask[140:,40:90].any())
        self.assertTrue(result.directional_mask[140:,230:280].any())

    def test_junction_pruning_preserves_curved_trunk(self):
        """교차점 가지치기 후에도 곡선 본줄기는 90% 이상 남고 가로 가지는 사라진다."""
        curve = np.zeros((180,320),np.uint8)
        rows = np.arange(20,170)
        points = np.column_stack((np.rint(80+.004*(rows-85)**2),rows)).astype(np.int32)
        cv2.polylines(curve,[points],False,1,3)
        mask = curve.copy()
        cv2.line(mask,(20,100),(290,100),1,3)
        kept = extract_lane_contours(mask).directional_mask
        self.assertGreater((kept & curve).sum()/curve.sum(),.90)
        self.assertEqual(int(kept[98:103,180:280].sum()),0)

    def test_direction_filter_can_be_disabled_for_comparison(self):
        """비교용으로 방향 필터를 끄면 마스크를 그대로 반환한다."""
        mask = np.zeros((180,320), np.uint8)
        mask[90:94,20:290] = 1
        np.testing.assert_array_equal(
            extract_lane_contours(mask, direction_filter=False).directional_mask,
            mask)


if __name__ == '__main__':
    unittest.main()
