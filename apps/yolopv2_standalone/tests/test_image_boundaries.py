"""image_boundaries.ImageBoundaryTracker(영상 좌표 경계선 추적기) 테스트.

페인트 검출, 영상 움직임 추정, 이력 융합, 부분 관측/짧은 조각 처리, 실측 고정 데이터(fixtures) 회귀를 다룬다.
"""
import unittest
from types import SimpleNamespace
import numpy as np
from image_boundaries import ImageBoundaryTracker,road_motion

class ImageBoundaryTests(unittest.TestCase):
    def scene(self,shift=0,curve=0,left=True,right=True):
        """회색 노면 위에 좌우 흰 차선을 그린 합성 RGB. shift=x 이동, curve=곡률, left/right=각 쪽 표시 여부."""
        image=np.full((180,320,3),60,np.uint8)
        for y in range(78,176):
            for visible,x in ((left,160-(y-65)*.9),(right,160+(y-65)*.9)):
                if visible:
                    x=int(x+shift+curve*((y-125)/50)**2)
                    image[y,max(0,x-1):min(320,x+2)]=230
        return image
    def update(self,t,image,n):
        """도로 전체·NN 차선 없음 조건으로 RGB만 이용해 n번째 프레임(0.1초 간격)을 갱신한다."""
        return t.update(np.ones((180,320),np.uint8),np.zeros((180,320),np.uint8),n*.1,rgb=image)
    def test_actual_paint_free_anchors_and_width(self):
        """실제 페인트 위치를 2px 이내로 찾고, 시작점이나 폭에 묶이지 않고 이동을 따라간다. 영상 좌표라 미터 폭은 보고하지 않는다."""
        t=ImageBoundaryTracker()
        a=self.update(t,self.scene(),0)
        self.assertTrue(a.pair_valid)
        self.assertEqual(a.reference_side,'both')
        for p,sign in ((a.image_left,-1),(a.image_right,1)):
            good=np.isfinite(p[:,0]);truth=160+sign*(p[good,1]-65)*.9
            self.assertLess(np.mean(abs(p[good,0]-truth)),2)
        b=self.update(t,self.scene(12),1)
        common=np.isfinite(a.image_left[:,0])&np.isfinite(b.image_left[:,0])
        self.assertGreater(np.mean(b.image_left[common,0]-a.image_left[common,0]),4)
        self.assertIsNone(b.telemetry()['corridor_width_m'])
    def test_no_initial_fabrication_and_one_side_loss(self):
        """처음부터 없는 경계선은 만들어 내지 않고, 한쪽을 잃으면 held/predicted로 낮은 신뢰도로 유지한 뒤 재관측 시 복귀한다."""
        t=ImageBoundaryTracker()
        empty=self.update(t,np.full((180,320,3),60,np.uint8),0)
        self.assertFalse(empty.valid)
        a=self.update(t,self.scene(),1)
        b=self.update(t,self.scene(right=False),2)
        self.assertTrue(b.pair_valid)
        self.assertIn(b.right_state,('held','predicted'))
        self.assertLess(b.right_confidence,a.right_confidence)
        c=self.update(t,self.scene(5),3)
        self.assertIn(c.right_state,('observed','aligning'))
    def test_affine_motion_translation_and_texture_rejection(self):
        """노면 텍스처로 영상 이동(아핀)을 추정하고, 텍스처가 없으면 신뢰도 0을 낸다."""
        rng=np.random.default_rng(4)
        image=rng.uniform(0,255,(90,160)).astype(np.float32)
        shifted=np.roll(np.roll(image,2,axis=0),3,axis=1)
        m,score=road_motion(image,shifted,np.ones_like(image,bool))
        self.assertGreater(score,.7)
        np.testing.assert_allclose(m,[[1,0,6],[0,1,4]],atol=.01)
        m,score=road_motion(np.zeros_like(image),np.zeros_like(image),np.ones_like(image,bool))
        self.assertEqual(score,0)
    def test_motion_warp_and_timestamp_reset(self):
        """추정 움직임으로 경계선을 옮기고, 시간이 역행하면 초기화한다."""
        t=ImageBoundaryTracker()
        a=self.update(t,self.scene(),0)
        p=t._warp(a.image_left,np.array([[1,0,6],[0,1,0]]))
        good=np.isfinite(p[:,0])&np.isfinite(a.image_left[:,0])
        np.testing.assert_allclose(p[good,0]-a.image_left[good,0],6)
        b=self.update(t,np.full((180,320,3),60,np.uint8),-1)
        self.assertFalse(b.valid)
    def test_object_mask_prevents_vehicle_paint(self):
        """객체(차량) 박스 안의 밝은 픽셀은 페인트로 쓰지 않는다."""
        t=ImageBoundaryTracker();image=self.scene()
        b=t.update(np.ones((180,320),np.uint8),np.zeros((180,320),np.uint8),0,rgb=image,
                   detections=[SimpleNamespace(xmin=0,ymin=0,xmax=1,ymax=1)])
        self.assertFalse(b.valid)

    def test_dark_seams_and_offroad_paint_are_rejected(self):
        """어두운 이음새나 도로 밖 페인트는 경계선 근거가 아니다."""
        image=self.scene();image[image==230]=105
        self.assertFalse(self.update(ImageBoundaryTracker(),image,0).valid)
        image=self.scene()
        t=ImageBoundaryTracker()
        b=t.update(np.zeros((180,320),np.uint8),np.zeros((180,320),np.uint8),0,rgb=image)
        self.assertFalse(b.valid)

    def test_shadow_paint_requires_network_and_local_contrast(self):
        """그늘 속 흐린 페인트는 NN 차선과 주변 대비가 함께 있어야 인정하고, 도로/객체 제외 마스크를 우회하지 못한다."""
        image=self.scene();lane=(image[:,:,0]==230).astype(np.uint8)
        image[image==230]=105
        road=np.ones((180,320),np.uint8)
        result=ImageBoundaryTracker().update(road,lane,0,rgb=image)
        self.assertEqual(result.reference_side,'both')
        self.assertTrue(result.pair_valid)
        # A network hallucination on uniform pavement is not paint evidence.
        flat=np.full_like(image,105)
        self.assertFalse(ImageBoundaryTracker().update(road,lane,0,rgb=flat).valid)
        # Shadow admission must not bypass either exclusion mask.
        self.assertFalse(ImageBoundaryTracker().update(road*0,lane,0,rgb=image).valid)
        box=SimpleNamespace(xmin=0,ymin=0,xmax=1,ymax=1)
        self.assertFalse(ImageBoundaryTracker().update(road,lane,0,rgb=image,detections=[box]).valid)

    def test_render_does_not_change_tracking_coordinates(self):
        """그리기 함수는 추적 좌표를 바꾸지 않는다."""
        from lane_boundaries import draw_boundaries,render_boundary_bev
        import json
        b=self.update(ImageBoundaryTracker(),self.scene(),0)
        original=b.image_left.copy()
        draw_boundaries(np.zeros((540,960,3),np.uint8),b,(320,180))
        render_boundary_bev(b)
        np.testing.assert_array_equal(original,b.image_left)
        json.dumps(b.telemetry())

    def test_partial_reacquisition_cannot_amplify_correction(self):
        """일부만 다시 잡힌 관측이 크게 어긋나 있어도 보정량은 한 프레임 한도(8px)로 제한된다."""
        from unittest.mock import patch
        t=ImageBoundaryTracker()
        initial=self.update(t,self.scene(),0)
        previous=initial.image_left.copy()
        measured=previous.copy();measured[:,0]+=90
        measured[:10,0]=np.nan
        t.history=[previous.copy(),initial.image_right.copy()]
        with patch('image_boundaries.road_motion',return_value=(np.array([[1,0,0],[0,1,0]]),1.)), \
             patch.object(t,'_detect',side_effect=[(measured,.95),None]):
            result=self.update(t,self.scene(),1)
        common=np.isfinite(previous[:,0])&np.isfinite(result.image_left[:,0])
        self.assertGreater(common.sum(),12)
        self.assertLessEqual(np.max(abs(previous[common,0]-result.image_left[common,0])),8.01)

    def test_lost_paint_follows_measured_image_motion(self):
        """페인트를 잃으면 측정된 영상 움직임을 따라 경계선을 예측(predicted)한다."""
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        texture=np.random.default_rng(8).uniform(25,100,(90,160)).astype(np.float32)
        t.previous=texture
        moved=np.roll(np.roll(texture,2,axis=0),3,axis=1)
        image=np.repeat(np.repeat(moved,2,axis=0),2,axis=1)
        image=np.repeat(image[:,:,None],3,axis=2).astype(np.uint8)
        b=self.update(t,image,1)
        self.assertEqual(b.left_state,'predicted')
        self.assertGreater(b.motion_confidence,.7)
        expected=t._warp(a.image_left,b.motion_matrix)
        common=np.isfinite(expected[:,0])&np.isfinite(b.image_left[:,0])
        self.assertLess(np.mean(abs(expected[common,0]-b.image_left[common,0])),.5)

    def test_long_occlusion_preserves_pair_and_damps_drift(self):
        """오랫동안 가려져도 쌍을 유지하되 움직임에 따른 표류는 점점 줄어들고, 신뢰도는 거의 0까지 내려간다. 다시 보이면 복귀한다."""
        from unittest.mock import patch
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        blank=np.full((180,320,3),60,np.uint8)
        flow=np.array([[1,0,1.5],[0,1,2.]])
        with patch('image_boundaries.road_motion',return_value=(flow,.9)):
            for i in range(1,301):
                b=self.update(t,blank,i)
                self.assertTrue(b.pair_valid)
                # Preserve the supported extent, not unobserved full-height extrapolation.
                for p in (b.image_left,b.image_right):
                    self.assertGreaterEqual(np.isfinite(p[:,0]).sum(),8)
                if i==200:late=b.image_left.copy()
        self.assertLess(np.nanmax(abs(b.image_left-late)),.05)
        self.assertLess(b.left_confidence,.01)
        recovered=self.update(t,self.scene(),301)
        self.assertTrue(recovered.pair_valid)
        self.assertIn(recovered.left_state,('observed','aligning'))

    def test_partial_curve_is_never_extrapolated_to_horizon(self):
        """일부만 관측된 곡선을 지평선까지 외삽하지 않는다."""
        t=ImageBoundaryTracker()
        u=(t.rows-125)/50
        points=np.column_stack((230+35*u-53.2891*u*u,t.rows))
        points[t.rows<120,0]=np.nan
        completed=t._complete(points)
        self.assertTrue(np.isnan(completed[t.rows<120,0]).all())
        np.testing.assert_allclose(completed[t.rows>=120],points[t.rows>=120])

    def test_crossed_candidates_cannot_poison_valid_history(self):
        """좌우가 뒤바뀐 잘못된 후보는 검증된 이전 쌍을 오염시키지 못하고, 이전 쌍을 낮은 신뢰도로 복원한다."""
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        previous=[a.image_left.copy(),a.image_right.copy()]
        # Deliberately invalid predictions and candidates must restore the
        # separately validated pair, with no false observed confidence.
        crossed=[previous[1].copy(),previous[0].copy()]
        b=t._fuse([(crossed[0],.99),(crossed[1],.99)],crossed,
                  np.array([[1,0,0],[0,1,0]]),0,.1)
        self.assertTrue(b.pair_valid)
        self.assertTrue(b.restored_pair)
        self.assertEqual(b.reference_side,'none')
        self.assertEqual((b.left_state,b.right_state),('held','held'))
        self.assertLessEqual(max(b.left_confidence,b.right_confidence),.15)
        for actual,expected in zip(t.history,previous):
            np.testing.assert_allclose(actual,expected,equal_nan=True)

    def test_partial_observation_preserves_one_quadratic(self):
        """부분 관측으로 보정해도 경계선은 하나의 2차식 형태를 유지하고, 다른 쪽은 그대로다."""
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        prior=[a.image_left.copy(),a.image_right.copy()]
        measured=prior[0].copy();measured[:,0]+=5
        measured[:20,0]=np.nan
        b=t._fuse([(measured,.9),None],prior,np.array([[1,0,0],[0,1,0]]),0,.1)
        good=np.isfinite(b.image_left[:,0])
        c=np.polynomial.polynomial.polyfit(b.image_left[good,1],b.image_left[good,0],2)
        np.testing.assert_allclose(np.polynomial.polynomial.polyval(b.image_left[good,1],c),b.image_left[good,0],atol=1e-8)
        self.assertLess(np.nanmax(abs(b.image_left[:,0]-prior[0][:,0])),8.01)
        np.testing.assert_allclose(b.image_right,prior[1],equal_nan=True)
        self.assertEqual(b.reference_side,'left')

    def test_stale_side_requires_measured_width_and_stable_opposite(self):
        """어긋난 오래된 한쪽은 학습된 폭과 안정적인 반대쪽을 기준으로 프레임당 2px 이내로 천천히 바로잡힌다."""
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        for i in range(1,10):self.update(t,self.scene(),i)
        baseline=t.history[1].copy()
        # Simulate drift of the old right boundary, retaining learned width.
        t.history[1][:,0]+=15;t.last_valid_pair=[p.copy() for p in t.history]
        start=t.history[1].copy()
        inferred=0
        for i in range(10,40):
            before=t.history[1].copy()
            b=self.update(t,self.scene(right=False),i)
            inferred+=b.right_state=='inferred'
            self.assertTrue(b.pair_valid)
            self.assertLessEqual(np.nanmax(abs(b.image_right[:,0]-before[:,0])),2.01)
        good=np.isfinite(baseline[:,0])&np.isfinite(b.image_right[:,0])
        self.assertLess(np.mean(abs(b.image_right[good,0]-baseline[good,0])),
                        np.mean(abs(start[good,0]-baseline[good,0])))
        self.assertGreater(inferred,0)
        self.assertLessEqual(b.right_confidence,.15)

    def test_no_unlearned_width_drags_existing_side(self):
        """폭을 학습하지 않았으면 반대쪽을 기준으로 기존 경계선을 끌어당기지 않는다."""
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        t.width_samples[:]=0
        before=t.history[1].copy()
        for i in range(1,40):b=self.update(t,self.scene(right=False),i)
        np.testing.assert_allclose(b.image_right,before,equal_nan=True,atol=1e-8)

    def test_partial_join_is_bounded_over_many_frames(self):
        """부분 관측을 여러 프레임 이어 붙여도 쌍과 형태가 계속 유효하다."""
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        p=t.history[0].copy();p[:,0]+=40;p[t.rows<140,0]=np.nan
        for _ in range(30):
            b=t._fuse([(p,.9),None],[v.copy() for v in t.history],np.eye(2,3),0,.1)
            self.assertTrue(b.pair_valid)
            self.assertTrue(t._smooth_shape(b.image_left))

    def test_shape_gate_keeps_smooth_curve_rejects_fold(self):
        """매끄러운 곡선은 통과시키고 접히거나 꺾인 형태는 거부한다."""
        t=ImageBoundaryTracker();u=(t.rows-125)/50
        p=np.column_stack((85+20*u+15*u*u,t.rows))
        self.assertTrue(t._smooth_shape(p))
        p[20,0]+=30
        self.assertFalse(t._smooth_shape(p))

    def test_short_fragment_only_corrects_owned_support(self):
        """짧은 조각은 이전값이 있을 때만 낮은 신뢰도로 쓰이며, 조각이 있는 구간만 보정한다."""
        t=ImageBoundaryTracker();mask=np.zeros((180,320),bool)
        for y in range(164,175):mask[y,200+(y-164)]=True
        self.assertIsNone(t._detect(mask,1,None))
        prior=np.column_stack((200+(t.rows-164),t.rows))
        found=t._detect(mask,1,prior)
        self.assertIsNotNone(found)
        p,score=found
        self.assertLess(score,.65)
        self.assertTrue(np.isnan(p[t.rows<164,0]).all())
        self.assertGreaterEqual(np.isfinite(p[:,0]).sum(),5)

    def test_long_gap_and_off_direction_reflection_are_not_bridged(self):
        """멀리 떨어져 있거나 방향이 다른 반사광은 이어 붙이지 않는다."""
        t=ImageBoundaryTracker();mask=np.zeros((180,320),bool)
        for y in range(164,175):mask[y,200+(y-164)]=True
        for y in range(130,145):mask[y,240]=True
        prior=np.column_stack((200+(t.rows-164),t.rows))
        p,score=t._detect(mask,1,prior)
        self.assertTrue(np.isnan(p[t.rows<164,0]).all())

    def test_collinear_second_dash_is_claimed_but_not_the_gap(self):
        """같은 직선 위 두 번째 점선은 지지점으로 인정하지만 그 사이 빈 구간은 인정하지 않는다."""
        t=ImageBoundaryTracker();mask=np.zeros((180,320),bool)
        for y in list(range(164,175))+list(range(154,159)):
            mask[y,200+(y-164)]=True
        prior=np.column_stack((200+(t.rows-164),t.rows))
        p,score=t._detect(mask,1,prior)
        self.assertTrue(np.isfinite(p[t.rows==164,0]).all())
        self.assertLess(abs(p[t.rows==154,0][0]-190),1.)
        self.assertTrue(np.isnan(p[t.rows==160,0]).all())
        self.assertLess(score,.65)

    def test_offset_second_fragment_is_not_chained(self):
        """방향은 같아도 옆으로 어긋난 조각은 같은 선으로 잇지 않는다."""
        t=ImageBoundaryTracker();mask=np.zeros((180,320),bool)
        for y in range(164,175):mask[y,200+(y-164)]=True
        # Same direction, but laterally offset: not one straight marking.
        for y in range(150,159):mask[y,205+(y-164)]=True
        prior=np.column_stack((200+(t.rows-164),t.rows))
        p,score=t._detect(mask,1,prior)
        self.assertTrue(np.isnan(p[t.rows<162,0]).all())

    def test_partial_recovery_preserves_shape_without_creating_a_seam(self):
        """부분 복구는 전체 형태를 평행 이동시켜 이음매를 만들지 않고, partial 상태와 낮은 신뢰도로 표시한다."""
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        original=t.history[1].copy();p=original.copy();p[:,0]+=20
        p[t.rows<160,0]=np.nan
        for _ in range(30):
            b=t._fuse([None,(p,.55)],[v.copy() for v in t.history],np.eye(2,3),0,.1)
        good=np.isfinite(original[:,0])
        delta=b.image_right[good,0]-original[good,0]
        self.assertLess(np.ptp(delta),1e-8)
        self.assertEqual(b.right_state,'partial')
        self.assertLess(b.right_confidence,.65)

    # Halla campus frame (2026-10-23 multiview): bright overcast pavement,
    # dashed right marking behind parked cars, lamp post above the road apex.
    # 320x180 RGB/drivable mask/vehicle boxes rebuilt from the screenshot;
    # truth points are hand-labelled paint centres in tracking pixels.
    def parked_scene(self):
        """실측 고정 데이터(주차 차량 뒤 오른쪽 점선)를 불러와 (데이터, (도로, 빈 차선), 차량 박스)를 반환한다."""
        from pathlib import Path
        f=np.load(Path(__file__).with_name('fixtures')/'parked_dashed_right_20261023.npz')
        boxes=[SimpleNamespace(xmin=a,ymin=b,xmax=c,ymax=d) for a,b,c,d in f['boxes']]
        return f,(f['road'].astype(np.uint8)*255,np.zeros((180,320),np.uint8)),boxes
    @staticmethod
    def truth_error(points,truth):
        """추적 경계선과 수동 라벨 점 사이의 x 오차(px). 라벨이 경계선 행 범위 안에 있는 것만 비교한다."""
        good=np.isfinite(points[:,0]);rows=points[good,1][::-1];xs=points[good,0][::-1]
        inside=(truth[:,1]>=rows[0])&(truth[:,1]<=rows[-1])
        return abs(np.interp(truth[inside,1],rows,xs)-truth[inside,0])
    def test_parked_scene_two_dashes_correct_stale_right_slope(self):
        """실측 장면에서 잘못 기울어진 오른쪽 이전값을 두 점선 조각으로 3px 이내까지 바로잡는다. 점선 사이 빈 구간은 지지점이 아니다."""
        f,(road,lane),boxes=self.parked_scene();t=ImageBoundaryTracker()
        def line(points):
            x=np.interp(t.rows,points[::-1,1],points[::-1,0],left=np.nan,right=np.nan)
            return np.column_stack((x,t.rows))
        # Device state seen on screen: right too steep, left bent onto the post.
        pair=[line(f['left_prior']),line(f['right_prior'])]
        upper=f['right_truth'][:,1]<=154
        self.assertGreater(self.truth_error(pair[1],f['right_truth'])[upper[2:]].min(),10)
        t.history=[p.copy() for p in pair];t.last_valid_pair=[p.copy() for p in pair]
        t.scores=[.64,.35];t.last_time=-.1
        for i in range(10):
            b=t.update(road,lane,i*.1,rgb=f['rgb'],detections=boxes)
        self.assertTrue(b.pair_valid)
        self.assertEqual(b.right_state,'partial')
        self.assertLess(b.right_confidence,.65)
        error=self.truth_error(b.image_right,f['right_truth'])
        self.assertLess(error.max(),3.)
        # Both dashes are current support; the gap between them is not.
        rows=b.image_right[b.right_support,1]
        self.assertTrue(any(abs(rows-152)<=1) and any(abs(rows-170)<=1))
        self.assertFalse(any((rows>156)&(rows<164)))
        self.assertLess(self.truth_error(b.image_left,f['left_truth']).max(),3.)
    def test_parked_scene_lamp_post_is_never_boundary_support(self):
        """실측 장면의 가로등 기둥은 어느 쪽 경계선 지지점으로도 쓰이지 않는다."""
        f,(road,lane),boxes=self.parked_scene();t=ImageBoundaryTracker()
        lo,hi=f['lamp_post_x']
        for i in range(5):
            b=t.update(road,lane,i*.1,rgb=f['rgb'],detections=boxes)
            for side in ('left','right'):
                p=getattr(b,'image_'+side);support=getattr(b,side+'_support')
                if support is None or not support.any():continue
                post=support&(p[:,1]<134)&(p[:,0]>=lo-2)&(p[:,0]<=hi+2)
                self.assertFalse(post.any(),side)
    def test_worn_paint_on_bright_pavement_is_detected(self):
        """밝은 노면 위 닳은 페인트(대비가 약함)도 검출한다."""
        image=self.scene();image[image==60]=180;image[image==230]=200
        b=self.update(ImageBoundaryTracker(),image,0)
        self.assertTrue(b.pair_valid)
        self.assertEqual((b.left_state,b.right_state),('observed','observed'))
    def test_vertical_lane_survives_truncated_road_mask(self):
        """도로 마스크가 아래쪽만 있어도 수직 차선을 놓치지 않는다."""
        image=np.full((180,320,3),60,np.uint8)
        image[110:176,99:102]=230;image[110:176,219:222]=230
        road=np.zeros((180,320),np.uint8);road[130:]=1
        t=ImageBoundaryTracker()
        for i in range(3):
            b=t.update(road,np.zeros_like(road),i*.1,rgb=image)
            self.assertTrue(b.pair_valid)
            self.assertEqual((b.left_state,b.right_state),('observed','observed'))
            for p,x in ((b.image_left,100),(b.image_right,220)):
                good=np.isfinite(p[:,0])
                self.assertLess(np.max(abs(p[good,0]-x)),3)

    def test_quadratic_invariant_through_loss_recovery_and_motion(self):
        """손실·복구·이동을 거쳐도 경계선 점은 텔레메트리의 2차식 계수와 정확히 일치한다."""
        t=ImageBoundaryTracker()
        for i in range(45):
            frame=(self.scene(curve=8) if i<10 else self.scene(right=False)
                   if i<25 else self.scene(shift=12,curve=-4))
            b=self.update(t,frame,i)
            self.assertTrue(b.pair_valid)
            payload=b.telemetry()
            for side in ('left','right'):
                p=getattr(b,'image_'+side);good=np.isfinite(p[:,0])
                c=payload[side+'_image_coefficients']
                self.assertEqual(len(c),3)
                np.testing.assert_allclose(np.polynomial.polynomial.polyval(p[good,1],c),p[good,0],atol=1e-7)
        warped=t._warp(b.image_left,np.array([[1,.02,1.],[-.01,1.,2.]]))
        good=np.isfinite(warped[:,0])
        c=np.polynomial.polynomial.polyfit(warped[good,1],warped[good,0],2)
        np.testing.assert_allclose(np.polynomial.polynomial.polyval(warped[good,1],c),warped[good,0],atol=1e-7)

    def test_regions_connect_top_bottom_without_middle(self):
        """위/아래 영역에서 찾은 조각을 이어 하나의 곡선을 만들되, 가운데 빈 구간은 지지점으로 표시하지 않는다."""
        t=ImageBoundaryTracker();mask=np.zeros((180,320),bool)
        for lo,hi in ((84,102),(156,176)):
            for y in range(lo,hi):
                x=int(160+(y-65)*.7);mask[y,x:x+2]=True
        found=t._detect_regions(mask,1,None)
        self.assertIsNotNone(found)
        curve,score,support=found
        self.assertTrue(support[t.rows<104].any())
        self.assertTrue(support[t.rows>154].any())
        self.assertFalse(support[(t.rows>110)&(t.rows<150)].any())
        self.assertTrue(np.isfinite(curve[(t.rows>110)&(t.rows<150),0]).all())
        self.assertLess(np.max(abs(curve[np.isfinite(curve[:,0]),0]-(160+(curve[np.isfinite(curve[:,0]),1]-65)*.7))),2)

    def test_regions_reject_wrong_direction_upper_fragment(self):
        """방향이 반대인 위쪽 조각은 아래쪽과 잇지 않는다."""
        t=ImageBoundaryTracker();mask=np.zeros((180,320),bool)
        for y in range(156,176):mask[y,int(160+(y-65)*.7)]=True
        for y in range(84,102):mask[y,int(270-(y-84)*.7)]=True
        self.assertIsNone(t._detect_regions(mask,1,None))

    def test_regions_join_real_short_dashes(self):
        """실측 장면의 짧은 점선 조각들을 영역 검출로 이어 3px 이내 정확도로 찾는다."""
        f,(road,lane),boxes=self.parked_scene();t=ImageBoundaryTracker()
        original=t._detect_regions;results=[]
        def capture(paint,side,prior):
            """오른쪽(side=1) 영역 검출 결과를 가로채서 기록한다."""
            result=original(paint,side,prior)
            if side==1:results.append(result)
            return result
        t._detect_regions=capture
        t.update(road,lane,0,rgb=f['rgb'],detections=boxes)
        self.assertIsNotNone(results[0])
        curve,score,support=results[0]
        self.assertTrue(support[t.rows<156].any())
        self.assertTrue(support[t.rows>164].any())
        self.assertFalse(support[(t.rows>156)&(t.rows<164)].any())
        self.assertLess(self.truth_error(curve,f['right_truth']).max(),3.)

    def test_temporal_fragments_require_repeat_and_reset_on_loss(self):
        """시간 누적 조각은 3회 반복돼야 인정하고(신뢰도 0.55 이하), 조각이 사라지면 누적 횟수를 초기화한다."""
        t=ImageBoundaryTracker();prior=np.column_stack((200+(t.rows-164),t.rows))
        measured=prior.copy();measured[t.rows<164,0]=np.nan
        q=np.array([(200+y-164+2,y) for y in range(148,155,2)])
        t.region_segments[1]=[q]
        observation=(measured,.5)
        for _ in range(2):self.assertEqual(len(t._temporal_fragments(observation,1,prior)),2)
        result=t._temporal_fragments(observation,1,prior)
        self.assertTrue(result[3]);self.assertLessEqual(result[1],.55)
        self.assertTrue(result[2][t.rows<156].any())
        t.region_segments[1]=[]
        self.assertEqual(len(t._temporal_fragments(observation,1,prior)),2)
        self.assertEqual(t.fragment_hits[1].sum(),0)

    def test_temporal_fragments_reject_distant_repeat(self):
        """이전값에서 멀리 떨어진 조각은 반복돼도 인정하지 않는다."""
        t=ImageBoundaryTracker();prior=np.column_stack((200+(t.rows-164),t.rows))
        measured=prior.copy();measured[t.rows<164,0]=np.nan
        t.region_segments[1]=[np.array([(200+y-164+12,y) for y in range(148,155,2)])]
        for _ in range(5):self.assertEqual(len(t._temporal_fragments((measured,.5),1,prior)),2)
        self.assertEqual(t.fragment_hits[1].sum(),0)

    def test_temporal_correction_is_bounded_and_does_not_move_other_side(self):
        """시간 누적 조각 보정은 프레임당 0.5px로 제한되고, 반대쪽이나 폭 학습에는 영향을 주지 않는다."""
        t=ImageBoundaryTracker()
        left=np.column_stack((np.full(48,80.),t.rows))
        right=np.column_stack((200+(t.rows-164)*.5,t.rows))
        measured=right.copy();support=(t.rows>=148)&(t.rows<=154)
        measured[~support,0]=np.nan;measured[support,0]+=3
        b=t._fuse([None,(measured,.5,support,True)],[left,right],np.eye(2,3),0,.1)
        self.assertTrue(b.pair_valid)
        np.testing.assert_allclose(b.image_left,left,atol=1e-8)
        self.assertLessEqual(np.max(abs(b.image_right[:,0]-right[:,0])),.500001)
        self.assertEqual(b.right_state,'partial')
        self.assertEqual(t.width_samples.sum(),0)

    def test_real_left_short_fragment_replaces_duplicate_right_paint(self):
        """실측 장면에서 왼쪽이 오른쪽 페인트를 중복으로 잡은 상태를, 실제 왼쪽 짧은 조각으로 대체해 바로잡는다."""
        from pathlib import Path
        f=np.load(Path(__file__).with_name('fixtures')/'left_short_duplicate_20261023.npz')
        t=ImageBoundaryTracker();t.history=[f['left'].copy(),f['right'].copy()]
        self.assertFalse(t._plausible_pair(t.history))
        for _ in range(12):
            predicted=[p.copy() for p in t.history]
            obs=[t._detect(f['paint'],side,predicted[side]) for side in (0,1)]
            obs,predicted=t._resolve_duplicate_observations(obs,predicted)
            b=t._fuse(obs,predicted,np.eye(2,3),0,.1)
        near=np.isfinite(b.image_left[:,0])&(t.rows>=150)&(t.rows<=164)
        self.assertGreaterEqual(near.sum(),8)
        self.assertLess(np.max(b.image_left[near,0]),125)
        self.assertTrue(t._plausible_pair([b.image_left,b.image_right]))
        self.assertIsNone(t.last_valid_pair)
        self.assertEqual(t.width_samples.sum(),0)

    def test_wide_but_wrong_low_confidence_prior_can_reacquire(self):
        """폭은 그럴듯하지만 잘못된 이전값은 버리고 낮은 신뢰도로 다시 획득할 수 있다."""
        from pathlib import Path
        f=np.load(Path(__file__).with_name('fixtures')/'left_short_duplicate_20261023.npz')
        t=ImageBoundaryTracker()
        prior=[np.column_stack((np.full(48,50.),t.rows)),f['right'].copy()]
        self.assertTrue(t._plausible_pair(prior))
        for _ in range(3):
            obs=[t._detect(f['paint'],side,prior[side]) for side in (0,1)]
            obs,resolved=t._resolve_duplicate_observations(obs,prior)
        self.assertIsNone(resolved[0])
        self.assertLess(obs[0][1],.65)
        self.assertTrue(t._plausible_pair([obs[0][0],obs[1][0]]))

    def test_missing_candidate_clears_short_pair_confirmation(self):
        """후보가 없으면 짧은 쌍 확인 횟수를 초기화한다."""
        t=ImageBoundaryTracker();t.short_pair_hits=[4,3]
        t.short_pair_previous=[np.zeros((48,2)),np.zeros((48,2))]
        t._resolve_duplicate_observations([None,None],[None,None])
        self.assertEqual(t.short_pair_hits,[0,0])
        self.assertEqual(t.short_pair_previous,[None,None])

    def test_scene_reset_clears_temporal_and_reacquisition_state(self):
        """장면 리셋(시간 역행) 시 시간 누적·재획득 상태를 모두 초기화한다."""
        t=ImageBoundaryTracker();t.fragment_hits[:]=5;t.fragment_offsets[:]=2
        t.reacquire_hits=[5,5];t.short_pair_hits=[5,5];t.last_time=10.
        self.update(t,np.full((180,320,3),60,np.uint8),0)
        self.assertEqual(t.fragment_hits.sum(),0)
        self.assertEqual(t.reacquire_hits,[0,0])
        self.assertEqual(t.short_pair_hits,[0,0])

    def test_single_initial_side_supplies_low_confidence_pair(self):
        """처음에 한쪽만 보여도 반대쪽을 낮은 신뢰도의 inferred로 채운 쌍을 제공한다."""
        t=ImageBoundaryTracker();b=self.update(t,self.scene(right=False),0)
        self.assertTrue(b.pair_valid)
        self.assertEqual(b.right_state,'inferred')
        self.assertLess(b.right_confidence,b.left_confidence)
        self.assertIsNone(b.telemetry()['corridor_width_m'])

if __name__=='__main__':unittest.main()
