"""lane_overlay(차선 점수 미리보기 오버레이) 테스트.

미리보기 모드/임계값은 화면 표시용이며, 인지용 차선 마스크(임계값 0.5)에는 영향을 주지 않아야 한다.
"""
import unittest
import numpy as np
from lane_overlay import (crop_lane_scores,render_lane_overlay,validate_preview,lane_score_colors,
                          RAINBOW_COLORS,RAINBOW_ANCHORS,reference_band_colors,BAND_COLORS)
from postprocess import segmentation_masks


class LaneOverlayTests(unittest.TestCase):
    def test_normal_rgb_uses_each_score_without_coloring_background(self):
        """normal 오버레이는 임계값 이상 픽셀만 점수별로 다르게 칠하고 배경은 원본 그대로 두며, 입력도 바꾸지 않는다."""
        from lane_overlay import overlay_pixel_confidence
        rgb=np.full((6,9,3),80,np.uint8)
        scores=np.array([[.4,.6,.65],[.4,.6,.65]],np.float32)
        before=scores.copy()
        result=overlay_pixel_confidence(rgb,scores,.5)
        np.testing.assert_array_equal(result[:,:3],rgb[:,:3])
        self.assertFalse(np.array_equal(result[0,3],result[0,6]))
        np.testing.assert_array_equal(scores,before)
        self.assertTrue(np.all(rgb==80))

    def test_square_and_rectangular_tensor_layouts_remove_letterbox(self):
        """정사각형/직사각형 모델 입력, NCHW/NHWC 배치 모두에서 레터박스 패딩을 정확히 잘라낸다."""
        for mh in (320,192):
            raw=np.zeros((1,1,mh,320),np.float32)
            top=(mh-180)//2
            content=np.linspace(0,1,320,dtype=np.float32)[None].repeat(180,axis=0)
            raw[0,0,top:top+180]=content
            for tensor in (raw,raw.transpose(0,2,3,1)):
                actual=crop_lane_scores(tensor,(180,320),(320,mh))
                np.testing.assert_array_equal(actual,content)

    def test_mask_threshold_is_inclusive_and_only_changes_preview(self):
        """lane_mask 임계값은 '이상(>=)'으로 적용되고, 미리보기 결과만 바뀐다."""
        scores=np.array([[.1,.5,.9]],np.float32)
        original=scores.copy();rgb=np.full((2,6,3),20,np.uint8)
        low=render_lane_overlay(rgb,scores,.5,'lane_mask')
        high=render_lane_overlay(rgb,scores,.8,'lane_mask')
        self.assertTrue(np.all(low[:,:2]==20))
        self.assertTrue(np.any(low[:,2:4]!=20))
        self.assertTrue(np.all(high[:,:4]==20))
        self.assertTrue(np.any(high[:,4:]!=20))
        np.testing.assert_array_equal(scores,original)
        self.assertTrue(np.all(rgb==20))

    def test_perception_mask_still_uses_half_for_every_preview_setting(self):
        """미리보기 임계값을 바꿔도 인지용 차선 마스크는 항상 0.5 기준으로 동일하다."""
        raw=np.full((1,1,320,320),.6,np.float32)
        road=np.zeros((1,2,320,320),np.float32)
        before=segmentation_masks(road,raw,(1920,1080))[1]
        scores=crop_lane_scores(raw,before.shape,(320,320))
        rgb=np.zeros((540,960,3),np.uint8)
        for threshold in (0,.2,.8,1):
            render_lane_overlay(rgb,scores,threshold,'lane_mask')
            np.testing.assert_array_equal(before,segmentation_masks(road,raw,(1920,1080))[1])
        self.assertTrue(np.all(before==255))

    def test_score_overlay_preserves_below_threshold_rgb_and_normal_is_exact(self):
        """lane_score는 임계값 미만 픽셀을 원본 그대로 두고, 차선이 없을 때 normal 결과는 원본과 정확히 같다."""
        rgb=np.full((2,6,3),30,np.uint8)
        scores=np.array([[0,.4,1]],np.float32)
        colored=render_lane_overlay(rgb,scores,.8,'lane_score')
        self.assertTrue(np.all(colored[:,:2]==30))
        self.assertTrue(np.all(colored[:,2:4]==30))
        self.assertGreater(colored[0,5,2],colored[0,5,0])
        np.testing.assert_array_equal(render_lane_overlay(rgb,scores,.5,'normal'),rgb)

    def test_invalid_settings_and_raw_logits_are_not_silently_used(self):
        """잘못된 모드/임계값, 0~1 범위 밖 점수(로짓), NaN, 잘못된 차원은 거부한다."""
        for mode,threshold in (('unknown',.5),('lane_mask',True),('lane_score',float('nan')),
                               ('lane_score',-.1),('normal',1.1),('lane_mask','0.5')):
            with self.assertRaises(ValueError):validate_preview(mode,threshold)
        for raw in (np.full((1,1,320,320),2.),np.full((1,1,320,320),np.nan),np.zeros((320,320))):
            with self.assertRaises(ValueError):crop_lane_scores(raw,(180,320),(320,320))

    def test_rainbow_uses_continuous_absolute_pixel_scores(self):
        """무지개 색상은 점수의 절대값으로 연속적으로 정해지며, 같은 프레임 안 다른 픽셀 값에 영향받지 않는다."""
        np.testing.assert_array_equal(lane_score_colors(np.array(RAINBOW_ANCHORS)),RAINBOW_COLORS)
        np.testing.assert_array_equal(lane_score_colors(np.array(RAINBOW_ANCHORS,dtype=np.float32)),RAINBOW_COLORS)
        np.testing.assert_array_equal(lane_score_colors(np.array([0.,1.])),RAINBOW_COLORS[[0,-1]])
        # A score's color must not change with the other pixels in its frame.
        np.testing.assert_array_equal(lane_score_colors(np.array([.3,.95]))[0],
                                      lane_score_colors(np.array([.3,.4]))[0])
        self.assertFalse(np.array_equal(lane_score_colors(.55),lane_score_colors(.60)))
        self.assertFalse(np.array_equal(lane_score_colors(.91),lane_score_colors(.99)))

    def test_same_lane_contains_distinct_pixel_colors_and_no_background_tint(self):
        """같은 차선 안에서도 픽셀별 점수에 따라 색이 다르고, 3배 확대 시 블록 단위로 정확히 칠해진다."""
        scores=np.full((3,5),.05,np.float32)
        scores[1]=[.55,.60,.75,.85,.95]
        rgb=np.arange(9*15*3,dtype=np.uint16).reshape(9,15,3).astype(np.uint8)
        out=render_lane_overlay(rgb,scores,.5,'lane_score')
        np.testing.assert_array_equal(out[:3],rgb[:3])
        np.testing.assert_array_equal(out[6:],rgb[6:])
        # Read each native score pixel's center after exact 3x enlargement.
        base=np.zeros_like(rgb)
        colors=render_lane_overlay(base,scores,.5,'lane_score')[4,1::3]
        self.assertEqual(len(np.unique(colors,axis=0)),5)
        for i in range(5):
            block=render_lane_overlay(base,scores,.5,'lane_score')[3:6,i*3:i*3+3]
            self.assertTrue(np.all(block==colors[i]))

    def test_threshold_selects_pixels_but_does_not_recolor_retained_pixels(self):
        """임계값은 표시할 픽셀만 고를 뿐, 남은 픽셀의 색은 바꾸지 않는다."""
        rgb=np.full((3,9,3),40,np.uint8)
        scores=np.array([[.25,.7,.95]],np.float32)
        low=render_lane_overlay(rgb,scores,.2,'lane_score')
        high=render_lane_overlay(rgb,scores,.5,'lane_score')
        self.assertTrue(np.any(low[:,:3]!=rgb[:,:3]))
        np.testing.assert_array_equal(high[:,:3],rgb[:,:3])
        np.testing.assert_array_equal(low[:,3:],high[:,3:])

    def test_reference_bands_match_lower_bound_labels(self):
        """참조 밴드 색상은 하한 기준(0.05, 0.15, …)으로 구간을 나눈다."""
        scores=np.array([0,.049,.05,.149,.15,.95,1],np.float32)
        np.testing.assert_array_equal(reference_band_colors(scores),BAND_COLORS[[0,0,1,1,2,10,10]])
        np.testing.assert_array_equal(BAND_COLORS[1],[0,0,255])
        np.testing.assert_array_equal(BAND_COLORS[10],[255,0,180])

    def test_reference_black_background_and_rgb_overlay_are_independent_options(self):
        """lane_bands는 검정 배경, lane_bands_rgb는 RGB 위 오버레이로 서로 독립적인 옵션이다."""
        scores=np.array([[.02,.1,.99]],np.float32)
        rgb=np.full((3,9,3),80,np.uint8)
        black=render_lane_overlay(rgb,scores,.05,'lane_bands')
        np.testing.assert_array_equal(black,render_lane_overlay(np.zeros_like(rgb),scores,.05,'lane_bands'))
        self.assertTrue(np.all(black[:,:3]==0))
        self.assertTrue(np.all(black[:,3:6]==BAND_COLORS[1]))
        self.assertTrue(np.all(black[:,6:]==BAND_COLORS[10]))
        overlay=render_lane_overlay(rgb,scores,.05,'lane_bands_rgb')
        np.testing.assert_array_equal(overlay[:,:3],rgb[:,:3])
        self.assertTrue(np.any(overlay[:,3:]!=rgb[:,3:]))
        self.assertTrue(np.all(rgb==80))

    def test_reference_floor_keeps_under_point05_hidden_and_raw_rgb_is_exact(self):
        """0.05 미만 점수는 항상 숨기고, raw_rgb 모드는 원본과 정확히 같다."""
        scores=np.array([[.01,.04,.1]],np.float32)
        rgb=np.full((3,9,3),60,np.uint8)
        overlay=render_lane_overlay(rgb,scores,0,'lane_bands_rgb')
        np.testing.assert_array_equal(overlay[:,:6],rgb[:,:6])
        np.testing.assert_array_equal(render_lane_overlay(rgb,None,.5,'raw_rgb'),rgb)

    def test_comparison_panels_share_the_same_rgb_and_score_frame(self):
        """비교(lane_compare) 화면의 세 패널은 같은 RGB·점수 프레임으로 만든 원본/밴드/밴드+RGB이다."""
        scores=np.linspace(0,1,320,dtype=np.float32)[None].repeat(180,axis=0)
        rgb=np.full((540,960,3),90,np.uint8)
        comparison=render_lane_overlay(rgb,scores,.05,'lane_compare')
        np.testing.assert_array_equal(comparison[180:360,:320],rgb[::3,::3])
        np.testing.assert_array_equal(comparison[180:360,320:640],
            render_lane_overlay(rgb,scores,.05,'lane_bands')[::3,::3])
        np.testing.assert_array_equal(comparison[180:360,640:],
            render_lane_overlay(rgb,scores,.05,'lane_bands_rgb')[::3,::3])


if __name__=='__main__':unittest.main()
