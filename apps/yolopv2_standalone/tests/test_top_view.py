"""Spatial alignment and score-preservation checks for the display projection."""
import unittest
import numpy as np
from lane_overlay import TopViewRenderer, BAND_COLORS
from lane_boundaries import projection_for, BoundaryConfig


class TopViewTests(unittest.TestCase):
    def test_normal_top_colors_scores_above_same_threshold_differently(self):
        renderer = TopViewRenderer()
        rgb = np.full((180, 320, 3), 80, np.uint8)
        lane = np.full((180, 320), 255, np.uint8)
        projection = projection_for(lane.shape)
        scores = np.full(lane.shape, .60, np.float32)
        first = renderer.render(rgb, lane, projection, 'normal', .5, scores)
        scores[:] = .65
        second = renderer.render(rgb, lane, projection, 'normal', .5, scores)
        self.assertFalse(np.array_equal(first[400,320], second[400,320]))
        scores[:] = .4
        excluded = renderer.render(rgb, lane, projection, 'normal', .5, scores)
        np.testing.assert_array_equal(excluded[400,320], (80,80,80))
        unavailable = renderer.render(rgb, lane, projection, 'normal', .5)
        np.testing.assert_array_equal(unavailable[400,320], (80,80,80))

    def test_rgb_and_score_land_on_same_road_anchor(self):
        rgb = np.zeros((180, 320, 3), np.uint8)
        rgb[170:179, 60:68] = (30, 80, 170)
        scores = np.zeros((180, 320), np.float32)
        scores[170:179, 60:68] = .99
        lane = (scores > .5).astype(np.uint8)*255
        renderer = TopViewRenderer()
        projection = projection_for(lane.shape, BoundaryConfig(near_left_u=.2, near_right_u=.8))
        raw = renderer.render(rgb, lane, projection, 'raw_rgb')
        np.testing.assert_array_equal(raw[479, 80], (30, 80, 170))
        mask = renderer.render(rgb, lane, projection, 'lane_bands', .05, scores)
        np.testing.assert_array_equal(mask[479, 80], BAND_COLORS[-1])
        self.assertEqual(mask.shape, (480, 640, 3))
        self.assertFalse(mask[479, 320].any())

    def test_source_pixels_and_scores_are_not_modified_and_cache_invalidates(self):
        renderer = TopViewRenderer()
        rgb = np.full((180, 320, 3), 80, np.uint8)
        lane = np.zeros((180, 320), np.uint8)
        scores = np.full(lane.shape, .8, np.float32)
        projection = projection_for(lane.shape)
        originals = [a.copy() for a in (rgb, lane, scores, projection)]
        renderer.render(rgb, lane, projection, 'lane_score', .5, scores)
        previous = renderer.rx
        renderer.render(rgb, lane, projection, 'raw_rgb')
        self.assertIs(renderer.rx, previous)
        for original, current in zip(originals, (rgb, lane, scores, projection)):
            np.testing.assert_array_equal(original, current)
        renderer.render(np.repeat(rgb, 2, axis=1), lane, projection, 'raw_rgb')
        self.assertIsNot(renderer.rx, previous)

    def test_outside_camera_is_black_not_wrapped(self):
        renderer = TopViewRenderer()
        rgb = np.full((180, 320, 3), 127, np.uint8)
        lane = np.full((180, 320), 255, np.uint8)
        projection = np.array([[1., 0, -100], [0, .2, 0], [0, 0, 1]])
        top = renderer.render(rgb, lane, projection)
        self.assertFalse(top[~renderer.valid].any())

    def test_near_final_row_and_unseen_ground_stay_black_in_all_modes(self):
        from dataclasses import replace
        config=replace(BoundaryConfig(),near_left_u=.02,near_right_u=.98,near_v=1.,
                       far_left_u=.43,far_right_u=.59,far_v=.62)
        lane=np.full((180,320),255,np.uint8)
        rgb=np.full((360,640,3),127,np.uint8)
        scores=np.full(lane.shape,.99,np.float32)
        rgb[-2:]=(10,20,30)
        renderer=TopViewRenderer(419)
        for mode in ('raw_rgb','normal','lane_bands','lane_bands_rgb','lane_score','lane_mask'):
            top=renderer.render(rgb,lane,projection_for(lane.shape,config),mode,.5,scores)
            self.assertFalse(top[420:].any(),mode)
            self.assertFalse(top[~renderer.valid].any(),mode)
        raw=renderer.render(rgb,lane,projection_for(lane.shape,config),'raw_rgb')
        np.testing.assert_array_equal(raw[419,320],(10,20,30))

    def test_invalid_near_row_rejected(self):
        for row in (0,480):
            with self.assertRaises(ValueError):TopViewRenderer(row)

    def test_group_overlay_uses_same_map_and_preserves_unseen_ground(self):
        from lane_contours import extract_lane_contours
        lane=np.zeros((180,320),np.uint8)
        lane[90:180,60:66]=255
        rgb=np.full((180,320,3),50,np.uint8)
        renderer=TopViewRenderer(419)
        projection=projection_for(lane.shape,BoundaryConfig(near_left_u=.2,near_right_u=.8))
        top=renderer.render(rgb,lane,projection,'raw_rgb')
        result=extract_lane_contours(lane)
        original=lane.copy()
        before=top.copy()
        renderer.overlay_lane_groups(top,result)
        changed=np.any(top!=before,axis=2)
        self.assertTrue(changed.any())
        # Vector rasterization and nearest RGB sampling can differ by one
        # source pixel at the projected contour edge.
        self.assertTrue(np.all(renderer.mx[changed]>=59))
        self.assertTrue(np.all(renderer.mx[changed]<=66))
        self.assertTrue(np.all(renderer.my[changed]>=89))
        self.assertFalse(top[~renderer.valid].any())
        self.assertFalse(top[420:].any())
        np.testing.assert_array_equal(lane,original)

    def test_group_overlay_empty_and_dimension_validation(self):
        from lane_contours import extract_lane_contours
        lane=np.zeros((180,320),np.uint8)
        result=extract_lane_contours(lane)
        renderer=TopViewRenderer(419)
        with self.assertRaises(ValueError):
            renderer.overlay_lane_groups(np.zeros((480,640,3),np.uint8),result)
        top=renderer.render(np.full((180,320,3),50,np.uint8),lane,projection_for(lane.shape),'raw_rgb')
        before=top.copy()
        renderer.overlay_lane_groups(top,result)
        np.testing.assert_array_equal(top,before)
        with self.assertRaises(ValueError):
            renderer.overlay_lane_groups(top,extract_lane_contours(np.zeros((90,160),np.uint8)))

    def test_only_raw_skeleton_is_drawn_not_group_interpolation(self):
        from lane_overlay import contour_skeleton_layer
        from lane_contours import extract_lane_contours
        lane=np.zeros((180,320),np.uint8)
        lane[80:100,80:85]=1
        lane[115:140,80:85]=1
        result=extract_lane_contours(lane)
        self.assertTrue(result.links)
        hidden=contour_skeleton_layer(result)
        self.assertFalse(np.any(np.all(hidden==(255,255,0),axis=2)))
        layer=contour_skeleton_layer(result,show_skeleton=True)
        self.assertFalse(layer[100:115].any())
        cyan=np.all(layer==(255,255,0),axis=2)
        np.testing.assert_array_equal(cyan,result.skeleton_mask>0)
        self.assertTrue(np.all(lane[cyan]>0))


if __name__ == '__main__':
    unittest.main()
