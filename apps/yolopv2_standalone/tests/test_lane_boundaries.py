import json
import unittest
import numpy as np
from lane_boundaries import (BoundaryConfig, BoundaryTracker, draw_boundaries,
    projection_for, render_boundary_bev, transform, white_lane_evidence)


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.forward=np.linspace(0,25,81)
        self.lateral=np.linspace(-6,6,241)
        self.tracker=BoundaryTracker()

    def masks(self,left=-5.,right=5.,markings=False):
        left=np.broadcast_to(left,(81,))
        right=np.broadcast_to(right,(81,))
        x=self.lateral[None,:]
        road=(x>=left[:,None]) & (x<=right[:,None])
        lane=np.zeros_like(road)
        if markings:
            lane[np.arange(81),np.argmin(abs(x-left[:,None]),axis=1)]=True
            lane[np.arange(81),np.argmin(abs(x-right[:,None]),axis=1)]=True
        return road,lane

    def update(self,masks,t=0):
        return self.tracker.update_bev(self.forward,self.lateral,*masks,t)

    def assert_geometry(self,result):
        c=self.tracker.config
        self.assertTrue(result.pair_valid)
        self.assertEqual(result.forward[0],0.)
        self.assertEqual(result.left[0],-c.vehicle_width_m/2-c.left_margin_m)
        self.assertEqual(result.right[0],c.vehicle_width_m/2+c.right_margin_m)
        for values,coeff in ((result.left,result.left_coefficients),(result.right,result.right_coefficients)):
            np.testing.assert_allclose(values,np.polynomial.polynomial.polyval(result.forward,coeff))
        # Independent dense nearest-distance measurement, excluding endpoints.
        dense=np.linspace(0,c.max_forward_m,20001)
        right=np.polynomial.polynomial.polyval(dense,result.right_coefficients)
        for x,y in zip(result.forward[8:-8:4],result.left[8:-8:4]):
            distance=np.min(np.hypot(dense-x,right-y))
            self.assertLessEqual(abs(distance-c.width_m),c.width_tolerance_m+.003)
        left=np.polynomial.polynomial.polyval(dense,result.left_coefficients)
        for x,y in zip(result.forward[8:-8:4],result.right[8:-8:4]):
            distance=np.min(np.hypot(dense-x,left-y))
            self.assertLessEqual(abs(distance-c.width_m),c.width_tolerance_m+.003)
        json.dumps(result.telemetry(c),allow_nan=False)

    def test_initial_empty_input_still_has_fixed_quadratic_pair(self):
        empty=np.zeros((81,241),bool)
        result=self.update((empty,empty))
        self.assert_geometry(result)
        self.assertEqual(result.state,'held')
        self.assertIsNone(result.corridor_feasible)
        self.assertIsNone(result.telemetry(self.tracker.config)['clearance_sufficient'])

    def test_settings_control_anchors_and_normal_width(self):
        self.tracker=BoundaryTracker(BoundaryConfig(vehicle_width_m=2.,left_margin_m=.2,right_margin_m=.6))
        result=self.update(self.masks())
        self.assert_geometry(result)
        self.assertAlmostEqual(result.right[0]-result.left[0],2.8)
        for kw in ({'vehicle_width_m':0},{'width_tolerance_m':2.},{'road_support_threshold':1.1},
                   {'max_forward_m':41},{'left_margin_m':-1}):
            with self.assertRaises(ValueError): BoundaryConfig(**kw)

    def test_road_widening_and_paint_never_expand_width(self):
        first=self.update(self.masks(-1.5,1.5,True))
        for n in range(1,8):
            result=self.update(self.masks(-5,5,True),n*.1)
            self.assert_geometry(result)
            np.testing.assert_allclose(result.left,first.left)
            np.testing.assert_allclose(result.right,first.right)

    def test_seek_wider_side_with_fixed_start_and_smooth_return(self):
        for sign in (-1,1):
            self.tracker=BoundaryTracker()
            shift=sign*.004*self.forward**2
            road,lane=self.masks(shift-1.5,shift+1.5)
            previous=self.update(self.masks())
            for n in range(1,61):
                result=self.update((road,lane),n*.1)
                self.assertLessEqual(max(np.max(abs(result.left-previous.left)),
                                         np.max(abs(result.right-previous.right))),.100001)
                self.assertEqual(result.left[0],previous.left[0])
                self.assertEqual(result.right[0],previous.right[0])
                previous=result
            self.assert_geometry(result)
            self.assertTrue(result.corridor_feasible)
            self.assertGreater(sign*result.left_coefficients[2],0)
            self.assertNotAlmostEqual(result.left_coefficients[2],result.right_coefficients[2],places=7)
            for n in range(61,121): result=self.update(self.masks(),n*.1)
            self.assert_geometry(result)

    def test_narrow_road_reports_blocked_without_shrinking(self):
        result=self.update(self.masks(-.6,.6))
        self.assert_geometry(result)
        self.assertFalse(result.corridor_feasible)
        self.assertEqual(result.state,'blocked')

    def test_missing_road_and_time_errors_do_not_erase_pair(self):
        first=self.update(self.masks())
        empty=np.zeros((81,241),bool)
        for timestamp in (60.,120.,float('nan'),119.,120.):
            result=self.update((empty,empty),timestamp)
            self.assert_geometry(result)
            np.testing.assert_allclose(result.left,first.left)
            np.testing.assert_allclose(result.right,first.right)

    def test_all_search_candidates_meet_normal_width_tolerance(self):
        self.assertGreater(len(self.tracker.candidates),5)
        for (value,heading),geometry in self.tracker.candidates:
            self.tracker.curvature=value
            self.tracker.heading=heading
            result=self.update((np.zeros((81,241),bool),np.zeros((81,241),bool)),value+10)
            self.assert_geometry(result)

    def test_one_painted_edge_does_not_hide_virtual_opposite(self):
        road,lane=self.masks(-2,2,True)
        lane[:,self.lateral>0]=False
        self.assert_geometry(self.update((road,lane)))

    def test_width_reduction_changes_image_width_without_rescaling_camera(self):
        from lane_boundaries import boundary_points
        wide=BoundaryConfig()
        narrow=BoundaryConfig(left_margin_m=.15,right_margin_m=.15)
        np.testing.assert_allclose(projection_for((180,320),wide),projection_for((180,320),narrow))
        widths=[]
        empty=np.zeros((81,241),bool)
        for c in (wide,narrow):
            r=BoundaryTracker(c).update_bev(self.forward,self.lateral,empty,empty,0)
            pixels=[transform(boundary_points(r,side)[:1],projection_for((180,320),c))[0,0]
                    for side in ('left','right')]
            widths.append(pixels[1]-pixels[0])
        self.assertAlmostEqual(widths[1]/widths[0],2.1/2.4)

    def test_paint_disappears_and_returns_without_jump_at_narrow_width(self):
        self.tracker=BoundaryTracker(BoundaryConfig.load())
        target=self.tracker._geometry(.004,-.1)
        road,lane=self.masks()
        for edge in target[:2]:
            lane[np.arange(81),np.argmin(abs(self.lateral[None,:]-edge[:,None]),axis=1)]=True
        previous=None
        for n in range(100):
            visible=n<30 or n>=60
            result=self.update((road,lane if visible else np.zeros_like(lane)),n*.1)
            self.assert_geometry(result)
            if previous is not None:
                self.assertLessEqual(max(np.max(abs(result.left-previous.left)),
                                         np.max(abs(result.right-previous.right))),.100001)
            previous=result

    def test_coarse_search_local_tracking_and_periodic_reacquisition(self):
        self.assertLess(len(self.tracker.candidates),60)
        self.update(self.masks(),0)
        full=self.tracker.last_candidate_count
        self.update(self.masks(),.1)
        self.assertLess(self.tracker.last_candidate_count,full)
        for n in range(2,5): self.update(self.masks(),n*.1)
        self.assertEqual(self.tracker.last_candidate_count,full)
        self.update(self.masks(-.5,.5),.5)
        self.update(self.masks(),.6)
        self.assertEqual(self.tracker.last_candidate_count,full)

    def test_batched_stroke_matches_original_raster(self):
        from lane_boundaries import draw_polyline
        points=np.array([[-5.,3.],[15.5,27.2],[40.,10.],[85.,70.]])
        for dashed in (False,True):
            expected=np.zeros((60,70,3),np.uint8)
            actual=expected.copy()
            for i,(start,end) in enumerate(zip(points[:-1],points[1:])):
                if dashed and i%6>=3: continue
                count=min(2000,int(np.max(np.abs(end-start)))+2)
                x,y=np.rint(np.linspace(start,end,count)).astype(int).T
                for dy in range(-3,4):
                    for dx in range(-3,4):
                        xx,yy=x+dx,y+dy
                        valid=(xx>=0)&(xx<70)&(yy>=0)&(yy<60)
                        expected[yy[valid],xx[valid]]=(0,220,255)
            draw_polyline(actual,points,(0,220,255),3,dashed)
            np.testing.assert_array_equal(actual,expected)

    def test_heading_follows_paint_in_broad_road_without_moving_anchors(self):
        target=self.tracker._geometry(-.002,.15)
        road,lane=self.masks()
        for edge in target[:2]:
            lane[np.arange(81),np.argmin(abs(self.lateral[None,:]-edge[:,None]),axis=1)]=True
        first=self.update((road,lane))
        for n in range(1,45): result=self.update((road,lane),n*.1)
        self.assert_geometry(result)
        self.assertGreater(result.left_coefficients[1],.05)
        self.assertLess(np.mean(abs(result.left-target[0])),np.mean(abs(first.left-target[0])))

    def test_fixed_image_anchors_and_continuous_lines_when_blocked(self):
        from lane_boundaries import boundary_points
        result=self.update(self.masks(-.6,.6))
        h,w=180,320
        projection=projection_for((h,w))
        for side,u in (('left',.12),('right',.88)):
            points=boundary_points(result,side)
            actual=transform(points[:1],projection)[0]
            np.testing.assert_allclose(actual,[u*(w-1),.98*(h-1)],atol=1e-7)
        image=render_boundary_bev(result)
        for side,color in (('left',(0,220,255)),('right',(255,180,0))):
            points=np.rint(boundary_points(result,side)).astype(int)
            self.assertTrue(np.all(image[points[:,1],points[:,0]]==color))

    def test_unobserved_forward_range_is_not_reported_feasible(self):
        road,lane=self.masks()
        result=self.tracker.update_bev(self.forward[:30],self.lateral,road[:30],lane[:30],0)
        self.assert_geometry(result)
        self.assertFalse(result.corridor_feasible)

    def test_mask_projection_render_and_json(self):
        h, w = 180, 320
        projection = projection_for((h, w))
        source = np.array([[.12*(w-1), .98*(h-1)], [.54*(w-1), .60*(h-1)]])
        np.testing.assert_allclose(transform(source, np.linalg.inv(projection)), [[.4*639,479],[.6*639,0]], atol=1e-7)
        road = np.full((h, w), 255, np.uint8)
        lane = np.zeros_like(road)
        for side in (-2.5,3.):
            points = transform(np.column_stack(((np.full(len(self.forward),side)/12+.5)*639,
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


if __name__ == '__main__':
    unittest.main()
