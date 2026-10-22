import unittest
from types import SimpleNamespace
import numpy as np
from image_boundaries import ImageBoundaryTracker,road_motion

class ImageBoundaryTests(unittest.TestCase):
    def scene(self,shift=0,curve=0,left=True,right=True):
        image=np.full((180,320,3),60,np.uint8)
        for y in range(78,176):
            for visible,x in ((left,160-(y-65)*.9),(right,160+(y-65)*.9)):
                if visible:
                    x=int(x+shift+curve*((y-125)/50)**2)
                    image[y,max(0,x-1):min(320,x+2)]=230
        return image
    def update(self,t,image,n):
        return t.update(np.ones((180,320),np.uint8),np.zeros((180,320),np.uint8),n*.1,rgb=image)
    def test_actual_paint_free_anchors_and_width(self):
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
        rng=np.random.default_rng(4)
        image=rng.uniform(0,255,(90,160)).astype(np.float32)
        shifted=np.roll(np.roll(image,2,axis=0),3,axis=1)
        m,score=road_motion(image,shifted,np.ones_like(image,bool))
        self.assertGreater(score,.7)
        np.testing.assert_allclose(m,[[1,0,6],[0,1,4]],atol=.01)
        m,score=road_motion(np.zeros_like(image),np.zeros_like(image),np.ones_like(image,bool))
        self.assertEqual(score,0)
    def test_motion_warp_and_timestamp_reset(self):
        t=ImageBoundaryTracker()
        a=self.update(t,self.scene(),0)
        p=t._warp(a.image_left,np.array([[1,0,6],[0,1,0]]))
        good=np.isfinite(p[:,0])&np.isfinite(a.image_left[:,0])
        np.testing.assert_allclose(p[good,0]-a.image_left[good,0],6)
        b=self.update(t,np.full((180,320,3),60,np.uint8),-1)
        self.assertFalse(b.valid)
    def test_object_mask_prevents_vehicle_paint(self):
        t=ImageBoundaryTracker();image=self.scene()
        b=t.update(np.ones((180,320),np.uint8),np.zeros((180,320),np.uint8),0,rgb=image,
                   detections=[SimpleNamespace(xmin=0,ymin=0,xmax=1,ymax=1)])
        self.assertFalse(b.valid)

    def test_dark_seams_and_offroad_paint_are_rejected(self):
        image=self.scene();image[image==230]=105
        self.assertFalse(self.update(ImageBoundaryTracker(),image,0).valid)
        image=self.scene()
        t=ImageBoundaryTracker()
        b=t.update(np.zeros((180,320),np.uint8),np.zeros((180,320),np.uint8),0,rgb=image)
        self.assertFalse(b.valid)

    def test_render_does_not_change_tracking_coordinates(self):
        from lane_boundaries import draw_boundaries,render_boundary_bev
        import json
        b=self.update(ImageBoundaryTracker(),self.scene(),0)
        original=b.image_left.copy()
        draw_boundaries(np.zeros((540,960,3),np.uint8),b,(320,180))
        render_boundary_bev(b)
        np.testing.assert_array_equal(original,b.image_left)
        json.dumps(b.telemetry())

    def test_partial_reacquisition_cannot_amplify_correction(self):
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
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        texture=np.random.default_rng(8).uniform(25,100,(90,160)).astype(np.float32)
        t.previous=texture
        moved=np.roll(np.roll(texture,2,axis=0),3,axis=1)
        image=np.repeat(np.repeat(moved,2,axis=0),2,axis=1)
        image=np.repeat(image[:,:,None],3,axis=2).astype(np.uint8)
        b=self.update(t,image,1)
        self.assertEqual(b.left_state,'predicted')
        self.assertGreater(b.motion_confidence,.7)
        expected=t._warp(a.image_left,np.array([[1,0,6],[0,1,4]]))
        common=np.isfinite(expected[:,0])&np.isfinite(b.image_left[:,0])
        self.assertLess(np.mean(abs(expected[common,0]-b.image_left[common,0])),.5)

    def test_long_occlusion_damps_only_unmeasured_motion(self):
        from unittest.mock import patch
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        blank=np.full((180,320,3),60,np.uint8)
        flow=np.array([[1,0,1.5],[0,1,2.]])
        with patch('image_boundaries.road_motion',return_value=(flow,.9)):
            initial=self.update(t,blank,1)
        with patch('image_boundaries.road_motion',return_value=(np.eye(2,3),0.)):
            for i in range(2,301):
                b=self.update(t,blank,i)
                self.assertTrue(b.pair_valid)
                self.assertTrue(np.isfinite(b.image_left).all())
                self.assertTrue(np.isfinite(b.image_right).all())
                if i==200:late=b.image_left.copy()
        self.assertLess(np.max(abs(b.image_left-late)),.05)
        self.assertLess(b.left_confidence,.01)
        for i in range(301,311):
            recovered=self.update(t,self.scene(),i)
        self.assertTrue(recovered.pair_valid)
        self.assertIn(recovered.left_state,('observed','aligning'))

    def test_valid_motion_is_not_attenuated_during_missing_paint(self):
        from unittest.mock import patch
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        blank=np.full((180,320,3),60,np.uint8)
        flow=np.array([[1,0,.1],[0,1,0.]])
        with patch('image_boundaries.road_motion',return_value=(flow,.9)):
            for i in range(1,101): b=self.update(t,blank,i)
        np.testing.assert_allclose(b.image_left[:,0],a.image_left[:,0]+10,atol=.02)
        self.assertLess(b.left_confidence,.06)

    def test_remote_candidate_requires_persistence_and_reports_alignment_error(self):
        from unittest.mock import patch
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        measured=a.image_left.copy();measured[:,0]+=30
        with patch('image_boundaries.road_motion',return_value=(np.eye(2,3),1.)), \
             patch.object(t,'_detect',side_effect=[(measured,.95),None]*3):
            b=self.update(t,self.scene(),1)
            np.testing.assert_allclose(b.image_left,a.image_left)
            b=self.update(t,self.scene(),2)
            np.testing.assert_allclose(b.image_left,a.image_left)
            b=self.update(t,self.scene(),3)
        self.assertEqual(b.left_state,'aligning')
        self.assertAlmostEqual(b.left_residual_px,22.,places=3)
        self.assertLess(b.left_confidence,.03)
        self.assertLessEqual(np.max(abs(b.image_left[:,0]-a.image_left[:,0])),8.01)

    def test_observed_curve_follows_paint_and_reports_residual(self):
        t=ImageBoundaryTracker();b=self.update(t,self.scene(curve=8),0)
        self.assertTrue(b.pair_valid)
        for points,support,sign,residual,state in (
                (b.image_left,b.left_support,-1,b.left_residual_px,b.left_state),
                (b.image_right,b.right_support,1,b.right_residual_px,b.right_state)):
            y=points[support,1]
            truth=160+sign*(y-65)*.9+8*((y-125)/50)**2
            self.assertLess(np.mean(abs(points[support,0]-truth)),2)
            self.assertEqual(state,'observed')
            self.assertLessEqual(residual,4)

    def test_endpoint_extension_is_linear_not_quadratic(self):
        t=ImageBoundaryTracker()
        points=np.column_stack((80+.8*(t.rows-140)+.08*(t.rows-140)**2,t.rows))
        points[(t.rows<130)|(t.rows>156),0]=np.nan
        p=t._complete(points)
        far=p[t.rows<120,0]
        self.assertLess(np.max(abs(np.diff(far,2))),1e-6)

    def test_repeated_affine_expansion_stays_bounded_and_can_recover(self):
        from unittest.mock import patch
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        blank=np.full((180,320,3),60,np.uint8)
        flow=np.array([[1.05,0,-8],[0,1.05,-5.]])
        with patch('image_boundaries.road_motion',return_value=(flow,.9)):
            for i in range(1,101): self.update(t,blank,i)
        for p in t.history:
            self.assertLessEqual(np.max(p[:,0]),352)
            self.assertGreaterEqual(np.min(p[:,0]),-32)
        with patch('image_boundaries.road_motion',return_value=(np.eye(2,3),1.)):
            for i in range(101,151): b=self.update(t,self.scene(),i)
        self.assertEqual(b.left_state,'observed')
        self.assertEqual(b.right_state,'observed')
        self.assertTrue(b.pair_valid)

    def test_opposite_measurement_does_not_drag_owned_prediction(self):
        from unittest.mock import patch
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        flow=np.array([[1,0,3],[0,1,4.]])
        expected=t._warp(a.image_right,flow)
        left=t._warp(a.image_left,flow);left[:,0]+=5
        with patch('image_boundaries.road_motion',return_value=(flow,.9)), \
             patch.object(t,'_detect',side_effect=[(left,.95),None]):
            b=self.update(t,self.scene(),1)
        np.testing.assert_allclose(b.image_right,expected)

    def test_single_initial_side_supplies_low_confidence_pair(self):
        t=ImageBoundaryTracker();b=self.update(t,self.scene(right=False),0)
        self.assertTrue(b.pair_valid)
        self.assertEqual(b.right_state,'inferred')
        self.assertLess(b.right_confidence,b.left_confidence)
        self.assertIsNone(b.telemetry()['corridor_width_m'])

    def test_missing_left_does_not_acquire_right_paint(self):
        from unittest.mock import patch
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        t.history[0]=t.history[1].copy();t.history[0][:,0]-=3
        t.scores[0]=.05;t.ages[0]=2.
        with patch('image_boundaries.road_motion',return_value=(np.eye(2,3),1.)):
            b=self.update(t,self.scene(left=False),1)
        self.assertEqual(b.left_state,'inferred')
        self.assertLessEqual(b.left_confidence,.09)
        self.assertEqual(b.right_state,'observed')
        self.assertGreater(np.median(b.image_right[:,0]-b.image_left[:,0]),3)

    def test_long_missing_pair_is_held_when_local_warp_leaves_view(self):
        from unittest.mock import patch
        t=ImageBoundaryTracker();self.update(t,self.scene(),0)
        blank=np.full((180,320,3),60,np.uint8)
        flow=np.array([[1.05,0,-8],[0,1.05,-5.]])
        held=0
        with patch('image_boundaries.road_motion',return_value=(flow,.9)):
            for i in range(1,101):
                b=self.update(t,blank,i)
                self.assertTrue(b.pair_valid)
                for p in (b.image_left,b.image_right):
                    self.assertGreaterEqual(np.count_nonzero((p[:,0]>=0)&(p[:,0]<=319)),8)
                held+=b.left_state=='held'
        self.assertGreater(held,0)
        self.assertLess(b.left_confidence,.01)

if __name__=='__main__':unittest.main()
