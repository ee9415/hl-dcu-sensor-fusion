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
        expected=t._warp(a.image_left,b.motion_matrix)
        common=np.isfinite(expected[:,0])&np.isfinite(b.image_left[:,0])
        self.assertLess(np.mean(abs(expected[common,0]-b.image_left[common,0])),.5)

    def test_long_occlusion_preserves_pair_and_damps_drift(self):
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
        t=ImageBoundaryTracker()
        u=(t.rows-125)/50
        points=np.column_stack((230+35*u-53.2891*u*u,t.rows))
        points[t.rows<120,0]=np.nan
        completed=t._complete(points)
        self.assertTrue(np.isnan(completed[t.rows<120,0]).all())
        np.testing.assert_allclose(completed[t.rows>=120],points[t.rows>=120])

    def test_crossed_candidates_cannot_poison_valid_history(self):
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

    def test_partial_observation_leaves_unseen_rows_unchanged(self):
        t=ImageBoundaryTracker();a=self.update(t,self.scene(),0)
        prior=[a.image_left.copy(),a.image_right.copy()]
        measured=prior[0].copy();measured[:,0]+=5
        measured[:20,0]=np.nan
        b=t._fuse([(measured,.9),None],prior,np.array([[1,0,0],[0,1,0]]),0,.1)
        np.testing.assert_allclose(b.image_left[:20],prior[0][:20],equal_nan=True)
        np.testing.assert_allclose(b.image_right,prior[1],equal_nan=True)
        self.assertEqual(b.reference_side,'left')

    def test_single_initial_side_supplies_low_confidence_pair(self):
        t=ImageBoundaryTracker();b=self.update(t,self.scene(right=False),0)
        self.assertTrue(b.pair_valid)
        self.assertEqual(b.right_state,'inferred')
        self.assertLess(b.right_confidence,b.left_confidence)
        self.assertIsNone(b.telemetry()['corridor_width_m'])

if __name__=='__main__':unittest.main()
