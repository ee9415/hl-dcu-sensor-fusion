"""Temporal curvature evidence must cover the model, not just a short tip."""
import unittest
from types import SimpleNamespace
import numpy as np
from image_boundaries import ImageBoundaryTracker

def curves(bend=0.,start=82.,sides=(-1,1)):
    y=np.arange(start,176.)
    return SimpleNamespace(group_curves=[np.column_stack((160+s*.9*(y-65)+bend*((y-125)/50)**2,y,np.ones(len(y)))) for s in sides])

class CurvatureConfirmationTests(unittest.TestCase):
    def seed(self):
        t=ImageBoundaryTracker()
        for i in range(20):t.update_contours(curves(),(180,320),i*.05)
        return t

    def test_one_frame_bend_holds_exact_previous_model(self):
        t=self.seed();before=[v.copy() for v in t.history]
        e=t.update_contours(curves(10),(180,320),1.)
        self.assertEqual((e.left_state,e.right_state),('held','held'))
        for a,b in zip(before,t.history):np.testing.assert_array_equal(a,b)
        self.assertTrue(all(s['curvature_pending'] for s in e.detection_diagnostics))

    def test_consistent_long_observations_eventually_confirm_real_turn(self):
        t=self.seed();confirmed=False
        for i in range(12):
            e=t.update_contours(curves(10),(180,320),1.+i*.05)
            if i<6:self.assertTrue(all(s.get('curvature_pending') for s in e.detection_diagnostics))
            confirmed |= any(s.get('curvature_confirmed',False) for s in e.detection_diagnostics)
        self.assertTrue(confirmed)
        self.assertTrue(e.left_valid and e.right_valid)

    def test_narrow_support_cannot_confirm_extrapolated_bend_by_repetition(self):
        t=self.seed();before=t.history[1].copy()
        for i in range(30):
            e=t.update_contours(curves(25,start=140.,sides=(1,)),(180,320),1.+i*.05)
        np.testing.assert_array_equal(before,t.history[1])
        self.assertTrue(e.detection_diagnostics[1]['curvature_pending'])
        self.assertFalse(e.detection_diagnostics[1]['curvature_support_sufficient'])

    def test_alternating_bends_never_accumulate_confirmation(self):
        t=self.seed();before=[v.copy() for v in t.history]
        for i in range(20):t.update_contours(curves(10 if i%2 else -10),(180,320),1.+i*.05)
        for a,b in zip(before,t.history):np.testing.assert_array_equal(a,b)

    def test_full_bilateral_evidence_survives_modest_quality_difference(self):
        t=self.seed();r=curves();r.group_curves[0][:,0]+=1.5*np.sin(r.group_curves[0][:,1]/7.)
        for i in range(20):e=t.update_contours(r,(180,320),1.+i*.05)
        self.assertEqual((e.left_state,e.right_state),('observed','observed'))
        self.assertTrue(e.left_support.any() and e.right_support.any())
