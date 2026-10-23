import unittest
import numpy as np
from image_boundaries import ImageBoundaryTracker


class MovingRecoveryTests(unittest.TestCase):
    def pair(self,t,shift=0):
        return [np.column_stack((160+s*.8*(t.rows-65)+shift,t.rows)) for s in (-1,1)]

    def test_moving_current_pair_replaces_stale_prior_without_stopping(self):
        t=ImageBoundaryTracker();t.observation_motion=np.array([[1.,0.,6.],[0.,1.,0.]])
        for i in range(3):
            pair=self.pair(t,6*i)
            stale=[p+np.array([15.,0.]) for p in pair]
            out=t._confirm_current_pair([(p,.8) for p in pair],stale)
            if i<2:np.testing.assert_allclose(out[0],stale[0])
        self.assertEqual(t.current_pair_hits,3)
        for p,q in zip(out,pair):np.testing.assert_allclose(p,q)

    def test_inconsistent_jump_cannot_replace_prior(self):
        t=ImageBoundaryTracker();t.observation_motion=np.array([[1.,0.,6.],[0.,1.,0.]])
        stale=self.pair(t)
        for shift in (0,15,-15,20):
            out=t._confirm_current_pair([(p,.8) for p in self.pair(t,shift)],stale)
        self.assertLess(t.current_pair_hits,3)
        np.testing.assert_allclose(out,stale)

    def test_partial_pair_never_rebases_stale_geometry(self):
        t=ImageBoundaryTracker();stale=self.pair(t)
        for _ in range(5):
            p=self.pair(t,15)
            out=t._confirm_current_pair([(p[0],.6),(p[1],.9)],stale)
        self.assertEqual(t.current_pair_hits,0)
        np.testing.assert_allclose(out,stale)

    def test_single_side_reacquires_moving_candidate(self):
        t=ImageBoundaryTracker();t.observation_motion=np.array([[1.,0.,6.],[0.,1.,0.]])
        prior=self.pair(t)[0]
        for i in range(3):
            candidate=(self.pair(t,6*i)[0],.8)
            t._detect_full=lambda *args:candidate
            out=t._reacquire_observation(None,0,prior,None)
            if i<2:self.assertIsNone(out)
        self.assertIs(out,candidate)

    def test_confirmed_observed_span_survives_bad_unobserved_width_tail(self):
        t=ImageBoundaryTracker();old=self.pair(t)
        t.width=old[1][:,0]-old[0][:,0]
        t.width[t.rows<120]=200.;t.width_samples[:]=5
        measured=[p.copy() for p in old]
        for p in measured:p[t.rows<120,0]=np.nan
        for _ in range(3):
            e=t._fuse([(p,.8) for p in measured],old,
                      np.array([[1.,0.,0.],[0.,1.,0.]]),0.,.1)
        self.assertTrue(e.pair_valid)
        self.assertFalse(e.restored_pair)
        self.assertEqual(e.left_state,'observed')
        self.assertTrue(np.isnan(e.image_left[t.rows<120,0]).all())
        self.assertIn('confirmed_pair_replaced_invalid_fusion',t.detection_diagnostics[0])


if __name__=='__main__':unittest.main()
