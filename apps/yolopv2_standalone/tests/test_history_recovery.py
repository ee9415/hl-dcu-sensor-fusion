import unittest
import numpy as np
from image_boundaries import ImageBoundaryTracker


class HistoryRecoveryTests(unittest.TestCase):
    def pair(self,t,left=100,right=180,lo=120,hi=172):
        out=[]
        for x in (left,right):
            p=np.column_stack((np.full(48,float(x)),t.rows))
            p[(t.rows<lo)|(t.rows>hi),0]=np.nan
            out.append(p)
        return out
    def fuse(self,t,pair):
        obs=[(p,.8,np.isfinite(p[:,0])) for p in pair]
        return t._fuse(obs,t.history,np.array([[1.,0.,0.],[0.,1.,0.]]),0.,.1)
    def test_first_width_uses_measurement_not_default(self):
        t=ImageBoundaryTracker();p=self.pair(t)
        self.fuse(t,p)
        np.testing.assert_allclose(t.width[t.width_samples>0],80)
    def test_repeated_current_pair_unlocks_old_width(self):
        t=ImageBoundaryTracker();t.history=self.pair(t,70,210)
        t.width=np.full(48,200.);t.width_samples[:]=5;t.scores=[.8,.8]
        p=self.pair(t)
        self.fuse(t,p)
        np.testing.assert_equal(t.width,200.)
        for _ in range(19):e=self.fuse(t,p)
        self.assertFalse(e.restored_pair)
        self.assertEqual(e.right_state,'observed')
        self.assertTrue(e.pair_valid)
        self.assertLess(abs(e.image_right[10,0]-180),1)
    def test_disjoint_current_pair_is_adopted_after_confirmation(self):
        t=ImageBoundaryTracker();t.history=self.pair(t,80,210,150,172)
        p=self.pair(t,100,180,120,144)
        for _ in range(3):e=self.fuse(t,p)
        self.assertEqual(e.left_state,'observed')
        np.testing.assert_allclose(e.image_left[np.isfinite(e.image_left[:,0]),0],100)
    def test_crossing_and_narrow_pair_cannot_override_width(self):
        for l,r in ((190,180),(170,180)):
            t=ImageBoundaryTracker();t.width=np.full(48,200.);t.width_samples[:]=5
            p=self.pair(t,l,r);obs=[(v,.8,np.isfinite(v[:,0])) for v in p]
            for _ in range(5):t._confirm_current_pair(obs,[None,None])
            self.assertEqual(t.current_pair_hits,0)
            np.testing.assert_equal(t.width,200.)


if __name__=='__main__':unittest.main()
