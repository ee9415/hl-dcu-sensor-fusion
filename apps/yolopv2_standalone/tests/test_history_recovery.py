"""ImageBoundaryTracker의 이력(history)·차로 폭 복구 로직 테스트."""
import unittest
import numpy as np
from image_boundaries import ImageBoundaryTracker


class HistoryRecoveryTests(unittest.TestCase):
    def pair(self,t,left=100,right=180,lo=120,hi=172):
        """x=left/right 세로 경계선 한 쌍을 만든다. lo~hi 행 밖은 NaN(미관측)."""
        out=[]
        for x in (left,right):
            p=np.column_stack((np.full(48,float(x)),t.rows))
            p[(t.rows<lo)|(t.rows>hi),0]=np.nan
            out.append(p)
        return out
    def fuse(self,t,pair):
        """관측 쌍을 고정 신뢰도 0.8, 항등 변환으로 추적기에 융합한다."""
        obs=[(p,.8,np.isfinite(p[:,0])) for p in pair]
        return t._fuse(obs,t.history,np.array([[1.,0.,0.],[0.,1.,0.]]),0.,.1)
    def test_first_width_uses_measurement_not_default(self):
        """첫 차로 폭은 기본값이 아닌 실제 측정값(80px)으로 설정된다."""
        t=ImageBoundaryTracker();p=self.pair(t)
        self.fuse(t,p)
        np.testing.assert_allclose(t.width[t.width_samples>0],80)
    def test_repeated_current_pair_unlocks_old_width(self):
        """기존 폭(200)과 다른 현재 쌍이 한 번 들어오면 폭을 바꾸지 않지만, 반복 확인되면 현재 쌍을 관측으로 채택한다."""
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
        """이력과 겹치지 않는 현재 쌍도 몇 프레임 확인 후 채택된다."""
        t=ImageBoundaryTracker();t.history=self.pair(t,80,210,150,172)
        p=self.pair(t,100,180,120,144)
        for _ in range(3):e=self.fuse(t,p)
        self.assertEqual(e.left_state,'observed')
        np.testing.assert_allclose(e.image_left[np.isfinite(e.image_left[:,0]),0],100)
    def test_crossing_and_narrow_pair_cannot_override_width(self):
        """좌우가 교차하거나 너무 좁은 쌍은 확인 카운트를 올리지 못하고 폭도 바꾸지 못한다."""
        for l,r in ((190,180),(170,180)):
            t=ImageBoundaryTracker();t.width=np.full(48,200.);t.width_samples[:]=5
            p=self.pair(t,l,r);obs=[(v,.8,np.isfinite(v[:,0])) for v in p]
            for _ in range(5):t._confirm_current_pair(obs,[None,None])
            self.assertEqual(t.current_pair_hits,0)
            np.testing.assert_equal(t.width,200.)


if __name__=='__main__':unittest.main()
