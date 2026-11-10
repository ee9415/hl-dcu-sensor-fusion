"""ImageBoundaryTracker의 이동 중 복구 로직 테스트.

차량이 움직이는 동안 오래된(stale) 이전 경계선을 현재 관측 쌍으로 교체하는 조건을 확인한다.
"""
import unittest
import numpy as np
from image_boundaries import ImageBoundaryTracker


class MovingRecoveryTests(unittest.TestCase):
    def pair(self,t,shift=0):
        """중심 x=160에서 아래로 갈수록 벌어지는 좌우 경계선 쌍. shift만큼 x 방향 이동."""
        return [np.column_stack((160+s*.8*(t.rows-65)+shift,t.rows)) for s in (-1,1)]

    def test_moving_current_pair_replaces_stale_prior_without_stopping(self):
        """관측 이동량(6px/프레임)과 일치하게 움직이는 현재 쌍은 3회 확인 후 stale 이전값을 대체한다. 그 전까지는 이전값 유지."""
        t=ImageBoundaryTracker();t.observation_motion=np.array([[1.,0.,6.],[0.,1.,0.]])
        for i in range(3):
            pair=self.pair(t,6*i)
            stale=[p+np.array([15.,0.]) for p in pair]
            out=t._confirm_current_pair([(p,.8) for p in pair],stale)
            if i<2:np.testing.assert_allclose(out[0],stale[0])
        self.assertEqual(t.current_pair_hits,3)
        for p,q in zip(out,pair):np.testing.assert_allclose(p,q)

    def test_inconsistent_jump_cannot_replace_prior(self):
        """프레임마다 앞뒤로 튀는 관측은 확인 횟수를 채우지 못해 이전값을 대체할 수 없다."""
        t=ImageBoundaryTracker();t.observation_motion=np.array([[1.,0.,6.],[0.,1.,0.]])
        stale=self.pair(t)
        for shift in (0,15,-15,20):
            out=t._confirm_current_pair([(p,.8) for p in self.pair(t,shift)],stale)
        self.assertLess(t.current_pair_hits,3)
        np.testing.assert_allclose(out,stale)

    def test_partial_pair_never_rebases_stale_geometry(self):
        """좌우 신뢰도가 불균형한(한쪽만 약한) 쌍은 이전 기하를 재설정하지 못한다."""
        t=ImageBoundaryTracker();stale=self.pair(t)
        for _ in range(5):
            p=self.pair(t,15)
            out=t._confirm_current_pair([(p[0],.6),(p[1],.9)],stale)
        self.assertEqual(t.current_pair_hits,0)
        np.testing.assert_allclose(out,stale)

    def test_single_side_reacquires_moving_candidate(self):
        """한쪽 경계선만 있을 때도 이동량과 일관된 후보는 3회 확인 후 재획득된다."""
        t=ImageBoundaryTracker();t.observation_motion=np.array([[1.,0.,6.],[0.,1.,0.]])
        prior=self.pair(t)[0]
        for i in range(3):
            candidate=(self.pair(t,6*i)[0],.8)
            t._detect_full=lambda *args:candidate
            out=t._reacquire_observation(None,0,prior,None)
            if i<2:self.assertIsNone(out)
        self.assertIs(out,candidate)

    def test_confirmed_observed_span_survives_bad_unobserved_width_tail(self):
        """관측되지 않은 위쪽 구간의 잘못된 폭(200) 때문에 확인된 관측 쌍이 버려지지 않아야 한다."""
        t=ImageBoundaryTracker();old=self.pair(t)
        t.width=old[1][:,0]-old[0][:,0]
        # y<120 구간에만 비정상 폭을 심는다.
        t.width[t.rows<120]=200.;t.width_samples[:]=5
        measured=[p.copy() for p in old]
        # 측정값은 y<120 구간을 관측하지 않은 상태
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
