"""ImageBoundaryTracker의 직선/곡선 차선 피팅 안정성 테스트."""
import unittest
from pathlib import Path
import numpy as np
from image_boundaries import ImageBoundaryTracker


class StraightStripeTests(unittest.TestCase):
    def test_captured_straight_stays_straight_even_with_curved_prior(self):
        """실측 직선 차선 데이터는 이전값이 곡선이어도 직선(2차 계수 거의 0)으로 피팅되고 x 위치도 맞는다."""
        z=np.load(Path(__file__).parent/'data/left_straight.npz')
        for prior in (None,z['prior']):
            t=ImageBoundaryTracker();t.nn_rank_mask=z['lane']>0
            out=t._detect(z['paint'],0,prior)
            self.assertIsNotNone(out)
            p=out[0];p=p[np.isfinite(p[:,0])]
            coef=np.polynomial.polynomial.polyfit(p[:,1],p[:,0],2)
            self.assertLess(abs(coef[2]),.0003)
            self.assertLess(abs(np.polynomial.polynomial.polyval(150,coef)-68),2)

    def test_true_curve_survives_stripe_center_and_line_selection(self):
        """실제 곡선 차선은 2차 계수(0.005)가 보존된다."""
        t=ImageBoundaryTracker();paint=np.zeros((180,320),bool)
        for y in range(78,175):
            x=round(155-.9*(y-78)+.005*(y-125)**2)
            paint[y,x-2:x+3]=True
        out=t._detect_full(paint,0,None)
        self.assertIsNotNone(out)
        p=out[0];p=p[np.isfinite(p[:,0])]
        coef=np.polynomial.polynomial.polyfit(p[:,1],p[:,0],2)
        self.assertAlmostEqual(coef[2],.005,delta=.001)

    def test_variable_width_and_transverse_mark_do_not_bend_line(self):
        """선 두께가 들쭉날쭉하거나 가로 표시가 겹쳐도 직선이 휘지 않는다."""
        t=ImageBoundaryTracker();paint=np.zeros((180,320),bool)
        for y in range(78,175):
            x=230-y;width=1+(y//12)%4
            paint[y,x-width:x+width+1]=True
        # 가로로 지나가는 노면 표시
        paint[124:128,50:280]=True
        out=t._detect_full(paint,0,None)
        self.assertIsNotNone(out)
        p=out[0];p=p[np.isfinite(p[:,0])]
        self.assertLess(np.max(abs(p[:,0]-(230-p[:,1]))),1)


if __name__=='__main__':unittest.main()
