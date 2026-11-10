"""ImageBoundaryTracker의 위쪽(원거리) 구간 재피팅 테스트. 실측 데이터 right_upper_gap.npz 사용."""
import unittest
from pathlib import Path
import numpy as np
from image_boundaries import ImageBoundaryTracker


class UpperGapTests(unittest.TestCase):
    def load(self):
        """실측 데이터를 불러와 NN 마스크를 설정한 추적기와 (paint, prior)를 반환한다."""
        z=np.load(Path(__file__).parent/'data/right_upper_gap.npz')
        t=ImageBoundaryTracker();t.nn_rank_mask=z['lane']>0
        return t,z['paint'],z['prior']
    def test_captured_upper_paint_wins_with_and_without_stale_history(self):
        """이전 이력 유무와 관계없이 위쪽 실제 페인트(y=100에서 x≈179)를 따라가고 형태도 매끄럽다."""
        for history in (True,False):
            t,paint,prior=self.load()
            o=t._detect(paint,1,prior if history else None)
            at100=o[0][np.argmin(abs(t.rows-100)),0]
            self.assertLess(abs(at100-179),3)
            self.assertTrue(t._smooth_shape(o[0]))
    def test_upper_replacement_requires_actual_pixels(self):
        """위쪽 구간에 실제 페인트 픽셀이 없으면 재피팅하지 않는다."""
        t,paint,prior=self.load()
        regional=t._detect_regions(paint,1,prior);full=t._detect_full(paint,1,prior)
        missing=paint.copy();missing[:124]=False
        self.assertIsNone(t._regional_upper_refit(missing,regional,full))
    def test_separate_near_line_cannot_be_joined_to_upper(self):
        """근거리 쪽이 다른 선(15px 어긋남)이면 위쪽 선과 이어 붙이지 않는다."""
        t,paint,prior=self.load()
        regional=t._detect_regions(paint,1,prior);full=t._detect_full(paint,1,prior)
        wrong=full[0].copy();wrong[t.rows>=136,0]+=15
        self.assertIsNone(t._regional_upper_refit(paint,regional,(wrong,full[1])))


if __name__=='__main__':unittest.main()
