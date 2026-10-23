import unittest
from pathlib import Path
import numpy as np
from image_boundaries import ImageBoundaryTracker


class UpperGapTests(unittest.TestCase):
    def load(self):
        z=np.load(Path(__file__).parent/'data/right_upper_gap.npz')
        t=ImageBoundaryTracker();t.nn_rank_mask=z['lane']>0
        return t,z['paint'],z['prior']
    def test_captured_upper_paint_wins_with_and_without_stale_history(self):
        for history in (True,False):
            t,paint,prior=self.load()
            o=t._detect(paint,1,prior if history else None)
            at100=o[0][np.argmin(abs(t.rows-100)),0]
            self.assertLess(abs(at100-179),3)
            self.assertTrue(t._smooth_shape(o[0]))
    def test_upper_replacement_requires_actual_pixels(self):
        t,paint,prior=self.load()
        regional=t._detect_regions(paint,1,prior);full=t._detect_full(paint,1,prior)
        missing=paint.copy();missing[:124]=False
        self.assertIsNone(t._regional_upper_refit(missing,regional,full))
    def test_separate_near_line_cannot_be_joined_to_upper(self):
        t,paint,prior=self.load()
        regional=t._detect_regions(paint,1,prior);full=t._detect_full(paint,1,prior)
        wrong=full[0].copy();wrong[t.rows>=136,0]+=15
        self.assertIsNone(t._regional_upper_refit(paint,regional,(wrong,full[1])))


if __name__=='__main__':unittest.main()
