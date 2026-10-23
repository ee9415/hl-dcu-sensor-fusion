import unittest
import numpy as np
from image_boundaries import ImageBoundaryTracker
from lane_boundaries import BoundaryConfig


class NnRankingTests(unittest.TestCase):
    def test_selects_supported_parallel_candidate_without_confidence_boost(self):
        paint=np.zeros((180,320),bool)
        lane=np.zeros_like(paint)
        for y in range(90,173):
            x=int(170+.6*(y-90))
            paint[y,x:x+2]=True
            paint[y,x+28:x+30]=True
            lane[y,x+28:x+30]=True
        baseline=ImageBoundaryTracker(BoundaryConfig(image_nn_rank_weight=0))
        enabled=ImageBoundaryTracker()
        baseline.nn_rank_mask=lane
        enabled.nn_rank_mask=lane
        a=baseline._detect_full(paint,1,None)
        b=enabled._detect_full(paint,1,None)
        self.assertIsNotNone(a)
        self.assertIsNotNone(b)
        common=np.isfinite(a[0][:,0])&np.isfinite(b[0][:,0])
        self.assertGreater(np.median(b[0][common,0]-a[0][common,0]),20)
        self.assertAlmostEqual(a[1],b[1],places=5)

    def test_nn_alone_cannot_create_paint_or_pass_geometry(self):
        t=ImageBoundaryTracker()
        t.nn_rank_mask=np.ones((180,320),bool)
        empty=np.zeros((180,320),bool)
        self.assertIsNone(t._detect_full(empty,0,None))
        horizontal=empty.copy();horizontal[160,20:300]=True
        self.assertIsNone(t._detect_regions(horizontal,0,None))
        short=empty.copy();short[155:160,80]=True
        self.assertIsNone(t._detect_full(short,0,None))

    def test_empty_frame_clears_support_and_limits_weight(self):
        t=ImageBoundaryTracker()
        t.nn_rank_mask=np.ones((180,320),bool)
        t.update(np.ones((180,320),np.uint8),np.zeros((180,320),np.uint8),0,
                 rgb=np.full((180,320,3),60,np.uint8))
        self.assertFalse(t.nn_rank_mask.any())
        with self.assertRaises(ValueError):BoundaryConfig(image_nn_rank_weight=.11)
        with self.assertRaises(ValueError):BoundaryConfig(image_nn_rank_weight=-.01)


if __name__=='__main__':unittest.main()
