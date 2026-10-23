import unittest
import numpy as np
from image_boundaries import ImageBoundaryTracker


class VirtualTailTests(unittest.TestCase):
    def scene(self):
        t=ImageBoundaryTracker();t.short_pair_hits=[3,0]
        y=t.rows
        left=np.column_stack((116-.9*(y-148),y))
        right=np.column_stack((151+1.3*(y-128),y))
        left[(y<148)|(y>164),0]=np.nan
        right[(y<120)|(y>164),0]=np.nan
        pair=[left,right];support=[np.isfinite(p[:,0]) for p in pair]
        return t,pair,support

    def test_continuation_is_bounded_and_does_not_create_observed_support(self):
        t,pair,support=self.scene();before=[p.copy() for p in pair]
        out=t._display_continuation(pair,['partial','observed'],support)
        added=np.isfinite(out[0][:,0])&~support[0]
        self.assertGreater(added.sum(),0)
        self.assertGreaterEqual(t.rows[added].min(),124)
        self.assertTrue(t._plausible_pair(out))
        for a,b in zip(pair,before):np.testing.assert_equal(a,b)
        np.testing.assert_equal(out[0][support[0]],pair[0][support[0]])
        np.testing.assert_equal(out[1],pair[1])
        self.assertEqual(support[0].sum(),9)

    def test_confirmation_and_opposite_required(self):
        t,pair,support=self.scene();t.short_pair_hits=[2,0]
        out=t._display_continuation(pair,['partial','observed'],support)
        np.testing.assert_equal(out[0],pair[0])
        t.short_pair_hits=[3,0]
        out=t._display_continuation(pair,['partial','held'],support)
        np.testing.assert_equal(out[0],pair[0])

    def test_steep_fragment_cannot_launch_unbounded_tail(self):
        t,pair,support=self.scene()
        pair[0][:,0]=np.where(support[0],116-2*(t.rows-148),np.nan)
        out=t._display_continuation(pair,['partial','observed'],support)
        np.testing.assert_equal(out[0],pair[0])


if __name__=='__main__':unittest.main()
