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

    def test_opposite_curvature_changes_only_virtual_points_after_confirmation(self):
        t,pair,support=self.scene();t.scores=[.5,.7]
        pair[1][:,0]-=.004*(t.rows-148)**2
        baseline=t._display_continuation(pair,['partial','observed'],support)
        self.assertEqual(t.curve_guidance[0],0)
        for _ in range(20):out=t._display_continuation(pair,['partial','observed'],support)
        self.assertLess(t.curve_guidance[0],0)
        np.testing.assert_equal(out[0][support[0]],pair[0][support[0]])
        np.testing.assert_equal(out[1],pair[1])
        common=np.isfinite(out[0][:,0])&np.isfinite(baseline[0][:,0])&~support[0]
        change=out[0][common,0]-baseline[0][common,0]
        self.assertLess(change.min(),-.1)
        self.assertLessEqual(abs(change).max(),2)
        self.assertTrue(t._plausible_pair(out))

    def test_perspective_slope_is_not_copied_and_weak_reference_is_ignored(self):
        t,pair,support=self.scene();t.scores=[.5,.7]
        for _ in range(10):t._display_continuation(pair,['partial','observed'],support)
        self.assertAlmostEqual(t.curve_guidance[0],0,places=10)
        t,pair,support=self.scene();t.scores=[.5,.55]
        pair[1][:,0]-=.004*(t.rows-148)**2
        for _ in range(10):t._display_continuation(pair,['partial','observed'],support)
        self.assertEqual(t.curve_guidance[0],0)

    def test_reset_discards_guidance(self):
        t,_,_=self.scene();t.curve_guidance[:]=.001;t.curve_reference_hits[:]=10
        t.last_time=1.
        t.update(np.ones((180,320),np.uint8),np.zeros((180,320),np.uint8),0,
                 rgb=np.full((180,320,3),60,np.uint8))
        np.testing.assert_equal(t.curve_guidance,[0,0])
        np.testing.assert_equal(t.curve_reference_hits,[0,0])


if __name__=='__main__':unittest.main()
