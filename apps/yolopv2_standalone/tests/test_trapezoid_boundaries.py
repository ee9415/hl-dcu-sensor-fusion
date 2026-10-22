import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from trapezoid_boundaries import TrapezoidBoundaryTracker, TrapezoidConfig, interpolate_vertices


class TrapezoidTests(unittest.TestCase):
    def scene(self, curve=0., shift=0., left=True, right=True, road_shape=False):
        image = np.full((180, 320, 3), 60, np.uint8)
        road = np.ones((180, 320), np.uint8) if not road_shape else np.zeros((180, 320), np.uint8)
        for y in range(90, 179):
            bend = shift+curve*((174-y)/82)**2
            a, b = 160-.9*(y-65)+bend, 160+.9*(y-65)+bend
            if road_shape:
                road[y, max(0, int(a)):min(320, int(b)+1)] = 1
            for enabled, x in ((left, a), (right, b)):
                if enabled:
                    x = int(round(x))
                    image[y, max(0, x-1):min(320, x+2)] = 235
        return image, road

    def update(self, tracker, image, road, n):
        with patch('trapezoid_boundaries.road_motion', return_value=(np.eye(2, 3), 0.)):
            return tracker.update(road, np.zeros_like(road), n*.1, rgb=image)

    def test_straight_shared_vertices_follow_white_paint(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene()
        for n in range(8): b = self.update(t, image, road, n)
        self.assertTrue(b.pair_valid)
        self.assertEqual(b.region_count, 7)
        for side, sign in ((0, -1), (1, 1)):
            truth = 160+sign*.9*(t.rows-65)
            self.assertLess(np.max(abs(b.vertices[side, :, 0]-truth)), 3)
        self.assertEqual(b.left_state, 'observed')
        self.assertEqual(b.right_state, 'observed')
        data = b.telemetry()
        self.assertEqual(data['boundary_model'], 'trapezoid_segments')
        self.assertFalse(data['segment_distance_calibrated'])
        self.assertIsNone(data['segment_length_m'])
        json.dumps(data, allow_nan=False)

    def test_curve_moves_far_sections_and_keeps_gap(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene(curve=24, road_shape=True)
        for n in range(30): b = self.update(t, image, road, n)
        self.assertTrue(b.pair_valid)
        self.assertGreater(b.vertices[:, -1, 0].mean()-160, 12)
        for side, sign in ((0, -1), (1, 1)):
            truth = 160+sign*.9*(t.rows-65)+24*((174-t.rows)/82)**2
            self.assertLess(np.mean(abs(b.vertices[side, :, 0]-truth)), 5)
        gap = b.vertices[1, :, 0]-b.vertices[0, :, 0]
        self.assertTrue(np.all(gap > 12))

    def test_road_widening_does_not_widen_corridor(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene(left=False, right=False, road_shape=True)
        for n in range(5): self.update(t, image, road, n)
        width = t.edges[1]-t.edges[0]
        for n in range(5, 50): b = self.update(t, image, np.ones_like(road), n)
        np.testing.assert_allclose(b.vertices[1, :, 0]-b.vertices[0, :, 0], width)
        self.assertEqual(b.left_state, 'held')
        self.assertEqual(b.left_confidence, 0)

    def test_road_guidance_translates_sections_without_expansion(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene(shift=45, left=False, right=False, road_shape=True)
        before = t.edges.copy()
        self.update(t, image, road, 0)
        np.testing.assert_allclose(t.edges[1]-t.edges[0], before[1]-before[0])
        self.assertGreater(t.edges[:, -1].mean(), before[:, -1].mean())

    def test_one_side_preserves_learned_gap_at_each_cross_section(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene()
        for n in range(10): self.update(t, image, road, n)
        width = t.width.copy()
        image, road = self.scene(shift=10, right=False)
        for n in range(10, 30): b = self.update(t, image, road, n)
        self.assertTrue(b.pair_valid)
        np.testing.assert_allclose(t.edges[1]-t.edges[0], width, atol=3)
        self.assertEqual(b.right_state, 'inferred')
        self.assertLessEqual(b.right_confidence, .15)

    def test_long_loss_retains_regions_with_low_confidence(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene()
        for n in range(5): self.update(t, image, road, n)
        image[:] = 60
        for n in range(5, 155): b = self.update(t, image, np.zeros_like(road), n)
        self.assertTrue(b.pair_valid)
        self.assertEqual(b.region_count, 7)
        self.assertLess(b.left_confidence, .001)
        self.assertLess(b.right_confidence, .001)

    def test_reacquisition_is_local_and_rate_limited(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene()
        for n in range(5): self.update(t, image, road, n)
        image, road = self.scene(shift=14)
        for n in range(5, 20):
            before = t.edges.copy()
            b = self.update(t, image, road, n)
            self.assertLessEqual(np.max(abs(t.edges-before)), 5.01)
        self.assertGreater(t.edges[:, 0].mean(), 170)
        self.assertEqual(b.left_state, 'observed')

    def test_far_render_end_is_below_road_horizon(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene()
        road[:120] = 0
        for n in range(5): b = self.update(t, image, road, n)
        self.assertLess(b.region_count, 7)
        for p in (b.image_left, b.image_right):
            good = np.isfinite(p[:, 0])
            self.assertGreaterEqual(np.min(p[good, 1]), 120)

    def test_vehicle_and_stop_bar_do_not_become_side_paint(self):
        t = TrapezoidBoundaryTracker()
        image, road = self.scene()
        b = t.update(road, road*0, 0, rgb=image,
                     detections=[SimpleNamespace(xmin=0, ymin=0, xmax=1, ymax=1)])
        self.assertEqual(b.left_confidence, 0)
        self.assertEqual(b.right_confidence, 0)
        image[:] = 60
        image[137:143, 20:300] = 235
        for n in range(1, 5): b = self.update(t, image, road, n)
        self.assertEqual(b.left_confidence, 0)
        self.assertEqual(b.right_confidence, 0)

    def test_interpolator_passes_shared_vertices_without_overshoot(self):
        rows = np.array([174., 150., 130., 110., 92.])
        values = np.array([60., 80., 83., 120., 125.])
        np.testing.assert_allclose(interpolate_vertices(rows, values, rows), values)
        dense = interpolate_vertices(rows, values, np.linspace(174, 92, 200))
        self.assertTrue(np.all(np.diff(dense) >= -1e-6))
        self.assertGreaterEqual(dense.min(), values.min())
        self.assertLessEqual(dense.max(), values.max())

    def test_invalid_config_and_timestamp_reset(self):
        with self.assertRaises(ValueError): TrapezoidConfig(rows=(100, 150, 170))
        with self.assertRaises(ValueError): TrapezoidConfig(correction_px=0)
        t = TrapezoidBoundaryTracker()
        image, road = self.scene(shift=15)
        for n in range(10): self.update(t, image, road, n)
        b = self.update(t, np.full_like(image, 60), road, -1)
        np.testing.assert_allclose(t.edges, t.base)
        self.assertEqual(b.left_confidence, 0)


if __name__ == '__main__':
    unittest.main()
