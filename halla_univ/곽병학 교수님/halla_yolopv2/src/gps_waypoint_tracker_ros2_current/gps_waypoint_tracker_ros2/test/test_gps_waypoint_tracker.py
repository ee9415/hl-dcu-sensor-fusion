import unittest

import numpy as np
from sensor_msgs.msg import NavSatFix, NavSatStatus

from gps_waypoint_tracker_ros2.gps_waypoint_tracker import (
    final_waypoint_reached,
    gps_step_is_plausible,
    navsat_fix_is_rtk_float_or_fixed,
    nearest_route_segment_metrics,
    rolling_heading_vector,
    select_forward_start_index,
)


class ForwardStartIndexTest(unittest.TestCase):
    def setUp(self):
        self.waypoints = np.array(
            [
                [0.0, 10.0, 20.0, 30.0],
                [0.0, 0.0, 0.0, 0.0],
            ]
        )

    def test_selects_next_larger_index(self):
        nearest, target = select_forward_start_index(
            19.0,
            0.0,
            self.waypoints,
        )
        self.assertEqual(nearest, 2)
        self.assertEqual(target, 3)

    def test_last_waypoint_does_not_overflow(self):
        nearest, target = select_forward_start_index(
            31.0,
            0.0,
            self.waypoints,
        )
        self.assertEqual(nearest, 3)
        self.assertEqual(target, 3)

    def test_skips_lateral_near_points_for_smooth_forward_merge(self):
        waypoints = np.array(
            [
                [0.0, 3.0, 6.0, 9.0, 12.0, 15.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )
        nearest, target = select_forward_start_index(
            3.1,
            4.0,
            waypoints,
            lookahead_distance=10.0,
        )
        self.assertEqual(nearest, 1)
        self.assertEqual(target, 5)

    def test_rejects_empty_waypoints(self):
        with self.assertRaises(ValueError):
            select_forward_start_index(0.0, 0.0, np.empty((2, 0)))

    def test_completion_requires_final_index_and_radius(self):
        self.assertFalse(
            final_waypoint_reached(30.0, 0.0, self.waypoints, 2, 2.8)
        )
        self.assertTrue(
            final_waypoint_reached(31.0, 0.0, self.waypoints, 3, 2.8)
        )
        self.assertFalse(
            final_waypoint_reached(33.0, 0.0, self.waypoints, 3, 2.8)
        )

    def test_heading_ignores_sub_meter_gps_noise(self):
        vector = rolling_heading_vector(
            [0.0, 0.02, -0.01, 0.03],
            [0.0, -0.01, 0.02, 0.01],
            [0.0, 0.1, 0.2, 0.3],
            1.0,
            5.0,
        )
        self.assertIsNone(vector)

    def test_heading_uses_rolling_one_meter_chord(self):
        vector = rolling_heading_vector(
            [0.0, 0.4, 0.8, 1.2],
            [0.0, 0.0, 0.1, 0.1],
            [0.0, 0.5, 1.0, 1.5],
            1.0,
            5.0,
        )
        np.testing.assert_allclose(vector, np.array([1.2, 0.1]))

    def test_heading_rejects_slow_stationary_drift(self):
        vector = rolling_heading_vector(
            [0.0, 0.4, 0.8, 1.2],
            [0.0, 0.0, 0.1, 0.1],
            [0.0, 10.0, 20.0, 30.0],
            1.0,
            5.0,
        )
        self.assertIsNone(vector)

    def test_rejects_impossible_single_fix_jump(self):
        self.assertFalse(
            gps_step_is_plausible(
                step_distance_m=1.13,
                elapsed_sec=0.10,
                feedback_speed_kph=0.90,
            )
        )

    def test_accepts_step_consistent_with_six_kph(self):
        self.assertTrue(
            gps_step_is_plausible(
                step_distance_m=0.20,
                elapsed_sec=0.10,
                feedback_speed_kph=6.0,
            )
        )

    def test_rejects_non_finite_step(self):
        self.assertFalse(
            gps_step_is_plausible(float("nan"), 0.10, 1.0)
        )

    def test_rtk_gate_accepts_fixed_and_bounded_float(self):
        fix = NavSatFix()
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        fix.position_covariance[0] = 0.0025
        fix.position_covariance[4] = 0.0025
        self.assertTrue(navsat_fix_is_rtk_float_or_fixed(fix))

        fix.position_covariance[0] = 0.0625
        fix.position_covariance[4] = 0.0625
        self.assertTrue(navsat_fix_is_rtk_float_or_fixed(fix))

        fix.position_covariance[0] = 0.36
        fix.position_covariance[4] = 0.36
        self.assertFalse(navsat_fix_is_rtk_float_or_fixed(fix))

    def test_rtk_fixed_gate_rejects_non_gbas_fix(self):
        fix = NavSatFix()
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        fix.position_covariance[0] = 0.001
        fix.position_covariance[4] = 0.001
        self.assertFalse(navsat_fix_is_rtk_float_or_fixed(fix))

    def test_route_metrics_find_local_cross_track_and_tangent(self):
        waypoints = np.array(
            [[0.0, 10.0, 20.0, 30.0], [0.0, 0.0, 0.0, 0.0]]
        )
        distance, tangent, segment = nearest_route_segment_metrics(
            12.0,
            0.4,
            waypoints,
            target_index=2,
        )
        self.assertAlmostEqual(distance, 0.4)
        np.testing.assert_allclose(tangent, np.array([10.0, 0.0]))
        self.assertEqual(segment, 1)


if __name__ == "__main__":
    unittest.main()
