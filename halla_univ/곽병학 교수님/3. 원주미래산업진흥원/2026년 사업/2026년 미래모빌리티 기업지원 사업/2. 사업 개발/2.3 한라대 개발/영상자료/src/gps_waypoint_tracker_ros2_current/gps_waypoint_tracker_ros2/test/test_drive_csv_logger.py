import json
import math
import unittest

from gps_waypoint_tracker_ros2.drive_csv_logger import (
    age_or_blank,
    bool_or_blank,
    finite_or_blank,
    int_hex_or_blank,
    lat_lon_to_meters,
    parse_json_object,
)


class DriveCsvLoggerHelpersTest(unittest.TestCase):
    def test_web_mercator_conversion_is_finite(self):
        x_m, y_m = lat_lon_to_meters(37.0, 127.0)

        self.assertTrue(math.isfinite(x_m))
        self.assertTrue(math.isfinite(y_m))
        self.assertGreater(x_m, 0.0)
        self.assertGreater(y_m, 0.0)

    def test_json_status_requires_an_object(self):
        value = parse_json_object(
            json.dumps(
                {
                    "enabled": True,
                    "sequence": 7,
                }
            )
        )
        self.assertTrue(value["enabled"])
        self.assertEqual(value["sequence"], 7)

        with self.assertRaises(ValueError):
            parse_json_object("[1, 2, 3]")

    def test_csv_value_helpers(self):
        self.assertEqual(finite_or_blank(None), "")
        self.assertEqual(finite_or_blank(float("nan")), "")
        self.assertEqual(finite_or_blank(1.25), 1.25)
        self.assertEqual(bool_or_blank(None), "")
        self.assertEqual(bool_or_blank(False), 0)
        self.assertEqual(bool_or_blank(True), 1)
        self.assertEqual(int_hex_or_blank(None, 3), "")
        self.assertEqual(int_hex_or_blank(0x300, 3), "0x300")

    def test_age_is_non_negative(self):
        self.assertEqual(age_or_blank(None, 5.0), "")
        self.assertEqual(age_or_blank(4.5, 5.0), 0.5)
        self.assertEqual(age_or_blank(5.5, 5.0), 0.0)


if __name__ == "__main__":
    unittest.main()
