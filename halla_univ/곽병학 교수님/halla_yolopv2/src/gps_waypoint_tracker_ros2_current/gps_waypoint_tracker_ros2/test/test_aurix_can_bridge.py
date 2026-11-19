import struct
import unittest

from gps_waypoint_tracker_ros2.aurix_can_bridge import (
    NOMINAL_DRIVING_CAN_ID,
    NOMINAL_DRIVING_ACTIVE_HEADER,
    NOMINAL_DRIVING_HEADER,
    NOMINAL_DRIVING_INACTIVE_HEADER,
    REQUEST_ACTIVE_VALID,
    REQUEST_INACTIVE,
    build_nominal_driving_payload,
    build_socketcan_frame,
    build_status_sequence,
    calculate_nominal_crc8,
    decode_nominal_driving_payload,
    decode_aurix_feedback_payload,
    nominal_header_for_request_status,
    move_towards,
    ramp_up_or_drop,
    waypoint_to_pass_index,
)


class AurixCanBridgeProtocolTest(unittest.TestCase):
    def test_active_nominal_payload_little_endian(self):
        payload = build_nominal_driving_payload(
            steering_deg=10.0,
            speed_kph=5.0,
            pass_index=0,
            request_status=REQUEST_ACTIVE_VALID,
            sequence=0,
        )
        self.assertEqual(
            payload,
            bytes([0x59, 0xF4, 0x01, 0x64, 0x00, 0x00, 0x40, 0xE3]),
        )
        decoded = decode_nominal_driving_payload(payload)
        self.assertEqual(decoded["header"], NOMINAL_DRIVING_HEADER)
        self.assertEqual(decoded["speed_kph"], 5.0)
        self.assertEqual(decoded["steering_deg"], 10.0)
        self.assertEqual(decoded["pass_index"], 0)
        self.assertEqual(decoded["request_status"], REQUEST_ACTIVE_VALID)
        self.assertEqual(decoded["sequence"], 0)

    def test_negative_speed_and_steering_are_signed(self):
        payload = build_nominal_driving_payload(
            steering_deg=-10.0,
            speed_kph=-1.25,
            pass_index=4,
            request_status=REQUEST_ACTIVE_VALID,
            sequence=31,
        )
        self.assertEqual(payload[:7], bytes([0x59, 0x83, 0xFF, 0x9C, 0xFF, 0x04, 0x5F]))
        decoded = decode_nominal_driving_payload(payload)
        self.assertEqual(decoded["speed_kph"], -1.25)
        self.assertEqual(decoded["steering_deg"], -10.0)
        self.assertEqual(decoded["sequence"], 31)

    def test_inactive_status_sequence(self):
        self.assertEqual(build_status_sequence(REQUEST_INACTIVE, 7), 0x07)
        self.assertEqual(build_status_sequence(REQUEST_ACTIVE_VALID, 31), 0x5F)
        self.assertEqual(build_status_sequence(REQUEST_ACTIVE_VALID, 32), 0x40)

    def test_header_enable_bit_matches_request_status(self):
        self.assertEqual(
            nominal_header_for_request_status(REQUEST_INACTIVE),
            NOMINAL_DRIVING_INACTIVE_HEADER,
        )
        self.assertEqual(
            nominal_header_for_request_status(REQUEST_ACTIVE_VALID),
            NOMINAL_DRIVING_ACTIVE_HEADER,
        )

        inactive_payload = build_nominal_driving_payload(
            steering_deg=0.0,
            speed_kph=0.0,
            pass_index=0,
            request_status=REQUEST_INACTIVE,
            sequence=7,
        )
        self.assertEqual(inactive_payload[0], 0x19)
        self.assertTrue(
            decode_nominal_driving_payload(inactive_payload)["header_valid"]
        )

    def test_socketcan_header_is_0x314(self):
        payload = build_nominal_driving_payload(
            steering_deg=0.0,
            speed_kph=0.0,
            pass_index=0,
            request_status=REQUEST_INACTIVE,
            sequence=0,
        )
        frame = build_socketcan_frame(NOMINAL_DRIVING_CAN_ID, payload)
        can_id, dlc, unpacked_payload = struct.unpack("=IB3x8s", frame)
        self.assertEqual(can_id, 0x314)
        self.assertEqual(dlc, 8)
        self.assertEqual(unpacked_payload, payload)
        self.assertEqual(unpacked_payload[0], 0x19)

    def test_default_pass_mapping(self):
        # Default code configuration uses only PassIndex 0 from waypoint 0 onward.
        self.assertEqual(waypoint_to_pass_index(-1), 0xFF)
        self.assertEqual(waypoint_to_pass_index(0), 0)
        self.assertEqual(waypoint_to_pass_index(1000), 0)

    def test_feedback_payload_decode(self):
        payload = struct.pack("<hhHBB", 50, -123, 2821, 9, 0xD7)
        feedback = decode_aurix_feedback_payload(payload)
        self.assertEqual(feedback["speed_kph"], 0.5)
        self.assertEqual(feedback["steering_deg"], -12.3)
        self.assertEqual(feedback["steering_adc_raw"], 2821)
        self.assertEqual(feedback["sequence"], 9)
        self.assertTrue(feedback["speed_valid"])
        self.assertTrue(feedback["steering_valid"])
        self.assertTrue(feedback["pc_control_active"])
        self.assertFalse(feedback["estop_active"])

    def test_uploaded_aurix_crc_vector(self):
        self.assertEqual(
            calculate_nominal_crc8(
                0x314,
                bytes.fromhex("59 64 00 00 00 00 40"),
            ),
            0xAD,
        )
        self.assertEqual(
            calculate_nominal_crc8(
                0x314,
                bytes.fromhex("19 00 00 00 00 00 00"),
            ),
            0x1D,
        )

    def test_steering_rate_limit(self):
        self.assertAlmostEqual(move_towards(0.0, 10.0, 0.4), 0.4)
        self.assertAlmostEqual(move_towards(2.0, -10.0, 0.4), 1.6)
        self.assertAlmostEqual(move_towards(1.0, 1.2, 0.4), 1.2)

    def test_speed_ramps_up_but_drops_immediately(self):
        self.assertAlmostEqual(ramp_up_or_drop(1.0, 8.0, 1.0, 0.05), 1.05)
        self.assertAlmostEqual(ramp_up_or_drop(8.0, 1.0, 1.0, 0.05), 1.0)


if __name__ == "__main__":
    unittest.main()
