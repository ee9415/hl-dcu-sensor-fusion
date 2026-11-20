#!/usr/bin/env python3
"""Bridge GPS steering/waypoint data to the PPT Nominal Driving CAN protocol."""

import json
import math
import socket
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, Int32, String, UInt16, UInt8MultiArray


# =============================================================================
# USER-EDITABLE NOMINAL DRIVING SETTINGS
# =============================================================================
# PPT Nominal Driving CAN ID / Enable-bit headers
NOMINAL_DRIVING_CAN_ID = 0x314
NOMINAL_DRIVING_INACTIVE_HEADER = 0x19
NOMINAL_DRIVING_ACTIVE_HEADER = 0x59
# Backward-compatible name used by existing callers/tests.
NOMINAL_DRIVING_HEADER = NOMINAL_DRIVING_ACTIVE_HEADER

# Waypoint index at which each PassIndex STARTS.
# Edit ONLY these values to match the real course.
# - Use None for a PassIndex you do not use.
# - PassIndex 0 starts at waypoint 0 by default.
#
# Example:
#   0: 0,    # waypoint 0~19   -> PassIndex 0
#   1: 20,   # waypoint 20~39  -> PassIndex 1
#   2: 40,   # waypoint 40~... -> PassIndex 2
#
# PPT PassIndex meanings:
#   0 = straight
#   1 = static obstacle
#   2 = hill
#   3 = dynamic obstacle / acceleration
#   4 = parking
#   5 = intersection
#   6 = selected lane
PASS_INDEX_START_WAYPOINTS = {
    0: 0,
    1: None,
    2: None,
    3: None,
    4: None,
    5: None,
    6: None,
}

# B7 CRC-8 profile implemented by the uploaded AURIX application:
# SAE-J1850, poly=0x1D, init=0xFF, xorout=0xFF, non-reflected. Input is the
# two-byte standard CAN ID (high then low) followed by B0..B6.
CRC8_POLYNOMIAL = 0x1D
CRC8_INITIAL = 0xFF
CRC8_XOR_OUT = 0xFF
# =============================================================================


SENSOR_FEEDBACK_CAN_ID = 0x130
AURIX_DIAGNOSTIC_CAN_IDS = (0x131, 0x132, 0x133, 0x134)
CAN_DLC = 8
CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
FEEDBACK_SIGNATURE = 0xD0

# B6 RequestStatus (upper 3 bits)
REQUEST_INACTIVE = 0b000
REQUEST_READY = 0b001
REQUEST_ACTIVE_VALID = 0b010
REQUEST_ACTIVE_DEGRADED = 0b011
REQUEST_COMPLETE = 0b100
REQUEST_ABORT = 0b101
REQUEST_FAULT = 0b110
REQUEST_INVALID = 0b111

CAN_EFF_FLAG = getattr(socket, "CAN_EFF_FLAG", 0x80000000)
CAN_RTR_FLAG = getattr(socket, "CAN_RTR_FLAG", 0x40000000)
CAN_ERR_FLAG = getattr(socket, "CAN_ERR_FLAG", 0x20000000)
CAN_SFF_MASK = getattr(socket, "CAN_SFF_MASK", 0x000007FF)


def nominal_header_for_request_status(request_status):
    """Return B0 with the drive-enable bit matching RequestStatus."""
    request_status = int(request_status) & 0x07
    if request_status in (REQUEST_ACTIVE_VALID, REQUEST_ACTIVE_DEGRADED):
        return NOMINAL_DRIVING_ACTIVE_HEADER
    return NOMINAL_DRIVING_INACTIVE_HEADER


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def round_half_away_from_zero(value):
    """Match conventional/C-style rounding instead of Python banker's rounding."""
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return -int(math.floor(-value + 0.5))


def move_towards(current, target, maximum_step):
    """Move current toward target by no more than a non-negative step."""
    current = float(current)
    target = float(target)
    maximum_step = float(maximum_step)
    if maximum_step < 0.0:
        raise ValueError("maximum_step must be non-negative")
    if target > current:
        return min(target, current + maximum_step)
    return max(target, current - maximum_step)


def ramp_up_or_drop(current, target, rise_rate, elapsed_sec):
    """Rate-limit acceleration while allowing an immediate speed reduction."""
    current = float(current)
    target = float(target)
    rise_rate = float(rise_rate)
    elapsed_sec = max(0.0, float(elapsed_sec))
    if rise_rate <= 0.0:
        raise ValueError("rise_rate must be positive")
    if target <= current:
        return target
    return min(target, current + rise_rate * elapsed_sec)


def speed_kph_to_x100(speed_kph):
    """PPT B1~B2: sint16, 0.01 km/h/bit."""
    raw = round_half_away_from_zero(float(speed_kph) * 100.0)
    return clamp(raw, -32768, 32767)


def steering_degrees_to_x10(steering_deg):
    """PPT B3~B4: sint16, 0.1 degree/bit."""
    raw = round_half_away_from_zero(float(steering_deg) * 10.0)
    return clamp(raw, -32768, 32767)


def waypoint_to_pass_index(waypoint_index):
    """Map current waypoint index to PPT B5 PassIndex using code constants above."""
    waypoint_index = int(waypoint_index)
    if waypoint_index < 0:
        return 0xFF

    enabled = []
    for pass_index, start_index in PASS_INDEX_START_WAYPOINTS.items():
        if start_index is None:
            continue
        pass_index = int(pass_index)
        start_index = int(start_index)
        if not 0 <= pass_index <= 6:
            raise ValueError("PassIndex keys must be in range 0..6")
        if start_index < 0:
            raise ValueError("PassIndex start waypoint must be >= 0 or None")
        enabled.append((start_index, pass_index))

    if not enabled:
        return 0xFF

    enabled.sort()
    selected = 0xFF
    for start_index, pass_index in enabled:
        if waypoint_index >= start_index:
            selected = pass_index
        else:
            break
    return selected


def build_status_sequence(request_status, sequence):
    """PPT B6: upper 3 bits RequestStatus + lower 5 bits Sequence."""
    request_status = int(request_status) & 0x07
    sequence = int(sequence) & 0x1F
    return (request_status << 5) | sequence


def calculate_nominal_crc8(can_id, payload_b0_to_b6):
    """Return the AURIX nominal-command SAE-J1850 CRC-8."""
    can_id = int(can_id)
    if not 0 <= can_id <= CAN_SFF_MASK:
        raise ValueError("can_id must be a standard 11-bit ID")
    if len(payload_b0_to_b6) != 7:
        raise ValueError("CRC input payload must be B0..B6")

    crc = CRC8_INITIAL
    crc_input = bytes([(can_id >> 8) & 0xFF, can_id & 0xFF]) + bytes(
        payload_b0_to_b6
    )
    for byte in crc_input:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLYNOMIAL) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ CRC8_XOR_OUT


def build_nominal_driving_payload(
    steering_deg,
    speed_kph,
    pass_index,
    request_status,
    sequence,
    can_id=NOMINAL_DRIVING_CAN_ID,
):
    """Build PPT 0x314 Nominal Driving 8-byte payload.

    Layout (little-endian chosen for the two sint16 fields):
      B0    = 0x19 when inactive, 0x59 only for an active request
      B1~B2 = speed, sint16, 0.01 km/h/bit
      B3~B4 = steering, sint16, 0.1 deg/bit
      B5    = PassIndex
      B6    = RequestStatus[7:5] + Sequence[4:0]
      B7    = AURIX SAE-J1850 CRC-8
    """
    pass_index = int(pass_index)
    if pass_index not in range(0, 7) and pass_index != 0xFF:
        raise ValueError("pass_index must be 0..6 or 0xFF")

    speed_raw = speed_kph_to_x100(speed_kph)
    steering_raw = steering_degrees_to_x10(steering_deg)
    status_sequence = build_status_sequence(request_status, sequence)
    header = nominal_header_for_request_status(request_status)

    payload_b0_to_b6 = struct.pack(
        "<BhhBB",
        header,
        speed_raw,
        steering_raw,
        pass_index,
        status_sequence,
    )
    crc = calculate_nominal_crc8(can_id, payload_b0_to_b6)
    return payload_b0_to_b6 + bytes([crc])


def decode_nominal_driving_payload(payload):
    if len(payload) != CAN_DLC:
        raise ValueError("Nominal Driving payload must be exactly 8 bytes")

    header, speed_raw, steering_raw, pass_index, status_sequence, crc = (
        struct.unpack("<BhhBBB", payload)
    )
    request_status = (status_sequence >> 5) & 0x07
    sequence = status_sequence & 0x1F
    expected_header = nominal_header_for_request_status(request_status)
    return {
        "header": header,
        "header_valid": header == expected_header,
        "header_active": header == NOMINAL_DRIVING_ACTIVE_HEADER,
        "speed_kph_x100": speed_raw,
        "speed_kph": speed_raw / 100.0,
        "steering_deg_x10": steering_raw,
        "steering_deg": steering_raw / 10.0,
        "pass_index": pass_index,
        "request_status": request_status,
        "sequence": sequence,
        "status_sequence": status_sequence,
        "crc": crc,
    }


def build_socketcan_frame(can_id, payload):
    if len(payload) != CAN_DLC:
        raise ValueError("CAN payload must be exactly 8 bytes")
    if not 0 <= int(can_id) <= CAN_SFF_MASK:
        raise ValueError("CAN ID must be a standard 11-bit identifier")

    return struct.pack(
        CAN_FRAME_FORMAT,
        int(can_id) & CAN_SFF_MASK,
        CAN_DLC,
        payload,
    )


def decode_aurix_feedback_payload(payload):
    """Keep the existing optional 0x130 feedback decoder unchanged."""
    if len(payload) != CAN_DLC:
        raise ValueError("AURIX feedback payload must be exactly 8 bytes")

    speed_x100, steering_x10, adc_raw, sequence, flags = struct.unpack(
        "<hhHBB",
        payload,
    )
    if (flags & 0xF0) != FEEDBACK_SIGNATURE:
        raise ValueError(f"Invalid AURIX feedback signature: 0x{flags:02X}")

    return {
        "speed_kph": speed_x100 / 100.0,
        "steering_deg": steering_x10 / 10.0,
        "steering_adc_raw": adc_raw,
        "sequence": sequence,
        "speed_valid": bool(flags & 0x01),
        "steering_valid": bool(flags & 0x02),
        "pc_control_active": bool(flags & 0x04),
        "estop_active": bool(flags & 0x08),
        "flags": flags,
    }


class AurixCanBridge(Node):
    """Send PPT Nominal Driving commands at a fixed period."""

    def __init__(self):
        super().__init__("aurix_can_bridge")

        self.declare_parameter("steering_topic", "/filtered_steering_angle1")
        self.declare_parameter("waypoint_index_topic", "/current_waypoint_index1")
        self.declare_parameter("route_completion_topic", "/route_completed1")
        self.declare_parameter("heading_valid_topic", "/heading_valid1")
        self.declare_parameter("rtk_fixed_topic", "/rtk_fixed1")
        self.declare_parameter(
            "route_alignment_topic", "/route_alignment_ready1"
        )
        self.declare_parameter(
            "route_cross_track_topic", "/route_cross_track_error1"
        )
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("can_id", NOMINAL_DRIVING_CAN_ID)
        self.declare_parameter("feedback_can_id", SENSOR_FEEDBACK_CAN_ID)
        self.declare_parameter("send_period_sec", 0.1)
        self.declare_parameter("command_timeout_sec", 0.20)
        self.declare_parameter("target_speed_kph", 0.0)
        self.declare_parameter("enable_drive", False)
        self.declare_parameter("invert_steering", True)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("startup_rearm_frames", 10)
        self.declare_parameter("crawl_speed_kph", 1.0)
        self.declare_parameter("maximum_speed_kph", 8.0)
        self.declare_parameter("speed_ramp_kph_per_sec", 1.0)
        self.declare_parameter("steering_command_scale", 0.70)
        self.declare_parameter("maximum_tracker_steering_deg", 7.0)
        self.declare_parameter("saturated_steering_speed_kph", 1.0)
        self.declare_parameter("max_steering_rate_deg_per_sec", 2.0)
        self.declare_parameter("feedback_timeout_sec", 0.20)
        self.declare_parameter("max_steering_tracking_error_deg", 5.0)
        self.declare_parameter("steering_error_timeout_sec", 0.30)
        self.declare_parameter("steering_center_tolerance_deg", 4.0)
        self.declare_parameter("steering_center_hold_sec", 0.50)
        self.declare_parameter("pc_active_timeout_sec", 0.50)
        self.declare_parameter("maximum_alignment_steering_lag_deg", 1.0)
        self.declare_parameter("maximum_alignment_tracking_error_deg", 2.0)
        self.declare_parameter("route_state_timeout_sec", 0.20)
        self.declare_parameter("maximum_route_cross_track_m", 0.50)
        self.declare_parameter("drive_promotion_hold_sec", 1.0)

        steering_topic = self.get_parameter("steering_topic").value
        waypoint_index_topic = self.get_parameter("waypoint_index_topic").value
        route_completion_topic = self.get_parameter("route_completion_topic").value
        heading_valid_topic = self.get_parameter("heading_valid_topic").value
        rtk_fixed_topic = self.get_parameter("rtk_fixed_topic").value
        route_alignment_topic = self.get_parameter(
            "route_alignment_topic"
        ).value
        route_cross_track_topic = self.get_parameter(
            "route_cross_track_topic"
        ).value
        send_period_sec = float(self.get_parameter("send_period_sec").value)
        if send_period_sec <= 0.0:
            raise ValueError("send_period_sec must be positive")

        startup_rearm_frames = int(
            self.get_parameter("startup_rearm_frames").value
        )
        if startup_rearm_frames < 1:
            raise ValueError("startup_rearm_frames must be at least 1")

        positive_parameters = (
            "crawl_speed_kph",
            "maximum_speed_kph",
            "speed_ramp_kph_per_sec",
            "steering_command_scale",
            "maximum_tracker_steering_deg",
            "saturated_steering_speed_kph",
            "max_steering_rate_deg_per_sec",
            "feedback_timeout_sec",
            "max_steering_tracking_error_deg",
            "steering_error_timeout_sec",
            "steering_center_tolerance_deg",
            "steering_center_hold_sec",
            "pc_active_timeout_sec",
            "maximum_alignment_steering_lag_deg",
            "maximum_alignment_tracking_error_deg",
            "route_state_timeout_sec",
            "maximum_route_cross_track_m",
            "drive_promotion_hold_sec",
        )
        for parameter_name in positive_parameters:
            if float(self.get_parameter(parameter_name).value) <= 0.0:
                raise ValueError(f"{parameter_name} must be positive")
        if float(self.get_parameter("steering_command_scale").value) > 1.0:
            raise ValueError("steering_command_scale must not exceed 1.0")

        can_id = int(self.get_parameter("can_id").value)
        feedback_can_id = int(self.get_parameter("feedback_can_id").value)
        if not 0 <= can_id <= CAN_SFF_MASK:
            raise ValueError("can_id must be a standard 11-bit ID")
        if not 0 <= feedback_can_id <= CAN_SFF_MASK:
            raise ValueError("feedback_can_id must be a standard 11-bit ID")

        self._can_socket = None
        self._last_connect_attempt = 0.0
        self._latest_steering_deg = 0.0
        self._last_steering_time = None
        self._current_waypoint_index = -1
        self._pass_index = 0xFF
        self._route_completed = None
        self._heading_valid = None
        self._rtk_fixed = None
        self._route_alignment_ready = None
        self._route_cross_track_error_m = None
        self._last_route_alignment_time = None
        self._last_route_cross_track_time = None
        self._drive_promotion_ready_since = None
        self._sequence = 0
        self._last_status_log_time = 0.0
        self._last_dry_run = bool(self.get_parameter("dry_run").value)
        self._startup_rearm_frames = startup_rearm_frames
        self._rearm_frames_remaining = startup_rearm_frames
        self._limited_speed_kph = 0.0
        self._limited_steering_deg = 0.0
        self._last_limiter_time = None
        self._last_sent_enabled = False
        self._last_sent_speed_kph = 0.0
        self._last_sent_steering_deg = 0.0
        self._latest_feedback = None
        self._last_feedback_time = None
        self._last_feedback_sequence = None
        self._last_feedback_sequence_change_time = None
        self._steering_error_started_at = None
        self._safety_fault_latched = False
        self._safety_fault_reason = ""
        self._latest_diagnostic_frames = {}
        self._last_vm_state = None
        self._vm_rearm_in_progress = False
        self._vm_rearm_attempts = 0
        self._active_started_at = None
        self._steering_centered_since = None
        self._steering_centered_ready = False
        self._desired_speed_kph = 0.0
        self._desired_steering_deg = 0.0
        self._tracker_requested_steering_deg = 0.0
        self._alignment_steering_lag_deg = 0.0
        self._alignment_tracking_error_deg = 0.0
        self._drive_mode = "INACTIVE"

        self._steering_subscription = self.create_subscription(
            Float32,
            steering_topic,
            self._steering_callback,
            10,
        )
        waypoint_index_qos = QoSProfile(depth=10)
        waypoint_index_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._waypoint_subscription = self.create_subscription(
            Int32,
            waypoint_index_topic,
            self._waypoint_callback,
            waypoint_index_qos,
        )
        self._route_completion_subscription = self.create_subscription(
            Bool,
            route_completion_topic,
            self._route_completion_callback,
            waypoint_index_qos,
        )
        self._heading_valid_subscription = self.create_subscription(
            Bool,
            heading_valid_topic,
            self._heading_valid_callback,
            waypoint_index_qos,
        )
        self._rtk_fixed_subscription = self.create_subscription(
            Bool,
            rtk_fixed_topic,
            self._rtk_fixed_callback,
            waypoint_index_qos,
        )
        self._route_alignment_subscription = self.create_subscription(
            Bool,
            route_alignment_topic,
            self._route_alignment_callback,
            waypoint_index_qos,
        )
        self._route_cross_track_subscription = self.create_subscription(
            Float32,
            route_cross_track_topic,
            self._route_cross_track_callback,
            waypoint_index_qos,
        )
        self._tx_payload_publisher = self.create_publisher(
            UInt8MultiArray,
            "/aurix/can_tx_payload",
            10,
        )
        self._tx_status_publisher = self.create_publisher(
            String,
            "/aurix/can_tx_status",
            10,
        )
        self._feedback_speed_publisher = self.create_publisher(
            Float32,
            "/aurix/feedback/speed_kph",
            10,
        )
        self._feedback_steering_publisher = self.create_publisher(
            Float32,
            "/aurix/feedback/steering_angle_deg",
            10,
        )
        self._feedback_adc_publisher = self.create_publisher(
            UInt16,
            "/aurix/feedback/steering_adc_raw",
            10,
        )
        self._feedback_status_publisher = self.create_publisher(
            String,
            "/aurix/feedback/status",
            10,
        )

        self._send_timer = self.create_timer(send_period_sec, self._send_timer_callback)

        self.get_logger().info(
            f"Nominal Driving bridge ready: CAN ID=0x{can_id:03X}, "
            f"B0 inactive/active="
            f"0x{NOMINAL_DRIVING_INACTIVE_HEADER:02X}/"
            f"0x{NOMINAL_DRIVING_ACTIVE_HEADER:02X}, "
            f"period={send_period_sec:.3f}s, "
            f"steering_topic={steering_topic}, waypoint_topic={waypoint_index_topic}, "
            f"completion_topic={route_completion_topic}, "
            f"heading_topic={heading_valid_topic}, "
            f"rtk_fixed_topic={rtk_fixed_topic}, "
            f"route_alignment_topic={route_alignment_topic}, "
            f"dry_run={self._last_dry_run}, "
            f"startup_rearm_frames={self._startup_rearm_frames}"
        )
        self.get_logger().info(
            "PassIndex mapping is code-configured in PASS_INDEX_START_WAYPOINTS "
            "inside aurix_can_bridge.py"
        )
        self.get_logger().info(
            "B7 CRC enabled: SAE-J1850 poly=0x1D init/xorout=0xFF, "
            "input=CAN-ID(high,low)+B0..B6"
        )

    def _steering_callback(self, message):
        steering_deg = float(message.data)
        if not math.isfinite(steering_deg):
            self._last_steering_time = None
            self.get_logger().error(
                "Non-finite steering command received; sending Inactive command"
            )
            return
        self._latest_steering_deg = steering_deg
        self._last_steering_time = time.monotonic()

    def _waypoint_callback(self, message):
        waypoint_index = int(message.data)
        new_pass_index = waypoint_to_pass_index(waypoint_index)
        if new_pass_index != self._pass_index:
            self.get_logger().info(
                f"PassIndex changed: waypoint={waypoint_index}, "
                f"0x{self._pass_index:02X} -> 0x{new_pass_index:02X}"
            )
        self._current_waypoint_index = waypoint_index
        self._pass_index = new_pass_index

    def _route_completion_callback(self, message):
        completed = bool(message.data)
        # Completion is a one-way safety latch for the lifetime of this bridge.
        # A tracker restart must not clear it and unexpectedly re-enable drive.
        if self._route_completed is True and not completed:
            self.get_logger().warning(
                "Ignoring route completion reset; restart the bridge for a new run"
            )
            return
        if completed:
            # Clear steering freshness as a second, independent drive gate.
            self._last_steering_time = None
        if completed != self._route_completed:
            self.get_logger().info(f"Route completion changed: {completed}")
        self._route_completed = completed

    def _heading_valid_callback(self, message):
        valid = bool(message.data)
        if valid != self._heading_valid:
            self.get_logger().info(f"Heading validity changed: {valid}")
        self._heading_valid = valid

    def _rtk_fixed_callback(self, message):
        fixed = bool(message.data)
        if fixed != self._rtk_fixed:
            self.get_logger().info(
                f"RTK Float/Fixed drive gate changed: {fixed}"
            )
        self._rtk_fixed = fixed

    def _route_alignment_callback(self, message):
        ready = bool(message.data)
        if ready != self._route_alignment_ready:
            self.get_logger().info(f"Route alignment changed: {ready}")
        self._route_alignment_ready = ready
        self._last_route_alignment_time = time.monotonic()

    def _route_cross_track_callback(self, message):
        error_m = float(message.data)
        if not math.isfinite(error_m) or error_m < 0.0:
            self._route_cross_track_error_m = None
            self._last_route_cross_track_time = None
            return
        self._route_cross_track_error_m = error_m
        self._last_route_cross_track_time = time.monotonic()

    def _latch_safety_fault(self, reason):
        if self._safety_fault_latched:
            return
        self._safety_fault_latched = True
        self._safety_fault_reason = str(reason)
        self.get_logger().error(
            f"SAFETY FAULT LATCHED: {self._safety_fault_reason}; "
            "restart bridge only after inspection"
        )

    def _reset_motion_state(self, reset_centering=True):
        self._limited_speed_kph = 0.0
        self._limited_steering_deg = 0.0
        self._last_limiter_time = None
        self._active_started_at = None
        self._steering_error_started_at = None
        self._desired_speed_kph = 0.0
        self._desired_steering_deg = 0.0
        self._drive_mode = "INACTIVE"
        self._drive_promotion_ready_since = None
        if reset_centering:
            self._steering_centered_since = None
            self._steering_centered_ready = False

    def _inactive_state(self, reason, reset_centering=True):
        self._reset_motion_state(reset_centering=reset_centering)
        return False, 0.0, 0.0, REQUEST_INACTIVE, str(reason)

    def _evaluate_feedback_safety(self, feedback, now):
        sequence = int(feedback["sequence"])
        if sequence != self._last_feedback_sequence:
            self._last_feedback_sequence = sequence
            self._last_feedback_sequence_change_time = now
        self._latest_feedback = feedback
        self._last_feedback_time = now

        if feedback["estop_active"]:
            self._latch_safety_fault("AURIX 0x130 E-stop flag active")

        monitor_tracking = (
            self._last_sent_enabled
            and feedback["pc_control_active"]
            and feedback["steering_valid"]
        )
        if not monitor_tracking:
            self._steering_error_started_at = None
            return

        tracking_error = abs(
            float(feedback["steering_deg"])
            - float(self._last_sent_steering_deg)
        )
        error_limit = float(
            self.get_parameter("max_steering_tracking_error_deg").value
        )
        error_timeout = float(
            self.get_parameter("steering_error_timeout_sec").value
        )
        if tracking_error > error_limit:
            if self._steering_error_started_at is None:
                self._steering_error_started_at = now
            elif now - self._steering_error_started_at >= error_timeout:
                self._latch_safety_fault(
                    "steering tracking error "
                    f"{tracking_error:.1f}deg > {error_limit:.1f}deg "
                    f"for {now - self._steering_error_started_at:.2f}s"
                )
        else:
            self._steering_error_started_at = None

    def _handle_vm_status(self, state, owner, fault):
        """React to AURIX 0x131 state before the next command is selected."""
        state = int(state)
        owner = int(owner)
        fault = int(fault)

        failed_retry = (
            state == 5
            and self._vm_rearm_in_progress
            and self._last_vm_state != 5
            and self._rearm_frames_remaining == 0
            and self._last_sent_enabled
        )
        if state == 5 and (not self._vm_rearm_in_progress or failed_retry):
            # Uploaded firmware commissions 0x314 on the first ACTIVE frame,
            # then requires a newer ACTIVE only after an observed INACTIVE
            # phase. Perform that handshake automatically and only a bounded
            # number of times.
            self._vm_rearm_attempts += 1
            if self._vm_rearm_attempts > 2:
                self._latch_safety_fault(
                    "AURIX remained in CanRearm after two handshakes "
                    f"(fault=0x{fault:02X})"
                )
            else:
                self._vm_rearm_in_progress = True
                self._rearm_frames_remaining = self._startup_rearm_frames
                self._reset_motion_state()
                self.get_logger().warning(
                    "AURIX CanRearm detected; sending "
                    f"{self._startup_rearm_frames} inactive frames before "
                    f"retry {self._vm_rearm_attempts}/2"
                )
        elif state == 4 and owner == 2:
            if self._vm_rearm_in_progress:
                self.get_logger().info(
                    "AURIX rearm handshake complete: AutoCAN owner=CAN"
                )
            self._vm_rearm_in_progress = False
            self._vm_rearm_attempts = 0
        elif state == 3 or owner == 1:
            self._latch_safety_fault("PS2 owns AURIX vehicle control")
        elif state in (6, 7):
            self._latch_safety_fault(
                f"AURIX VM safety state={state}, fault=0x{fault:02X}"
            )

        self._last_vm_state = state

    def _close_can_socket(self):
        if self._can_socket is not None:
            try:
                self._can_socket.close()
            except OSError:
                pass
            self._can_socket = None

    def _ensure_can_socket(self, now):
        if self._can_socket is not None:
            return True
        if now - self._last_connect_attempt < 1.0:
            return False

        self._last_connect_attempt = now
        can_interface = str(self.get_parameter("can_interface").value)
        try:
            can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # Receive only feedback/diagnostic frames used by this bridge.
            # Without kernel filters, draining a busy bus can delay the 20 Hz
            # command timer and violate the AURIX 150 ms watchdog.
            filter_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
            filters = b"".join(
                struct.pack("=II", frame_id, filter_mask)
                for frame_id in (
                    int(self.get_parameter("feedback_can_id").value),
                    *AURIX_DIAGNOSTIC_CAN_IDS,
                )
            )
            can_socket.setsockopt(
                getattr(socket, "SOL_CAN_RAW", 101),
                getattr(socket, "CAN_RAW_FILTER", 1),
                filters,
            )
            can_socket.bind((can_interface,))
            can_socket.setblocking(False)
            self._can_socket = can_socket
            # Every physical CAN (re)connection must prove an inactive phase
            # before an active command is allowed onto the bus.
            self._rearm_frames_remaining = self._startup_rearm_frames
            self.get_logger().info(f"SocketCAN connected: {can_interface}")
            return True
        except OSError as exc:
            self._close_can_socket()
            self.get_logger().error(f"SocketCAN connection failed: {exc}")
            return False

    def _command_state(self, now):
        enable_requested = bool(self.get_parameter("enable_drive").value)
        command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )

        if self._safety_fault_latched:
            return self._inactive_state(
                f"safety fault latched: {self._safety_fault_reason}"
            )
        if self._route_completed is None:
            return self._inactive_state("no route completion status")
        if self._route_completed:
            return self._inactive_state("route completed")
        if not enable_requested:
            return self._inactive_state("enable_drive=false")
        if self._rearm_frames_remaining > 0:
            return self._inactive_state(
                f"startup rearm ({self._rearm_frames_remaining} frames remaining)"
            )
        if self._rtk_fixed is not True:
            return self._inactive_state("RTK Float or Fixed required")
        if self._route_alignment_ready is None:
            return self._inactive_state("no route alignment status")
        if self._last_route_alignment_time is None:
            return self._inactive_state("no fresh route alignment state")
        if self._last_route_cross_track_time is None:
            return self._inactive_state("no fresh route cross-track state")
        route_state_timeout = float(
            self.get_parameter("route_state_timeout_sec").value
        )
        alignment_age = now - self._last_route_alignment_time
        cross_track_age = now - self._last_route_cross_track_time
        if alignment_age > route_state_timeout:
            return self._inactive_state(
                f"route alignment stale ({alignment_age:.3f}s)"
            )
        if cross_track_age > route_state_timeout:
            return self._inactive_state(
                f"route cross-track stale ({cross_track_age:.3f}s)"
            )
        if self._last_steering_time is None:
            return self._inactive_state("no steering command")

        command_age = now - self._last_steering_time
        if command_age > command_timeout_sec:
            return self._inactive_state(
                f"steering stale ({command_age:.3f}s)"
            )
        if self._heading_valid is None:
            return self._inactive_state("no heading validity status")
        if self._current_waypoint_index < 0 or self._pass_index == 0xFF:
            return self._inactive_state("no valid waypoint/pass index")

        dry_run = bool(self.get_parameter("dry_run").value)
        feedback = self._latest_feedback
        feedback_age = None
        if self._last_feedback_time is not None:
            feedback_age = now - self._last_feedback_time
        if not dry_run:
            feedback_timeout = float(
                self.get_parameter("feedback_timeout_sec").value
            )
            if feedback is None or feedback_age is None:
                return self._inactive_state("no AURIX 0x130 feedback")
            if feedback_age > feedback_timeout:
                return self._inactive_state(
                    f"AURIX feedback stale ({feedback_age:.3f}s)"
                )
            sequence_age = (
                None
                if self._last_feedback_sequence_change_time is None
                else now - self._last_feedback_sequence_change_time
            )
            if sequence_age is None or sequence_age > feedback_timeout:
                return self._inactive_state(
                    "AURIX feedback sequence stale "
                    f"({0.0 if sequence_age is None else sequence_age:.3f}s)"
                )
            if not feedback["speed_valid"] or not feedback["steering_valid"]:
                return self._inactive_state("AURIX feedback validity false")
            if feedback["estop_active"]:
                self._latch_safety_fault("AURIX 0x130 E-stop flag active")
                return self._inactive_state(
                    f"safety fault latched: {self._safety_fault_reason}"
                )

        feedback_steering_deg = (
            float(feedback["steering_deg"])
            if feedback is not None and feedback.get("steering_valid")
            else 0.0
        )
        center_tolerance = float(
            self.get_parameter("steering_center_tolerance_deg").value
        )
        center_hold = float(
            self.get_parameter("steering_center_hold_sec").value
        )

        # Keep sending INACTIVE until AURIX centering has physically brought
        # the wheels near zero and held them there. This prevents the first
        # crawl metre from starting with the wheels still turned.
        if not self._last_sent_enabled and self._active_started_at is None:
            if dry_run:
                self._steering_centered_ready = True
            elif abs(feedback_steering_deg) <= center_tolerance:
                if self._steering_centered_since is None:
                    self._steering_centered_since = now
                self._steering_centered_ready = (
                    now - self._steering_centered_since >= center_hold
                )
            else:
                self._steering_centered_since = None
                self._steering_centered_ready = False

            if not self._steering_centered_ready:
                held = (
                    0.0
                    if self._steering_centered_since is None
                    else now - self._steering_centered_since
                )
                return self._inactive_state(
                    "waiting for steering center "
                    f"(actual={feedback_steering_deg:.1f}deg, "
                    f"hold={held:.2f}/{center_hold:.2f}s)",
                    reset_centering=False,
                )

            self._limited_speed_kph = 0.0
            self._limited_steering_deg = feedback_steering_deg
            self._last_limiter_time = now
            self._active_started_at = now

        pc_control_active = bool(
            dry_run
            or (feedback is not None and feedback["pc_control_active"])
        )
        waiting_for_pc = not pc_control_active
        if waiting_for_pc:
            if self._active_started_at is None:
                self._active_started_at = now
            pc_timeout = float(
                self.get_parameter("pc_active_timeout_sec").value
            )
            if now - self._active_started_at > pc_timeout:
                self._latch_safety_fault(
                    "AURIX did not enter PC control within "
                    f"{pc_timeout:.2f}s"
                )
                return self._inactive_state(
                    f"safety fault latched: {self._safety_fault_reason}"
                )

        requested_steering_deg = self._latest_steering_deg
        if bool(self.get_parameter("invert_steering").value):
            requested_steering_deg = -requested_steering_deg
        self._tracker_requested_steering_deg = requested_steering_deg
        requested_steering_deg = clamp(
            requested_steering_deg,
            -float(
                self.get_parameter("maximum_tracker_steering_deg").value
            ),
            float(
                self.get_parameter("maximum_tracker_steering_deg").value
            ),
        )
        requested_steering_deg *= float(
            self.get_parameter("steering_command_scale").value
        )
        self._alignment_steering_lag_deg = abs(
            requested_steering_deg - self._limited_steering_deg
        )
        self._alignment_tracking_error_deg = abs(
            requested_steering_deg - feedback_steering_deg
        )

        requested_speed_kph = float(
            self.get_parameter("target_speed_kph").value
        )
        if not math.isfinite(requested_speed_kph):
            return self._inactive_state("target_speed_kph is non-finite")
        maximum_speed_kph = float(
            self.get_parameter("maximum_speed_kph").value
        )
        requested_speed_kph = clamp(
            requested_speed_kph,
            0.0,
            maximum_speed_kph,
        )

        if waiting_for_pc:
            self._drive_promotion_ready_since = None
            self._drive_mode = "WAIT_PC_ACTIVE"
            self._desired_speed_kph = 0.0
            self._desired_steering_deg = 0.0
            reason = "active speed=0; waiting for AURIX PC control"
        elif not self._heading_valid:
            self._drive_promotion_ready_since = None
            self._drive_mode = "CRAWL"
            self._desired_speed_kph = min(
                requested_speed_kph,
                float(self.get_parameter("crawl_speed_kph").value),
            )
            self._desired_steering_deg = 0.0
            reason = "heading baseline pending; zero-steer crawl"
        else:
            self._desired_steering_deg = requested_steering_deg
            maximum_tracker_steering_deg = float(
                self.get_parameter("maximum_tracker_steering_deg").value
            )
            maximum_alignment_lag = float(
                self.get_parameter(
                    "maximum_alignment_steering_lag_deg"
                ).value
            )
            maximum_alignment_tracking_error = float(
                self.get_parameter(
                    "maximum_alignment_tracking_error_deg"
                ).value
            )
            maximum_cross_track = float(
                self.get_parameter("maximum_route_cross_track_m").value
            )
            geometry_ready = (
                self._route_alignment_ready
                and self._route_cross_track_error_m is not None
                and self._route_cross_track_error_m <= maximum_cross_track
            )
            steering_ready = (
                self._alignment_steering_lag_deg <= maximum_alignment_lag
                and self._alignment_tracking_error_deg
                <= maximum_alignment_tracking_error
            )
            turn_ready = (
                abs(self._tracker_requested_steering_deg)
                <= maximum_tracker_steering_deg
            )
            combined_ready = geometry_ready and steering_ready and turn_ready
            if combined_ready:
                if self._drive_promotion_ready_since is None:
                    self._drive_promotion_ready_since = now
            else:
                self._drive_promotion_ready_since = None
            promotion_hold = float(
                self.get_parameter("drive_promotion_hold_sec").value
            )
            promotion_ready = (
                self._drive_promotion_ready_since is not None
                and now - self._drive_promotion_ready_since >= promotion_hold
            )

            if not geometry_ready:
                self._drive_mode = "COURSE_ALIGNMENT"
                crawl_speed = float(
                    self.get_parameter("crawl_speed_kph").value
                )
                self._desired_speed_kph = (
                    0.0
                    if self._limited_speed_kph > crawl_speed + 0.1
                    else min(requested_speed_kph, crawl_speed)
                )
                reason = (
                    "route alignment pending; crawl or stop before acceleration "
                    f"(cross-track={self._route_cross_track_error_m})"
                )
            elif not steering_ready:
                self._drive_mode = "STEERING_SETTLE"
                crawl_speed = float(
                    self.get_parameter("crawl_speed_kph").value
                )
                self._desired_speed_kph = (
                    0.0
                    if self._limited_speed_kph > crawl_speed + 0.1
                    else min(requested_speed_kph, crawl_speed)
                )
                reason = (
                    "steering not settled; crawl-speed limit "
                    f"(command lag={self._alignment_steering_lag_deg:.1f}deg, "
                    f"tracking={self._alignment_tracking_error_deg:.1f}deg)"
                )
            elif not turn_ready:
                self._drive_mode = "SATURATED_TURN"
                self._desired_speed_kph = min(
                    requested_speed_kph,
                    float(
                        self.get_parameter(
                            "saturated_steering_speed_kph"
                        ).value
                    ),
                )
                reason = (
                    "steering request saturated; crawl-speed course alignment"
                )
            elif not promotion_ready:
                self._drive_mode = "ALIGNMENT_HOLD"
                crawl_speed = float(
                    self.get_parameter("crawl_speed_kph").value
                )
                self._desired_speed_kph = (
                    0.0
                    if self._limited_speed_kph > crawl_speed + 0.1
                    else min(requested_speed_kph, crawl_speed)
                )
                held = now - self._drive_promotion_ready_since
                reason = (
                    "combined route/steering alignment hold "
                    f"{held:.2f}/{promotion_hold:.2f}s"
                )
            else:
                self._drive_mode = "NORMAL"
                self._desired_speed_kph = requested_speed_kph
                reason = "heading valid; rate-limited command"

        elapsed = 0.0
        if self._last_limiter_time is not None:
            elapsed = clamp(now - self._last_limiter_time, 0.0, 0.20)
        self._last_limiter_time = now

        self._limited_speed_kph = ramp_up_or_drop(
            self._limited_speed_kph,
            self._desired_speed_kph,
            float(self.get_parameter("speed_ramp_kph_per_sec").value),
            elapsed,
        )
        self._limited_steering_deg = move_towards(
            self._limited_steering_deg,
            self._desired_steering_deg,
            float(
                self.get_parameter("max_steering_rate_deg_per_sec").value
            )
            * elapsed,
        )

        return (
            True,
            self._limited_steering_deg,
            self._limited_speed_kph,
            REQUEST_ACTIVE_VALID,
            reason,
        )

    def _publish_tx_debug(self, payload, state):
        payload_message = UInt8MultiArray()
        payload_message.data = list(payload)
        self._tx_payload_publisher.publish(payload_message)

        status_message = String()
        status_message.data = json.dumps(state, ensure_ascii=False, sort_keys=True)
        self._tx_status_publisher.publish(status_message)

    def _publish_feedback(self, feedback):
        speed_message = Float32()
        speed_message.data = float(feedback["speed_kph"])
        self._feedback_speed_publisher.publish(speed_message)

        steering_message = Float32()
        steering_message.data = float(feedback["steering_deg"])
        self._feedback_steering_publisher.publish(steering_message)

        adc_message = UInt16()
        adc_message.data = int(feedback["steering_adc_raw"])
        self._feedback_adc_publisher.publish(adc_message)

        status_message = String()
        status_message.data = json.dumps(
            feedback,
            ensure_ascii=False,
            sort_keys=True,
        )
        self._feedback_status_publisher.publish(status_message)

    def _drain_feedback(self, now):
        if self._can_socket is None:
            return

        feedback_can_id = int(self.get_parameter("feedback_can_id").value)
        while True:
            try:
                frame = self._can_socket.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f"SocketCAN receive failed: {exc}")
                self._close_can_socket()
                return

            if len(frame) != CAN_FRAME_SIZE:
                continue

            raw_can_id, dlc, frame_data = struct.unpack(CAN_FRAME_FORMAT, frame)
            if raw_can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
                continue

            can_id = raw_can_id & CAN_SFF_MASK
            if dlc != CAN_DLC:
                continue

            if can_id in AURIX_DIAGNOSTIC_CAN_IDS:
                diagnostic = {
                    "payload_hex": frame_data[:dlc].hex(" "),
                    "age_sec": 0.0,
                    "received_monotonic": now,
                }
                if can_id == 0x131:
                    state_names = {
                        0: "Init",
                        1: "Standby",
                        2: "Centering",
                        3: "ManualPS2",
                        4: "AutoCAN",
                        5: "CanRearm",
                        6: "FaultTorqueOff",
                        7: "Estop",
                    }
                    owner_names = {0: "None", 1: "PS2", 2: "CAN"}
                    diagnostic.update(
                        {
                            "state": frame_data[0],
                            "state_name": state_names.get(
                                frame_data[0], "Unknown"
                            ),
                            "owner": frame_data[1],
                            "owner_name": owner_names.get(
                                frame_data[1], "Unknown"
                            ),
                            "fault": frame_data[2],
                            "flags": frame_data[6],
                            "sequence": frame_data[7],
                        }
                    )
                    self._handle_vm_status(
                        frame_data[0],
                        frame_data[1],
                        frame_data[2],
                    )
                self._latest_diagnostic_frames[f"0x{can_id:03X}"] = diagnostic
                continue

            if can_id != feedback_can_id:
                continue

            try:
                feedback = decode_aurix_feedback_payload(frame_data[:dlc])
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                continue
            self._evaluate_feedback_safety(feedback, now)
            self._publish_feedback(feedback)

    def _send_timer_callback(self):
        now = time.monotonic()
        dry_run = bool(self.get_parameter("dry_run").value)

        if dry_run and not self._last_dry_run:
            self._close_can_socket()
        self._last_dry_run = dry_run

        transport_ready = dry_run
        if not dry_run:
            transport_ready = self._ensure_can_socket(now)
            if transport_ready:
                # Evaluate the newest AURIX state before deciding this tick's
                # command, reducing fault response by one 50 ms period.
                self._drain_feedback(now)

        enabled, steering_deg, speed_kph, request_status, reason = (
            self._command_state(now)
        )
        can_id = int(self.get_parameter("can_id").value)
        payload = build_nominal_driving_payload(
            steering_deg=steering_deg,
            speed_kph=speed_kph,
            pass_index=self._pass_index,
            request_status=request_status,
            sequence=self._sequence,
            can_id=can_id,
        )
        decoded = decode_nominal_driving_payload(payload)

        feedback_age = (
            None
            if self._last_feedback_time is None
            else max(0.0, now - self._last_feedback_time)
        )
        feedback_sequence_age = (
            None
            if self._last_feedback_sequence_change_time is None
            else max(0.0, now - self._last_feedback_sequence_change_time)
        )
        tracking_error_deg = None
        if self._latest_feedback is not None:
            tracking_error_deg = abs(
                float(self._latest_feedback["steering_deg"])
                - float(self._last_sent_steering_deg)
            )
        tracking_error_duration_sec = (
            0.0
            if self._steering_error_started_at is None
            else max(0.0, now - self._steering_error_started_at)
        )
        diagnostic_frames = {}
        for frame_id, diagnostic in self._latest_diagnostic_frames.items():
            diagnostic_copy = {
                key: value
                for key, value in diagnostic.items()
                if key != "received_monotonic"
            }
            diagnostic_copy["age_sec"] = max(
                0.0,
                now - diagnostic["received_monotonic"],
            )
            diagnostic_frames[frame_id] = diagnostic_copy

        state = {
            **decoded,
            "can_id": can_id,
            "current_waypoint_index": self._current_waypoint_index,
            "route_completed": self._route_completed,
            "heading_valid": self._heading_valid,
            "rtk_fixed": self._rtk_fixed,
            "rtk_float_or_fixed": self._rtk_fixed,
            "route_alignment_ready": self._route_alignment_ready,
            "route_cross_track_error_m": self._route_cross_track_error_m,
            "dry_run": dry_run,
            "enabled": enabled,
            "drive_mode": self._drive_mode,
            "desired_speed_kph": self._desired_speed_kph,
            "desired_steering_deg": self._desired_steering_deg,
            "tracker_requested_steering_deg": (
                self._tracker_requested_steering_deg
            ),
            "alignment_steering_lag_deg": (
                self._alignment_steering_lag_deg
            ),
            "alignment_tracking_error_deg": (
                self._alignment_tracking_error_deg
            ),
            "feedback_age_sec": feedback_age,
            "feedback_sequence_age_sec": feedback_sequence_age,
            "feedback_steering_deg": (
                None
                if self._latest_feedback is None
                else self._latest_feedback["steering_deg"]
            ),
            "feedback_pc_control_active": (
                None
                if self._latest_feedback is None
                else self._latest_feedback["pc_control_active"]
            ),
            "steering_tracking_error_deg": tracking_error_deg,
            "steering_error_duration_sec": tracking_error_duration_sec,
            "safety_fault_latched": self._safety_fault_latched,
            "safety_fault_reason": self._safety_fault_reason,
            "steering_centered_ready": self._steering_centered_ready,
            "vm_rearm_in_progress": self._vm_rearm_in_progress,
            "vm_rearm_attempts": self._vm_rearm_attempts,
            "diagnostic_frames": diagnostic_frames,
            "rearm_frames_remaining": self._rearm_frames_remaining,
            "reason": reason,
            "payload_hex": payload.hex(" "),
        }
        self._publish_tx_debug(payload, state)

        sent = False
        if dry_run:
            sent = True
        elif transport_ready:
            try:
                frame = build_socketcan_frame(can_id, payload)
                sent_size = self._can_socket.send(frame)
                sent = sent_size == len(frame)
            except (OSError, ValueError) as exc:
                self.get_logger().error(f"SocketCAN send failed: {exc}")
                self._last_sent_enabled = False
                self._reset_motion_state()
                self._close_can_socket()

        if sent:
            # Track the quantized command that actually reached SocketCAN.
            self._last_sent_enabled = bool(enabled)
            self._last_sent_speed_kph = float(decoded["speed_kph"])
            self._last_sent_steering_deg = float(decoded["steering_deg"])
            if self._rearm_frames_remaining > 0:
                self._rearm_frames_remaining -= 1
            # PPT B6 has only 5 sequence bits: 0..31 then wrap to 0.
            self._sequence = (self._sequence + 1) & 0x1F

        if now - self._last_status_log_time >= 1.0:
            mode = "ACTIVE_VALID" if enabled else "INACTIVE"
            transport = "DRY-RUN" if dry_run else "SocketCAN"
            self.get_logger().info(
                f"{transport} {mode} 0x{can_id:03X} "
                f"WP={self._current_waypoint_index} Pass={self._pass_index} "
                f"[{payload.hex(' ')}] reason={reason}"
            )
            self._last_status_log_time = now

    def _send_shutdown_stop(self):
        if self._can_socket is None:
            return

        can_id = int(self.get_parameter("can_id").value)
        for _ in range(3):
            payload = build_nominal_driving_payload(
                steering_deg=0.0,
                speed_kph=0.0,
                pass_index=self._pass_index,
                request_status=REQUEST_INACTIVE,
                sequence=self._sequence,
                can_id=can_id,
            )
            try:
                self._can_socket.send(build_socketcan_frame(can_id, payload))
            except (OSError, ValueError):
                break
            self._sequence = (self._sequence + 1) & 0x1F
            time.sleep(0.01)

    def destroy_node(self):
        self._send_shutdown_stop()
        self._close_can_socket()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AurixCanBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
