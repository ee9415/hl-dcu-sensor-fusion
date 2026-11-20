#!/usr/bin/env python3
"""Record GPS tracking, CAN command, and AURIX feedback data to CSV."""

import csv
from datetime import datetime
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int32, String


EARTH_RADIUS = 6378137.0

CSV_FIELDS = [
    "row_index",
    "record_time_iso",
    "record_time_unix_sec",
    "elapsed_sec",
    "gps_age_sec",
    "gps_stamp_sec",
    "gps_stamp_nanosec",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "gps_status",
    "gps_service",
    "covariance_xx",
    "covariance_yy",
    "covariance_zz",
    "web_mercator_x_m",
    "web_mercator_y_m",
    "waypoint_index",
    "waypoint_index_age_sec",
    "tracker_steering_deg",
    "tracker_steering_age_sec",
    "can_mode",
    "can_reason",
    "can_drive_mode",
    "heading_valid",
    "desired_speed_kph",
    "desired_steering_deg",
    "steering_centered_ready",
    "steering_tracking_error_deg",
    "steering_error_duration_sec",
    "safety_fault_latched",
    "safety_fault_reason",
    "aurix_vm_state",
    "aurix_vm_state_name",
    "aurix_vm_owner",
    "aurix_vm_owner_name",
    "aurix_vm_fault",
    "can_dry_run",
    "can_id_hex",
    "target_speed_kph",
    "aurix_command_steering_deg",
    "steering_pulse_us",
    "can_sequence",
    "can_signature_hex",
    "can_payload_hex",
    "can_status_age_sec",
    "feedback_speed_kph",
    "feedback_steering_deg",
    "feedback_steering_adc_raw",
    "feedback_sequence",
    "feedback_speed_valid",
    "feedback_steering_valid",
    "feedback_pc_control_active",
    "feedback_estop_active",
    "feedback_age_sec",
]


def lat_lon_to_meters(latitude_deg, longitude_deg):
    """Convert WGS84 degrees to the same Web Mercator used by the tracker."""
    x = EARTH_RADIUS * float(longitude_deg) * math.pi / 180.0
    y = EARTH_RADIUS * math.log(
        math.tan((90.0 + float(latitude_deg)) * math.pi / 360.0)
    )
    return x, y


def parse_json_object(text):
    """Parse a ROS String JSON payload and require an object."""
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError("JSON status message must contain an object")
    return value


def finite_or_blank(value):
    if value is None:
        return ""
    numeric = float(value)
    return numeric if math.isfinite(numeric) else ""


def bool_or_blank(value):
    if value is None:
        return ""
    return int(bool(value))


def age_or_blank(received_monotonic, now_monotonic):
    if received_monotonic is None:
        return ""
    return max(0.0, now_monotonic - received_monotonic)


def int_hex_or_blank(value, width):
    if value is None or value == "":
        return ""
    return f"0x{int(value):0{width}X}"


class DriveCsvLogger(Node):
    """Write a synchronized 20 Hz snapshot of the latest driving data."""

    def __init__(self):
        super().__init__("drive_csv_logger")

        self.declare_parameter("fix_topic", "/gps1/fix")
        self.declare_parameter(
            "steering_topic",
            "/filtered_steering_angle1",
        )
        self.declare_parameter(
            "waypoint_index_topic",
            "/current_waypoint_index1",
        )
        self.declare_parameter(
            "can_tx_status_topic",
            "/aurix/can_tx_status",
        )
        self.declare_parameter(
            "feedback_status_topic",
            "/aurix/feedback/status",
        )
        self.declare_parameter(
            "log_directory",
            os.path.expanduser("~/ros2_ws/gps_logs"),
        )
        self.declare_parameter("file_prefix", "gps_drive_log")
        self.declare_parameter("log_period_sec", 0.05)
        self.declare_parameter("flush_period_sec", 1.0)
        self.declare_parameter("require_gps", True)

        log_period_sec = float(
            self.get_parameter("log_period_sec").value
        )
        flush_period_sec = float(
            self.get_parameter("flush_period_sec").value
        )
        if log_period_sec <= 0.0:
            raise ValueError("log_period_sec must be positive")
        if flush_period_sec <= 0.0:
            raise ValueError("flush_period_sec must be positive")

        log_directory = os.path.abspath(
            os.path.expanduser(
                str(self.get_parameter("log_directory").value)
            )
        )
        file_prefix = str(self.get_parameter("file_prefix").value)
        os.makedirs(log_directory, exist_ok=True)

        start_datetime = datetime.now().astimezone()
        timestamp = start_datetime.strftime("%Y%m%d_%H%M%S")
        filename = (
            f"{file_prefix}_{timestamp}_pid{os.getpid()}.csv"
        )
        self.log_path = os.path.join(log_directory, filename)

        self._csv_file = open(
            self.log_path,
            "x",
            encoding="utf-8-sig",
            newline="",
            buffering=1,
        )
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=CSV_FIELDS,
            extrasaction="ignore",
        )
        self._writer.writeheader()

        self._start_monotonic = time.monotonic()
        self._last_flush_monotonic = self._start_monotonic
        self._flush_period_sec = flush_period_sec
        self._row_index = 0
        self._closed = False
        self._last_json_warning_monotonic = 0.0

        self._gps = None
        self._gps_received_monotonic = None
        self._waypoint_index = 0
        self._waypoint_received_monotonic = None
        self._tracker_steering_deg = None
        self._steering_received_monotonic = None
        self._can_status = {}
        self._can_status_received_monotonic = None
        self._feedback_status = {}
        self._feedback_received_monotonic = None

        fix_topic = str(self.get_parameter("fix_topic").value)
        steering_topic = str(
            self.get_parameter("steering_topic").value
        )
        waypoint_index_topic = str(
            self.get_parameter("waypoint_index_topic").value
        )
        can_tx_status_topic = str(
            self.get_parameter("can_tx_status_topic").value
        )
        feedback_status_topic = str(
            self.get_parameter("feedback_status_topic").value
        )

        self._gps_subscription = self.create_subscription(
            NavSatFix,
            fix_topic,
            self._gps_callback,
            qos_profile_sensor_data,
        )
        self._steering_subscription = self.create_subscription(
            Float32,
            steering_topic,
            self._steering_callback,
            10,
        )
        waypoint_qos = QoSProfile(depth=10)
        waypoint_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._waypoint_subscription = self.create_subscription(
            Int32,
            waypoint_index_topic,
            self._waypoint_callback,
            waypoint_qos,
        )
        self._can_status_subscription = self.create_subscription(
            String,
            can_tx_status_topic,
            self._can_status_callback,
            20,
        )
        self._feedback_status_subscription = self.create_subscription(
            String,
            feedback_status_topic,
            self._feedback_status_callback,
            20,
        )

        self._timer = self.create_timer(
            log_period_sec,
            self._timer_callback,
        )

        self.get_logger().info(
            f"Drive CSV logging enabled: {self.log_path}"
        )
        self.get_logger().info(
            f"CSV period={log_period_sec:.3f}s, "
            f"flush period={flush_period_sec:.3f}s, "
            f"fix_topic={fix_topic}"
        )

    def _gps_callback(self, message):
        now = time.monotonic()
        covariance = list(message.position_covariance)
        x_m = ""
        y_m = ""
        if (
            math.isfinite(float(message.latitude))
            and math.isfinite(float(message.longitude))
            and -89.999999 < float(message.latitude) < 89.999999
        ):
            x_m, y_m = lat_lon_to_meters(
                message.latitude,
                message.longitude,
            )

        self._gps = {
            "gps_stamp_sec": int(message.header.stamp.sec),
            "gps_stamp_nanosec": int(message.header.stamp.nanosec),
            "latitude_deg": finite_or_blank(message.latitude),
            "longitude_deg": finite_or_blank(message.longitude),
            "altitude_m": finite_or_blank(message.altitude),
            "gps_status": int(message.status.status),
            "gps_service": int(message.status.service),
            "covariance_xx": finite_or_blank(covariance[0]),
            "covariance_yy": finite_or_blank(covariance[4]),
            "covariance_zz": finite_or_blank(covariance[8]),
            "web_mercator_x_m": x_m,
            "web_mercator_y_m": y_m,
        }
        self._gps_received_monotonic = now

    def _steering_callback(self, message):
        value = float(message.data)
        self._tracker_steering_deg = (
            value if math.isfinite(value) else None
        )
        self._steering_received_monotonic = time.monotonic()

    def _waypoint_callback(self, message):
        self._waypoint_index = int(message.data)
        self._waypoint_received_monotonic = time.monotonic()

    def _parse_status_message(self, message):
        try:
            return parse_json_object(message.data)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            now = time.monotonic()
            if now - self._last_json_warning_monotonic >= 1.0:
                self.get_logger().warning(
                    f"Ignoring invalid JSON status: {exc}"
                )
                self._last_json_warning_monotonic = now
            return None

    def _can_status_callback(self, message):
        value = self._parse_status_message(message)
        if value is not None:
            self._can_status = value
            self._can_status_received_monotonic = time.monotonic()

    def _feedback_status_callback(self, message):
        value = self._parse_status_message(message)
        if value is not None:
            self._feedback_status = value
            self._feedback_received_monotonic = time.monotonic()

    def _build_row(self, now_monotonic, now_wall_time):
        gps = self._gps or {}
        can_status = self._can_status
        feedback = self._feedback_status
        diagnostic_frames = can_status.get("diagnostic_frames", {})
        diagnostic_131 = (
            diagnostic_frames.get("0x131", {})
            if isinstance(diagnostic_frames, dict)
            else {}
        )

        enabled = can_status.get("enabled")
        can_mode = ""
        if enabled is not None:
            can_mode = "DRIVE" if bool(enabled) else "STOP"

        return {
            "row_index": self._row_index,
            "record_time_iso": datetime.fromtimestamp(
                now_wall_time
            ).astimezone().isoformat(timespec="milliseconds"),
            "record_time_unix_sec": f"{now_wall_time:.6f}",
            "elapsed_sec": (
                f"{now_monotonic - self._start_monotonic:.6f}"
            ),
            "gps_age_sec": age_or_blank(
                self._gps_received_monotonic,
                now_monotonic,
            ),
            **gps,
            "waypoint_index": self._waypoint_index,
            "waypoint_index_age_sec": age_or_blank(
                self._waypoint_received_monotonic,
                now_monotonic,
            ),
            "tracker_steering_deg": finite_or_blank(
                self._tracker_steering_deg
            ),
            "tracker_steering_age_sec": age_or_blank(
                self._steering_received_monotonic,
                now_monotonic,
            ),
            "can_mode": can_mode,
            "can_reason": can_status.get("reason", ""),
            "can_drive_mode": can_status.get("drive_mode", ""),
            "heading_valid": bool_or_blank(
                can_status.get("heading_valid")
            ),
            "desired_speed_kph": finite_or_blank(
                can_status.get("desired_speed_kph")
            ),
            "desired_steering_deg": finite_or_blank(
                can_status.get("desired_steering_deg")
            ),
            "steering_centered_ready": bool_or_blank(
                can_status.get("steering_centered_ready")
            ),
            "steering_tracking_error_deg": finite_or_blank(
                can_status.get("steering_tracking_error_deg")
            ),
            "steering_error_duration_sec": finite_or_blank(
                can_status.get("steering_error_duration_sec")
            ),
            "safety_fault_latched": bool_or_blank(
                can_status.get("safety_fault_latched")
            ),
            "safety_fault_reason": can_status.get(
                "safety_fault_reason", ""
            ),
            "aurix_vm_state": diagnostic_131.get("state", ""),
            "aurix_vm_state_name": diagnostic_131.get(
                "state_name", ""
            ),
            "aurix_vm_owner": diagnostic_131.get("owner", ""),
            "aurix_vm_owner_name": diagnostic_131.get(
                "owner_name", ""
            ),
            "aurix_vm_fault": diagnostic_131.get("fault", ""),
            "can_dry_run": bool_or_blank(
                can_status.get("dry_run")
            ),
            "can_id_hex": int_hex_or_blank(
                can_status.get("can_id"),
                3,
            ),
            "target_speed_kph": finite_or_blank(
                can_status.get("speed_kph")
            ),
            "aurix_command_steering_deg": finite_or_blank(
                can_status.get("steering_deg")
            ),
            "steering_pulse_us": can_status.get(
                "steering_pulse_us",
                "",
            ),
            "can_sequence": can_status.get("sequence", ""),
            "can_signature_hex": int_hex_or_blank(
                can_status.get("signature"),
                2,
            ),
            "can_payload_hex": can_status.get("payload_hex", ""),
            "can_status_age_sec": age_or_blank(
                self._can_status_received_monotonic,
                now_monotonic,
            ),
            "feedback_speed_kph": finite_or_blank(
                feedback.get("speed_kph")
            ),
            "feedback_steering_deg": finite_or_blank(
                feedback.get("steering_deg")
            ),
            "feedback_steering_adc_raw": feedback.get(
                "steering_adc_raw",
                "",
            ),
            "feedback_sequence": feedback.get("sequence", ""),
            "feedback_speed_valid": bool_or_blank(
                feedback.get("speed_valid")
            ),
            "feedback_steering_valid": bool_or_blank(
                feedback.get("steering_valid")
            ),
            "feedback_pc_control_active": bool_or_blank(
                feedback.get("pc_control_active")
            ),
            "feedback_estop_active": bool_or_blank(
                feedback.get("estop_active")
            ),
            "feedback_age_sec": age_or_blank(
                self._feedback_received_monotonic,
                now_monotonic,
            ),
        }

    def _timer_callback(self):
        if self._closed:
            return
        if bool(self.get_parameter("require_gps").value):
            if self._gps is None:
                return

        now_monotonic = time.monotonic()
        row = self._build_row(now_monotonic, time.time())
        self._writer.writerow(row)
        self._row_index += 1

        if (
            now_monotonic - self._last_flush_monotonic
            >= self._flush_period_sec
        ):
            self._csv_file.flush()
            self._last_flush_monotonic = now_monotonic

    def _close_csv(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._csv_file.flush()
            os.fsync(self._csv_file.fileno())
        except (OSError, KeyboardInterrupt):
            # A second SIGINT can arrive while launch is forwarding shutdown.
            # The file still needs to be closed even if fsync is interrupted.
            pass
        finally:
            try:
                self._csv_file.close()
            except (OSError, KeyboardInterrupt):
                pass
        if rclpy.ok(context=self.context):
            self.get_logger().info(
                f"Drive CSV closed: {self.log_path} "
                f"({self._row_index} data rows)"
            )

    def destroy_node(self):
        if hasattr(self, "_timer"):
            self._timer.cancel()
        self._close_csv()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DriveCsvLogger()
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
