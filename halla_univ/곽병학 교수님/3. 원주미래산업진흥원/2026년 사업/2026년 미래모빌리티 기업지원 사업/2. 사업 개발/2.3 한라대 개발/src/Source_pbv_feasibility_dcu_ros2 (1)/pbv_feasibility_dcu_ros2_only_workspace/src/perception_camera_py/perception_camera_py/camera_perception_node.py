#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from pbv_dcu_interfaces.msg import PerceptionSnapshot
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image

TRAFFIC_LIGHT_UNKNOWN = 0
TRAFFIC_LIGHT_RED = 1
TRAFFIC_LIGHT_YELLOW = 2
TRAFFIC_LIGHT_GREEN = 3


@dataclass
class LaneResult:
    valid: bool
    offset_m: float
    heading_rad: float
    confidence: float
    overlay: np.ndarray


@dataclass
class StopLineResult:
    distance_m: float
    overlay: np.ndarray


@dataclass
class TrafficLightResult:
    state: int
    confidence: float
    overlay: np.ndarray


@dataclass
class TargetProxyResult:
    valid: bool
    distance_m: float
    overlay: np.ndarray


class CameraPerceptionNode(Node):
    """
    Feasibility perception node.

    - Captures camera frames with OpenCV.
    - Computes lane / stop-line / traffic-light / target proxy features.
    - Publishes ROS 2 message at 5 FPS by default.
    """

    def __init__(self) -> None:
        super().__init__("camera_perception_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("target_fps", 5.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("lane_roi_top_ratio", 0.55)
        self.declare_parameter("stopline_roi_top_ratio", 0.72)
        self.declare_parameter("meters_per_half_frame", 0.9)
        self.declare_parameter("simulate_no_camera", False)

        self._camera_index = int(self.get_parameter("camera_index").value)
        self._frame_width = int(self.get_parameter("frame_width").value)
        self._frame_height = int(self.get_parameter("frame_height").value)
        self._target_fps = float(self.get_parameter("target_fps").value)
        self._publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self._lane_roi_top_ratio = float(self.get_parameter("lane_roi_top_ratio").value)
        self._stopline_roi_top_ratio = float(self.get_parameter("stopline_roi_top_ratio").value)
        self._meters_per_half_frame = float(self.get_parameter("meters_per_half_frame").value)
        self._simulate_no_camera = bool(self.get_parameter("simulate_no_camera").value)

        self.add_on_set_parameters_callback(self._on_params_changed)

        self._snapshot_pub = self.create_publisher(PerceptionSnapshot, "/pbv/perception/snapshot", 10)
        self._debug_pub = self.create_publisher(Image, "/pbv/perception/debug", 2)


        self._cap: Optional[cv2.VideoCapture] = None
        if not self._simulate_no_camera:
            self._cap = cv2.VideoCapture(self._camera_index)
            if self._cap is not None and self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)
            else:
                self.get_logger().warn("Camera open failed. Falling back to synthetic frame mode.")
                self._cap = None
                self._simulate_no_camera = True

        self._seq = 0
        self._last_loop_t = time.perf_counter()
        self._period_s = max(0.02, 1.0 / max(1e-3, self._target_fps))
        self._timer = self.create_timer(self._period_s, self._on_timer)

        self.get_logger().info(
            f"camera_perception_node started: camera_index={self._camera_index}, "
            f"frame={self._frame_width}x{self._frame_height}, target_fps={self._target_fps:.1f}"
        )

    def destroy_node(self) -> bool:
        if self._cap is not None:
            self._cap.release()
        return super().destroy_node()

    def _on_params_changed(self, params):
        for p in params:
            if p.name == "target_fps" and p.value is not None:
                fps = float(p.value)
                if fps < 1.0 or fps > 30.0:
                    return SetParametersResult(successful=False, reason="target_fps must be in [1, 30]")
        return SetParametersResult(successful=True)

    def _make_synthetic_frame(self) -> np.ndarray:
        frame = np.zeros((self._frame_height, self._frame_width, 3), dtype=np.uint8)
        frame[:] = (30, 30, 30)
        h, w = frame.shape[:2]

        # Lane markers
        cv2.line(frame, (int(w * 0.28), h), (int(w * 0.46), int(h * 0.58)), (255, 255, 255), 6)
        cv2.line(frame, (int(w * 0.72), h), (int(w * 0.54), int(h * 0.58)), (255, 255, 255), 6)

        # Stop line
        cv2.line(frame, (int(w * 0.32), int(h * 0.82)), (int(w * 0.68), int(h * 0.82)), (240, 240, 240), 8)

        # Traffic light (toggle every ~2 s)
        phase = int(time.time() / 2.0) % 3
        colors = [(0, 0, 90), (0, 90, 90), (0, 90, 0)]
        c = colors[phase]
        cv2.circle(frame, (int(w * 0.86), int(h * 0.18)), 18, c, -1)

        # Obstacle proxy
        cv2.rectangle(frame, (int(w * 0.43), int(h * 0.60)), (int(w * 0.57), int(h * 0.76)), (60, 60, 60), -1)
        return frame

    def _capture_frame(self) -> Tuple[np.ndarray, bool]:
        if self._simulate_no_camera or self._cap is None:
            return self._make_synthetic_frame(), True

        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Camera frame read failed. Using synthetic frame for this cycle.")
            return self._make_synthetic_frame(), False

        frame = cv2.resize(frame, (self._frame_width, self._frame_height))
        return frame, True

    def _detect_lane(self, frame: np.ndarray) -> LaneResult:
        h, w = frame.shape[:2]
        roi_top = int(h * self._lane_roi_top_ratio)
        roi = frame[roi_top:h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 80, 160)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=50,
            minLineLength=40,
            maxLineGap=30,
        )

        overlay = frame.copy()
        left_x, right_x = [], []
        left_angles, right_angles = [], []

        if lines is not None:
            for line in lines[:, 0]:
                x1, y1, x2, y2 = map(int, line)
                dx = x2 - x1
                dy = y2 - y1
                if abs(dx) < 5:
                    continue
                slope = dy / float(dx)
                angle = math.atan2(dy, dx)
                if abs(slope) < 0.35:
                    continue

                x1_full, y1_full = x1, y1 + roi_top
                x2_full, y2_full = x2, y2 + roi_top
                cv2.line(overlay, (x1_full, y1_full), (x2_full, y2_full), (0, 150, 255), 2)

                if slope < 0.0:
                    left_x.extend([x1, x2])
                    left_angles.append(angle)
                else:
                    right_x.extend([x1, x2])
                    right_angles.append(angle)

        valid = len(left_x) >= 2 and len(right_x) >= 2
        offset_m = 0.0
        heading_rad = 0.0
        confidence = 0.0

        if valid:
            lane_center_px = (float(np.mean(left_x)) + float(np.mean(right_x))) * 0.5
            img_center_px = w * 0.5
            px_offset = lane_center_px - img_center_px
            offset_m = (px_offset / (w * 0.5)) * self._meters_per_half_frame
            left_mean = float(np.mean(left_angles))
            right_mean = float(np.mean(right_angles))
            heading_rad = 0.5 * (left_mean + right_mean)
            coverage = min(1.0, (len(left_x) + len(right_x)) / 40.0)
            confidence = max(0.0, min(1.0, 0.55 + 0.45 * coverage))
            cv2.line(overlay, (int(lane_center_px), h), (int(lane_center_px), roi_top), (255, 0, 255), 2)
        else:
            confidence = 0.15 if lines is not None else 0.0

        cv2.putText(
            overlay,
            f"lane valid={int(valid)} offset={offset_m:+.2f} m conf={confidence:.2f}",
            (16, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return LaneResult(valid=valid, offset_m=offset_m, heading_rad=heading_rad, confidence=confidence, overlay=overlay)

    def _detect_stop_line(self, frame: np.ndarray) -> StopLineResult:
        h, w = frame.shape[:2]
        roi_top = int(h * self._stopline_roi_top_ratio)
        roi = frame[roi_top:h, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)

        horizontal_proj = np.sum(bw > 0, axis=1)
        best_row = int(np.argmax(horizontal_proj))
        pixels = int(horizontal_proj[best_row])

        overlay = frame.copy()
        distance_m = -1.0

        if pixels > int(w * 0.22):
            y = roi_top + best_row
            cv2.line(overlay, (0, y), (w - 1, y), (255, 180, 0), 2)
            normalized = (h - y) / max(1.0, h - roi_top)
            distance_m = max(0.0, normalized * 2.5)

        cv2.putText(
            overlay,
            f"stop_line_distance={distance_m:.2f} m",
            (16, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return StopLineResult(distance_m=distance_m, overlay=overlay)

    def _detect_traffic_light(self, frame: np.ndarray) -> TrafficLightResult:
        h, w = frame.shape[:2]
        x0, y0 = int(w * 0.72), int(h * 0.02)
        x1, y1 = int(w * 0.98), int(h * 0.32)
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        red2 = cv2.inRange(hsv, (160, 80, 80), (179, 255, 255))
        yellow = cv2.inRange(hsv, (18, 80, 80), (38, 255, 255))
        green = cv2.inRange(hsv, (40, 60, 60), (90, 255, 255))

        red_score = int(np.count_nonzero(red1 | red2))
        yellow_score = int(np.count_nonzero(yellow))
        green_score = int(np.count_nonzero(green))
        scores = {
            TRAFFIC_LIGHT_RED: red_score,
            TRAFFIC_LIGHT_YELLOW: yellow_score,
            TRAFFIC_LIGHT_GREEN: green_score,
        }

        state = TRAFFIC_LIGHT_UNKNOWN
        confidence = 0.0
        overlay = frame.copy()

        best_state = max(scores, key=scores.get)
        best_score = scores[best_state]
        total = red_score + yellow_score + green_score

        if best_score > 25 and total > 0:
            state = best_state
            confidence = float(best_score) / float(total)

        cv2.rectangle(overlay, (x0, y0), (x1, y1), (120, 120, 120), 1)
        state_name = {
            TRAFFIC_LIGHT_UNKNOWN: "UNKNOWN",
            TRAFFIC_LIGHT_RED: "RED",
            TRAFFIC_LIGHT_YELLOW: "YELLOW",
            TRAFFIC_LIGHT_GREEN: "GREEN",
        }[state]
        cv2.putText(
            overlay,
            f"traffic_light={state_name} conf={confidence:.2f}",
            (16, 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return TrafficLightResult(state=state, confidence=confidence, overlay=overlay)

    def _detect_target_proxy(self, frame: np.ndarray) -> TargetProxyResult:
        h, w = frame.shape[:2]
        x0, y0 = int(w * 0.32), int(h * 0.44)
        x1, y1 = int(w * 0.68), int(h * 0.90)

        roi = frame[y0:y1, x0:x1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, bw = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        overlay = frame.copy()
        valid = False
        distance_m = -1.0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(largest))
            if area > 1200.0:
                x, y, ww, hh = cv2.boundingRect(largest)
                cv2.rectangle(overlay, (x + x0, y + y0), (x + x0 + ww, y + y0 + hh), (0, 255, 0), 2)
                # Proxy: larger object height = closer object.
                distance_m = max(0.5, 7.5 - (hh / max(1.0, h * 0.45)) * 6.5)
                valid = True

        cv2.putText(
            overlay,
            f"target_valid={int(valid)} target_distance={distance_m:.2f} m",
            (16, 96),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return TargetProxyResult(valid=valid, distance_m=distance_m, overlay=overlay)

    def _make_debug_image(self, base: np.ndarray, texts: list[str]) -> np.ndarray:
        out = base.copy()
        y = 122
        for text in texts:
            cv2.putText(out, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            y += 24
        return out

    def _to_ros_time(self, stamp_sec: float) -> Time:
        sec = int(stamp_sec)
        nanosec = int((stamp_sec - sec) * 1e9)
        msg = Time()
        msg.sec = sec
        msg.nanosec = nanosec
        return msg

    def _to_image_msg(self, frame: np.ndarray, stamp: Time) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_front"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        return msg

    def _on_timer(self) -> None:
        loop_t = time.perf_counter()
        now_t = time.time()
        frame, camera_ok = self._capture_frame()

        lane = self._detect_lane(frame)
        stop_line = self._detect_stop_line(lane.overlay)
        tl = self._detect_traffic_light(stop_line.overlay)
        target = self._detect_target_proxy(tl.overlay)

        dt = max(1e-3, loop_t - self._last_loop_t)
        self._last_loop_t = loop_t
        fps = 1.0 / dt
        self._seq += 1
        stamp_ms = int(now_t * 1000.0)

        msg = PerceptionSnapshot()
        msg.stamp = self._to_ros_time(now_t)
        msg.seq = self._seq
        msg.lane_offset_m = float(lane.offset_m)
        msg.lane_heading_rad = float(lane.heading_rad)
        msg.lane_confidence = float(lane.confidence)
        msg.target_distance_m = float(target.distance_m)
        msg.stop_line_distance_m = float(stop_line.distance_m)
        msg.traffic_light_confidence = float(tl.confidence)
        msg.frame_fps = float(fps)
        msg.frame_age_ms = 0
        msg.camera_ok = bool(camera_ok)
        msg.lane_valid = bool(lane.valid)
        msg.target_valid = bool(target.valid)
        msg.traffic_light_state = int(tl.state)
        msg.flags = 0

        self._snapshot_pub.publish(msg)

        if self._publish_debug_image and self._debug_pub.get_subscription_count() > 0:
            overlay = self._make_debug_image(
                target.overlay,
                [
                    f"seq={self._seq}",
                    f"camera_ok={int(camera_ok)} fps={fps:.1f}",
                    f"lane_offset={lane.offset_m:+.2f} m lane_heading={lane.heading_rad:+.2f} rad",
                ],
            )
            self._debug_pub.publish(self._to_image_msg(overlay, msg.stamp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
