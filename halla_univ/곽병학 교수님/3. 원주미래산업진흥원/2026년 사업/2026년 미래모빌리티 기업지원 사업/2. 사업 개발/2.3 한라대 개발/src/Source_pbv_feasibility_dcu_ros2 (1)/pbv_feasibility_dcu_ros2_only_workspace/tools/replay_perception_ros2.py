#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import time

import rclpy
from pbv_dcu_interfaces.msg import PerceptionSnapshot
from rclpy.node import Node


class ReplayNode(Node):
    def __init__(self, rows: list[dict[str, str]], fps: float) -> None:
        super().__init__('replay_perception_ros2')
        self._rows = rows
        self._period = max(0.02, 1.0 / max(1e-3, fps))
        self._seq = 0
        self._index = 0
        self._pub = self.create_publisher(PerceptionSnapshot, '/pbv/perception/snapshot', 10)
        self._timer = self.create_timer(self._period, self._on_timer)
        self.get_logger().info(f'replay_perception_ros2 started: rows={len(rows)} fps={fps:.1f}')

    def _on_timer(self) -> None:
        if not self._rows:
            return
        row = self._rows[self._index % len(self._rows)]
        self._index += 1
        self._seq += 1
        now = self.get_clock().now().to_msg()
        msg = PerceptionSnapshot()
        msg.stamp = now
        msg.seq = self._seq
        msg.lane_offset_m = float(row['lane_offset_m'])
        msg.lane_heading_rad = float(row['lane_heading_rad'])
        msg.lane_confidence = float(row['lane_confidence'])
        msg.target_distance_m = float(row['target_distance_m'])
        msg.stop_line_distance_m = float(row['stop_line_distance_m'])
        msg.traffic_light_confidence = float(row['traffic_light_confidence'])
        msg.traffic_light_state = int(row['traffic_light_state'])
        msg.frame_fps = float(row.get('frame_fps', 5.0))
        msg.frame_age_ms = 0
        msg.camera_ok = bool(int(row.get('camera_ok', '1')))
        msg.lane_valid = bool(int(row.get('lane_valid', '1')))
        msg.target_valid = bool(int(row.get('target_valid', '1')))
        msg.flags = int(row.get('flags', '0'))
        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description='Replay PBV perception CSV to ROS 2 topic')
    parser.add_argument('--csv', required=True, help='path to perception_replay.csv')
    parser.add_argument('--fps', type=float, default=5.0)
    args = parser.parse_args()

    with open(args.csv, 'r', newline='') as f:
        rows = list(csv.DictReader(f))

    rclpy.init()
    node = ReplayNode(rows, args.fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
