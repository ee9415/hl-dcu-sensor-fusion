#!/usr/bin/env python3
"""RViz visualization sidecar for the GPS waypoint tracker."""

import csv
import math
import os
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker, MarkerArray


EARTH_RADIUS_M = 6378137.0


class WaypointRvizVisualizer(Node):
    """Show the configured route and live NavSatFix without changing control."""

    def __init__(self):
        super().__init__('waypoint_rviz_visualizer')

        self.declare_parameter(
            'csv_path',
            (
                '/home/jigu/ros2_ws/src/0730_gps_test/'
                'waypoint_csv/0730_waypoint_straight.csv'
            ),
        )
        self.declare_parameter('fix_topic', '/gps1/fix')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('min_waypoint_spacing_m', 2.8)
        self.declare_parameter('max_path_points', 5000)

        self.csv_path = os.path.expanduser(
            str(self.get_parameter('csv_path').value)
        )
        self.fix_topic = str(self.get_parameter('fix_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.min_waypoint_spacing_m = max(
            float(self.get_parameter('min_waypoint_spacing_m').value),
            0.0,
        )
        max_path_points = max(
            int(self.get_parameter('max_path_points').value),
            2,
        )

        raw_waypoints = self.load_waypoints(self.csv_path)
        if len(raw_waypoints) < 2:
            raise RuntimeError('At least two valid waypoints are required.')

        self.origin_lat = raw_waypoints[0][0]
        self.origin_lon = raw_waypoints[0][1]
        local_waypoints = [
            self.latlon_to_local(lat, lon)
            for lat, lon in raw_waypoints
        ]
        self.waypoints = self.reduce_waypoints(local_waypoints)

        self.current_xy = None
        self.heading_rad = None
        self.steering_deg = 0.0
        self.target_index = 0
        self.path_points = deque(maxlen=max_path_points)

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/jigu/rviz_markers',
            marker_qos,
        )
        self.path_pub = self.create_publisher(
            Path,
            '/jigu/gps_path',
            10,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.fix_sub = self.create_subscription(
            NavSatFix,
            self.fix_topic,
            self.on_fix,
            sensor_qos,
        )
        self.steering_sub = self.create_subscription(
            Float32,
            '/filtered_steering_angle1',
            self.on_steering,
            10,
        )
        self.target_sub = self.create_subscription(
            Int32,
            '/current_waypoint_index1',
            self.on_target_index,
            10,
        )
        self.static_timer = self.create_timer(1.0, self.publish_markers)

        self.publish_markers()
        self.get_logger().info(
            f'RViz visualizer ready | CSV={self.csv_path} | '
            f'waypoints={len(self.waypoints)} | GPS={self.fix_topic}'
        )

    def load_waypoints(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)

        result = []
        with open(path, newline='', encoding='utf-8-sig') as file:
            reader = csv.DictReader(file)
            if not reader.fieldnames:
                raise RuntimeError('CSV header is missing.')

            normalized = {
                str(field).strip().lower(): field
                for field in reader.fieldnames
            }
            lat_key = normalized.get('latitude') or normalized.get('lat')
            lon_key = normalized.get('longitude') or normalized.get('lon')
            if not lat_key or not lon_key:
                raise RuntimeError(
                    f'latitude/longitude columns not found: {reader.fieldnames}'
                )

            for row in reader:
                try:
                    lat = float(row[lat_key])
                    lon = float(row[lon_key])
                except (TypeError, ValueError):
                    continue
                if math.isfinite(lat) and math.isfinite(lon):
                    result.append((lat, lon))
        return result

    def latlon_to_local(self, lat, lon):
        lat0_rad = math.radians(self.origin_lat)
        x = (
            math.radians(lon - self.origin_lon)
            * EARTH_RADIUS_M
            * math.cos(lat0_rad)
        )
        y = math.radians(lat - self.origin_lat) * EARTH_RADIUS_M
        return x, y

    def reduce_waypoints(self, points):
        reduced = [points[0]]
        for point in points[1:]:
            if (
                math.hypot(
                    point[0] - reduced[-1][0],
                    point[1] - reduced[-1][1],
                )
                >= self.min_waypoint_spacing_m
            ):
                reduced.append(point)
        return reduced

    def on_fix(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
        ):
            return

        x, y = self.latlon_to_local(msg.latitude, msg.longitude)
        if self.current_xy is not None:
            dx = x - self.current_xy[0]
            dy = y - self.current_xy[1]
            if math.hypot(dx, dy) >= 0.05:
                self.heading_rad = math.atan2(dy, dx)

        self.current_xy = (x, y)
        self.path_points.append((x, y))
        self.publish_path()
        self.publish_markers()

    def on_steering(self, msg):
        self.steering_deg = float(msg.data)

    def on_target_index(self, msg):
        self.target_index = max(0, int(msg.data))

    @staticmethod
    def point(x, y, z=0.0):
        return Point(x=float(x), y=float(y), z=float(z))

    @staticmethod
    def set_color(marker, red, green, blue, alpha=1.0):
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)

    def marker(self, namespace, marker_id, marker_type, stamp):
        result = Marker()
        result.header.frame_id = self.frame_id
        result.header.stamp = stamp
        result.ns = namespace
        result.id = marker_id
        result.type = marker_type
        result.action = Marker.ADD
        result.pose.orientation.w = 1.0
        return result

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        output = MarkerArray()

        route = self.marker('route', 0, Marker.LINE_STRIP, stamp)
        route.scale.x = 0.15
        self.set_color(route, 0.1, 0.45, 1.0)
        route.points = [
            self.point(x, y, 0.04) for x, y in self.waypoints
        ]
        output.markers.append(route)

        points = self.marker('waypoints', 0, Marker.SPHERE_LIST, stamp)
        points.scale.x = 0.40
        points.scale.y = 0.40
        points.scale.z = 0.40
        self.set_color(points, 0.1, 0.7, 1.0)
        points.points = [
            self.point(x, y, 0.20) for x, y in self.waypoints
        ]
        output.markers.append(points)

        if self.waypoints:
            target_index = min(self.target_index, len(self.waypoints) - 1)
            target_x, target_y = self.waypoints[target_index]
            target = self.marker('target', 0, Marker.SPHERE, stamp)
            target.pose.position.x = target_x
            target.pose.position.y = target_y
            target.pose.position.z = 0.30
            target.scale.x = 0.70
            target.scale.y = 0.70
            target.scale.z = 0.70
            self.set_color(target, 1.0, 0.0, 1.0)
            output.markers.append(target)

        if self.current_xy is not None:
            x, y = self.current_xy
            vehicle = self.marker('vehicle', 0, Marker.CYLINDER, stamp)
            vehicle.pose.position.x = x
            vehicle.pose.position.y = y
            vehicle.pose.position.z = 0.20
            vehicle.scale.x = 0.75
            vehicle.scale.y = 0.75
            vehicle.scale.z = 0.40
            self.set_color(vehicle, 1.0, 0.15, 0.1)
            output.markers.append(vehicle)

            if self.heading_rad is not None:
                heading = self.marker('heading', 0, Marker.ARROW, stamp)
                heading.scale.x = 0.12
                heading.scale.y = 0.28
                heading.scale.z = 0.35
                self.set_color(heading, 0.1, 1.0, 0.2)
                heading.points = [
                    self.point(x, y, 0.45),
                    self.point(
                        x + 2.0 * math.cos(self.heading_rad),
                        y + 2.0 * math.sin(self.heading_rad),
                        0.45,
                    ),
                ]
                output.markers.append(heading)

                steering_rad = (
                    self.heading_rad + math.radians(self.steering_deg)
                )
                steering = self.marker('steering', 0, Marker.ARROW, stamp)
                steering.scale.x = 0.10
                steering.scale.y = 0.24
                steering.scale.z = 0.30
                self.set_color(steering, 1.0, 0.8, 0.0)
                steering.points = [
                    self.point(x, y, 0.55),
                    self.point(
                        x + 1.7 * math.cos(steering_rad),
                        y + 1.7 * math.sin(steering_rad),
                        0.55,
                    ),
                ]
                output.markers.append(steering)

        self.marker_pub.publish(output)

    def publish_path(self):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp

        for x, y in self.path_points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

