"""ROS camera publisher -> CAN bridge subscriber -> optional SocketCAN driver."""
from dataclasses import fields
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from hl_oak_interfaces.msg import TargetPath, CanFdFrame

from .path import Target, load_config, target_from_result
from .bridge import PathGate
from .driver import SocketCanDriver
from .windows_transport import WindowsCanClient
from .protocol import CAN_ID, PERIOD_S, FRAME_ID, check_packet

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


def configuration(node):
    default = str(Path(get_package_share_directory('hl_camera_can'))/'config/camera_can.json')
    path = node.declare_parameter('config_path', default).value
    return load_config(path)


def ros_seconds(node):
    return node.get_clock().now().nanoseconds/1e9


def target_message(target):
    message = TargetPath()
    ns = round(target.capture_s*1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)
    message.header.frame_id = target.frame_id
    for field in fields(Target):
        if field.name not in ('capture_s', 'frame_id'):
            setattr(message, field.name, getattr(target, field.name))
    return message


def target_value(message):
    values = {field.name: getattr(message, field.name) for field in fields(Target)
              if field.name not in ('capture_s', 'frame_id')}
    return Target(capture_s=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
        frame_id=message.header.frame_id, **values)


class CameraPathNode(Node):
    def __init__(self):
        super().__init__('camera_path_publisher')
        self.config = configuration(self)
        self.publisher = self.create_publisher(TargetPath, 'camera/target_path', QOS)
        self.create_subscription(String, 'oak/postprocess/result', self.receive, QOS)
        self.get_logger().info('Target path frame=vehicle_front_axle; calibration/timing gate enabled')

    def receive(self, message):
        try:
            result = json.loads(message.data)
            target = target_from_result(result, self.config)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            target = Target(reason='malformed_perception: '+str(exc))
        self.publisher.publish(target_message(target))


class CanBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_can_bridge')
        self.config = configuration(self)
        self.gate = PathGate(self.config, time.monotonic())
        self.publisher = self.create_publisher(CanFdFrame, 'can/tx', QOS)
        self.status = self.create_publisher(String, 'camera/can_status', QOS)
        self.count = 0
        self.create_subscription(TargetPath, 'camera/target_path', self.receive, QOS)
        # This clock keeps the 50 ms heartbeat alive even when /clock stops.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(PERIOD_S, self.transmit, clock=self.steady_clock)
        self.create_timer(1., self.publish_status, clock=self.steady_clock)

    def receive(self, message):
        self.gate.update(target_value(message), ros_seconds(self), time.monotonic())

    def transmit(self):
        data = self.gate.tick(ros_seconds(self), time.monotonic())
        if data is None:
            return
        message = CanFdFrame()
        message.header.stamp = self.get_clock().now().to_msg()
        message.created_monotonic_ns = time.monotonic_ns()
        ttl = (self.gate.remaining_s(ros_seconds(self), time.monotonic()) if data[0] == 0xd9
               else self.config['can']['max_tx_age_ms']/1000)
        message.valid_until_monotonic_ns = message.created_monotonic_ns+int(ttl*1e9)
        message.header.frame_id = FRAME_ID
        message.can_id, message.is_extended = CAN_ID, False
        message.is_fd, message.bitrate_switch, message.length = True, True, 32
        message.data = list(data)
        self.publisher.publish(message)
        self.count += 1

    def publish_status(self):
        self.status.publish(String(data=json.dumps(dict(
            frames_published=self.count, next_sequence=self.gate.sequence,
            reason=self.gate.reason, consecutive_good_frames=self.gate.streak,
            driver_subscribers=self.publisher.get_subscription_count()))))


class CanDriverNode(Node):
    def __init__(self):
        super().__init__('camera_can_driver')
        self.config = configuration(self)
        self.enabled = self.declare_parameter('enabled', False).value
        self.driver = None
        self.sent, self.rejected, self.errors = 0, 0, 0
        self.captured = 0
        self.last_error = ''
        if self.enabled:
            c = self.config['can']
            if c['driver_backend'] == 'windows_tcp':
                if not c['windows_key_file']:
                    raise ValueError('Set windows_key_file before enabling Windows CAN')
                self.driver = WindowsCanClient(c['windows_host'], c['windows_port'],
                    c['windows_key_file'], c['windows_allow_virtual'], c['windows_monitor_only'])
            else:
                self.driver = SocketCanDriver(c['interface'], c['allow_vcan'])
        self.create_subscription(CanFdFrame, 'can/tx', self.receive, QOS)
        self.status = self.create_publisher(String, 'can/driver_status', QOS)
        self.create_timer(1., self.publish_status,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, message):
        try:
            if (message.can_id != CAN_ID or message.is_extended or not message.is_fd or
                    not message.bitrate_switch or message.length != 32):
                raise ValueError('CAN FD envelope mismatch')
            age = ros_seconds(self)-(message.header.stamp.sec+message.header.stamp.nanosec/1e9)
            local_age_ms = (time.monotonic_ns()-message.created_monotonic_ns)/1e6
            if (not 0 <= age*1000 < self.config['can']['max_tx_age_ms'] or
                    not 0 <= local_age_ms < self.config['can']['max_tx_age_ms']):
                raise ValueError('Queued CAN frame expired or timestamp in future')
            check_packet(message.data)
            if time.monotonic_ns() >= message.valid_until_monotonic_ns:
                raise ValueError('CAN path deadline expired before driver write')
            if not self.driver:
                return  # Disabled by default; ROS diagnostics only, no physical write.
            transmitted = self.driver.send(bytes(message.data), message.valid_until_monotonic_ns)
            if transmitted is False:
                self.captured += 1
            else:
                self.sent += 1
            self.last_error = ''
        except (ValueError, TypeError, OverflowError) as exc:
            self.rejected += 1
            self.last_error = str(exc)
        except OSError as exc:
            self.errors += 1
            self.last_error = str(exc)
            self.get_logger().error('CAN write failed: '+str(exc))

    def publish_status(self):
        self.status.publish(String(data=json.dumps(dict(enabled=self.enabled,
            backend=self.config['can']['driver_backend'],
            monitor_only=self.config['can']['windows_monitor_only'], captured=self.captured,
            sent=self.sent, rejected=self.rejected, errors=self.errors, error=self.last_error))))

    def destroy_node(self):
        if self.driver:
            self.driver.close()
        return super().destroy_node()


def run(node_type, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def camera_main(args=None):
    run(CameraPathNode, args)


def bridge_main(args=None):
    run(CanBridgeNode, args)


def driver_main(args=None):
    run(CanDriverNode, args)
