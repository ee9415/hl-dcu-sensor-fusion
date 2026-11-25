"""Run only in an isolated ROS domain. No SocketCAN device is opened."""
import copy
import json
import os
from pathlib import Path
import time
import unittest

import test_core as fixtures
ROOT = fixtures.ROOT

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from hl_oak_interfaces.msg import CanFdFrame, TargetPath
    from hl_camera_can.nodes import CameraPathNode, CanBridgeNode, CanDriverNode, QOS
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'Build/source the camera_can overlay first')
class RosPipelineTests(unittest.TestCase):
    def setUp(self):
        cfg, self.example = fixtures.GeometryTests().example()
        cfg['geometry'].update(calibrated=False, timing_verified=False, bench_test_active=True)
        cfg['can']['allow_bench_active'] = True
        work = ROOT/'artifacts/camera_can/tests'
        work.mkdir(parents=True, exist_ok=True)
        path = work/'bench_config.json'
        path.write_text(json.dumps(cfg), encoding='utf-8')
        rclpy.init(args=['--ros-args', '-p', 'config_path:='+str(path)])
        self.camera, self.bridge, self.driver = CameraPathNode(), CanBridgeNode(), CanDriverNode()
        self.observer = Node('can_pipeline_test')
        self.executor = SingleThreadedExecutor()
        for node in (self.camera, self.bridge, self.driver, self.observer):
            self.executor.add_node(node)
        self.frames, self.paths = [], []
        self.pub = self.observer.create_publisher(String, '/oak/postprocess/result', QOS)
        self.sub = self.observer.create_subscription(CanFdFrame, '/can/tx',
            lambda msg: self.frames.append((time.monotonic(), bytes(msg.data))), QOS)
        self.path_sub = self.observer.create_subscription(TargetPath, '/camera/target_path', self.paths.append, QOS)
        self.sequence = 0
        self.spin(.3)

    def tearDown(self):
        self.executor.shutdown()
        for node in (self.camera, self.bridge, self.driver, self.observer):node.destroy_node()
        rclpy.shutdown()

    def spin(self, duration):
        until = time.monotonic()+duration
        while time.monotonic() < until:self.executor.spin_once(timeout_sec=.005)

    def publish(self, valid=True):
        result = copy.deepcopy(self.example)
        ns = self.observer.get_clock().now().nanoseconds
        result['stamp'] = dict(zip(('sec', 'nanosec'), divmod(ns, 10**9)))
        result['frame_seq'] = self.sequence
        result['boundary_pair_valid'] = valid
        self.sequence += 1
        self.pub.publish(String(data=json.dumps(result)))
        return result

    def test_real_ros_publisher_bridge_20hz_expiry_and_failure(self):
        start = time.monotonic()
        for _ in range(12):
            last = self.publish()
            self.spin(.1)
        self.assertTrue(any(data[0] == 0xd9 for _, data in self.frames))
        self.assertTrue(self.paths[-1].bench_only)
        self.assertFalse(self.paths[-1].calibrated)
        self.assertFalse(self.driver.enabled)
        self.assertIsNone(self.driver.driver)
        # A dead camera, including duplicate messages, must stop ACTIVE output.
        for _ in range(5):
            self.pub.publish(String(data=json.dumps(last)))
            self.spin(.1)
        self.assertTrue(all(data[0] == 0x81 for _, data in self.frames[-3:]))
        # Two new observations are insufficient; the third reactivates.
        for _ in range(2):
            self.publish(); self.spin(.1)
            self.assertEqual(self.frames[-1][1][0], 0x81)
        self.publish(); self.spin(.1)
        self.assertEqual(self.frames[-1][1][0], 0xd9)
        self.publish(valid=False); self.spin(.1)
        self.assertEqual(self.frames[-1][1][0], 0x81)
        sequences = [data[1] for _, data in self.frames]
        self.assertTrue(all((b-a) % 256 == 1 for a, b in zip(sequences, sequences[1:])))
        rate = (len(self.frames)-1)/(self.frames[-1][0]-self.frames[0][0])
        self.assertTrue(18 < rate < 22, rate)
        self.assertEqual(self.driver.sent, 0)
        self.assertEqual(self.driver.rejected, 0)
        report = dict(frames=len(self.frames), measured_hz=rate,
            physical_can_writes=self.driver.sent, driver_rejected=self.driver.rejected,
            active_frames=sum(data[0] == 0xd9 for _, data in self.frames),
            duration_s=time.monotonic()-start, expiry_and_reactivation_verified=True)
        (ROOT/'artifacts/camera_can/tests/ros_report.json').write_text(json.dumps(report, indent=2))

    def test_default_uncalibrated_candidate_remains_inactive(self):
        self.camera.config['geometry'].update(bench_test_active=False)
        for _ in range(6):self.publish(); self.spin(.1)
        self.assertTrue(self.paths)
        self.assertFalse(any(msg.valid for msg in self.paths))
        self.assertTrue(all(data[0] == 0x81 for _, data in self.frames))

    def test_driver_rejects_expired_handoff_and_path_deadline(self):
        from hl_camera_can.protocol import packet
        message = CanFdFrame()
        message.header.stamp = self.observer.get_clock().now().to_msg()
        message.can_id, message.is_fd, message.bitrate_switch, message.length = 0x320, True, True, 32
        message.data = list(packet(1, True, (0.,)*4, 3., 80))
        message.created_monotonic_ns = time.monotonic_ns()-500_000_000
        message.valid_until_monotonic_ns = time.monotonic_ns()+50_000_000
        self.driver.receive(message)
        self.assertEqual(self.driver.rejected, 1)
        message.created_monotonic_ns = time.monotonic_ns()
        message.valid_until_monotonic_ns = time.monotonic_ns()-1
        self.driver.receive(message)
        self.assertEqual(self.driver.rejected, 2)
        self.assertEqual(self.driver.sent, 0)

    @unittest.skipUnless(os.environ.get('OAK_TEST_WINDOWS_HOST'), 'Optional Windows Kvaser virtual service')
    def test_windows_kvaser_virtual_driver_from_real_ros_topics(self):
        from hl_camera_can.windows_transport import WindowsCanClient
        self.driver.driver = WindowsCanClient(os.environ['OAK_TEST_WINDOWS_HOST'], 53200,
            ROOT/'artifacts/camera_can/windows.key', allow_virtual=True)
        self.driver.enabled = True
        for _ in range(12):self.publish(); self.spin(.1)
        self.assertGreaterEqual(self.driver.sent, 10)
        self.assertEqual(self.driver.errors, 0)
        self.assertEqual(self.driver.rejected, 0)
        self.assertTrue(any(data[0] == 0xd9 for _, data in self.frames))
        (ROOT/'artifacts/camera_can/tests/windows_ros_report.json').write_text(json.dumps(dict(
            ros_frames=len(self.frames), windows_virtual_writes=self.driver.sent,
            errors=self.driver.errors, physical_leaf_opened=False), indent=2))


if __name__ == '__main__':
    unittest.main()
