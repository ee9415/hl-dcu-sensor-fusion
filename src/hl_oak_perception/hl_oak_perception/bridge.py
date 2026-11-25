"""Receive atomic OAK bundles, publish raw sensor topics and a synchronized frame."""
import json
import queue
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from hl_oak_interfaces.msg import PerceptionFrame

from .common import load_core, image_message, camera_info, json_text

load_core()
from perception_wire import receive_bundle
from host_postprocess import validate_frame

QOS = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)


class Bridge(Node):
    def __init__(self):
        super().__init__('oak_inference_bridge')
        self.ip = self.declare_parameter('device_ip', '169.254.100.228').value
        self.port = self.declare_parameter('port', 8560).value
        self.frames = queue.Queue(maxsize=8)
        self.stop = threading.Event()
        self.connection = None
        self.error = 'waiting_for_camera'
        self.received = 0.
        self.clock_key = None
        self.last_capture = None
        self.last_export = None
        self.received_count = 0
        self.published_count = 0
        self.queue_high_water = 0
        self.pending_frame = None
        self.pending_identity = None
        self.pending_sent = 0.
        self.retransmitted_count = 0
        self.acknowledged_count = 0
        self.frame_pub = self.create_publisher(PerceptionFrame, 'oak/frame', QOS)
        self.create_subscription(String, 'oak/frame_ack', self.acknowledge, 64)
        self.images = {name: self.create_publisher(Image, 'oak/'+name, QOS)
                       for name in ('rgb/image_raw', 'drivable_mask', 'lane_mask', 'depth/image_raw')}
        self.rgb_info = self.create_publisher(CameraInfo, 'oak/rgb/camera_info', QOS)
        self.depth_info = self.create_publisher(CameraInfo, 'oak/depth/camera_info', QOS)
        self.boxes = self.create_publisher(Detection2DArray, 'oak/detections', QOS)
        self.status = self.create_publisher(String, 'oak/bridge_status', 1)
        self.create_timer(.001, self.publish_next)
        self.create_timer(1., self.publish_status)
        self.worker = threading.Thread(target=self.receive, daemon=True)
        self.worker.start()

    def receive(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection((self.ip, self.port), timeout=3) as connection:
                    self.connection = connection
                    connection.settimeout(3)
                    connection.sendall(b'OAK_PERCEPTION_1\n')
                    while not self.stop.is_set():
                        metadata, arrays = receive_bundle(connection)
                        validate_frame(metadata, arrays)
                        item = (metadata, arrays, time.monotonic())
                        # Keep FIFO ordering. A full queue applies TCP
                        # backpressure; it must never replace an older result.
                        while not self.stop.is_set():
                            try:
                                self.frames.put(item, timeout=.1)
                                break
                            except queue.Full:
                                continue
                        self.received_count += 1
                        self.queue_high_water = max(self.queue_high_water, self.frames.qsize())
                        self.received = item[2]
                        self.error = ''
            except (OSError, EOFError, ValueError, KeyError, TypeError) as exc:
                self.error = str(exc)
                self.stop.wait(1.)
            finally:
                self.connection = None

    def publish_status(self):
        self.status.publish(String(data=json_text(dict(
            connected=not self.error and time.monotonic()-self.received < 2.,
            error=self.error, device_ip=self.ip, port=self.port,
            received=self.received_count, published=self.published_count,
            pending=self.frames.qsize(), queue_high_water=self.queue_high_water,
            awaiting_ack=self.pending_identity, acknowledged=self.acknowledged_count,
            retransmitted=self.retransmitted_count))))

    def acknowledge(self, message):
        try:
            value = json.loads(message.data)
            identity = (value['session'], value['frame_seq'], value['export_seq'])
        except (ValueError, KeyError, TypeError):
            return
        if identity == self.pending_identity:
            self.pending_frame = None
            self.pending_identity = None
            self.acknowledged_count += 1

    def publish_next(self):
        # Bound the DDS pipeline to one unacknowledged bundle. KEEP_LAST alone
        # can evict unread multi-megabyte frames even with RELIABLE delivery.
        if self.pending_frame is not None:
            if time.monotonic()-self.pending_sent >= .5:
                self.frame_pub.publish(self.pending_frame)
                self.pending_sent = time.monotonic()
                self.retransmitted_count += 1
            return
        if not self.frame_pub.get_subscription_count():
            return
        try:
            metadata, arrays, received = self.frames.get_nowait()
        except queue.Empty:
            return
        sequence = metadata.get('export_seq')
        if sequence is not None:
            if (self.last_export is not None and self.last_export[0] == metadata['session'] and
                    sequence != self.last_export[1]+1):
                raise RuntimeError('Camera export sequence gap: '+str((self.last_export, sequence)))
            self.last_export = (metadata['session'], sequence)
        source = metadata['source']
        key = (metadata['session'], source.get('source_epoch'), source.get('source_media_epoch'))
        capture = metadata['capture_timestamp_s']
        now = self.get_clock().now().nanoseconds
        if key != self.clock_key or self.last_capture is None or capture <= self.last_capture:
            self.clock_key = key
            self.clock_offset_ns = now-int(capture*1e9)
        self.last_capture = capture
        stamp_ns = self.clock_offset_ns+int(capture*1e9)
        header = Header(frame_id=metadata['frame_id'])
        header.stamp.sec, header.stamp.nanosec = divmod(stamp_ns, 10**9)
        metadata['timestamp_mapping'] = 'capture_clock_anchored_to_first_host_receive'
        metadata['host_receive_ros_ns'] = now
        metadata['host_bundle_received_monotonic_ns'] = int(received*1e9)
        frame = PerceptionFrame(header=header, session=metadata['session'], sequence=metadata['frame_seq'])
        frame.rgb = image_message(arrays['rgb'], 'bgr8', header)
        frame.drivable_mask = image_message(arrays['drivable'], 'mono8', header)
        frame.lane_mask = image_message(arrays['lane'], 'mono8', header)
        frame.depth = image_message(arrays['depth'], '16UC1', header)
        frame.rgb_camera_info = camera_info(arrays['rgb'], metadata['rgb_intrinsics'], header)
        frame.depth_camera_info = camera_info(arrays['depth'], metadata['depth_intrinsics'], header)
        frame.detections.header = header
        height, width = arrays['rgb'].shape[:2]
        for obj in metadata['objects']:
            detection = Detection2D(header=header)
            detection.bbox.center.position.x = (obj['xmin']+obj['xmax'])*width/2
            detection.bbox.center.position.y = (obj['ymin']+obj['ymax'])*height/2
            detection.bbox.size_x = (obj['xmax']-obj['xmin'])*width
            detection.bbox.size_y = (obj['ymax']-obj['ymin'])*height
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(obj['label'])
            hypothesis.hypothesis.score = float(obj['confidence'])
            detection.results = [hypothesis]
            frame.detections.detections.append(detection)
        frame.metadata_json = json_text(metadata)
        self.pending_frame = frame
        self.pending_identity = (frame.session, frame.sequence, sequence)
        self.pending_sent = time.monotonic()
        self.frame_pub.publish(frame)
        self.published_count += 1
        for name, value in zip(self.images, (frame.rgb, frame.drivable_mask, frame.lane_mask, frame.depth)):
            if self.images[name].get_subscription_count():
                self.images[name].publish(value)
        if self.rgb_info.get_subscription_count():
            self.rgb_info.publish(frame.rgb_camera_info)
        if self.depth_info.get_subscription_count():
            self.depth_info.publish(frame.depth_camera_info)
        if self.boxes.get_subscription_count():
            self.boxes.publish(frame.detections)

    def destroy_node(self):
        self.stop.set()
        connection = self.connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.worker.join(timeout=4)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
