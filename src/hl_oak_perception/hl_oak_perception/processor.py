"""Run the existing contour tracker, stop line and guidance on the ROS host."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from hl_oak_interfaces.msg import PerceptionFrame

from .common import load_core, image_array, image_message, json_text
from .bridge import QOS

load_core()
from host_postprocess import HostPostprocess


class Processor(Node):
    def __init__(self):
        super().__init__('oak_host_postprocess')
        self.engine = HostPostprocess()
        self.last_frame = 0.
        self.last_identity = None
        self.last_header = None
        self.results = self.create_publisher(String, 'oak/postprocess/result', 64)
        self.timing = self.create_publisher(String, 'oak/postprocess/timing', 64)
        self.last_export = None
        self.sequence_gaps = 0
        self.ack = self.create_publisher(String, 'oak/frame_ack', 64)
        self.debug = self.create_publisher(Image, 'oak/postprocess/debug_image', QOS)
        self.top = self.create_publisher(Image, 'oak/postprocess/top_image', QOS)
        self.create_subscription(PerceptionFrame, 'oak/frame', self.process, QOS)
        self.create_timer(.5, self.watchdog)

    def unavailable(self, reason):
        self.results.publish(String(data=json_text(dict(stale=True, reason=reason,
            processing_location='ros', boundary_valid=False, guidance_valid=False,
            stop_line_valid=False, metric_path_available=False,
            last_frame=self.last_identity))))

    def watchdog(self):
        if time.monotonic()-self.last_frame > 2.:
            self.engine.reset()
            self.unavailable('camera_frame_timeout')

    def process(self, frame):
        callback_ns = time.monotonic_ns()
        try:
            metadata = json.loads(frame.metadata_json)
            identity = (frame.session, frame.sequence)
            if metadata['session'] != frame.session or metadata['frame_seq'] != frame.sequence:
                raise ValueError('Bundle identity mismatch')
            sequence = metadata.get('export_seq')
            if identity == self.last_identity:
                self.acknowledge(frame, sequence)
                return
            continuity_error = None
            if sequence is not None:
                if (self.last_export is not None and self.last_export[0] == frame.session and
                        sequence != self.last_export[1]+1):
                    continuity_error = dict(previous=self.last_export[1], current=sequence)
                    self.sequence_gaps += 1
                    self.engine.reset()
                    self.unavailable('ROS inference frame sequence gap: '+str(continuity_error))
                    self.get_logger().error('ROS inference frame sequence gap: '+str(continuity_error))
            arrays = dict(rgb=image_array(frame.rgb), drivable=image_array(frame.drivable_mask),
                          lane=image_array(frame.lane_mask), depth=image_array(frame.depth))
            result, debug, top = self.engine.process(metadata, arrays)
            processed_ns = time.monotonic_ns()
            result['stamp'] = dict(sec=frame.header.stamp.sec, nanosec=frame.header.stamp.nanosec)
            result['frame_id'] = frame.header.frame_id
            result.update(source=metadata['source'], export_seq=sequence,
                          objects=metadata['objects'], depth_available=metadata['depth_available'],
                          sequence_gaps=self.sequence_gaps, continuity_error=continuity_error)
            self.results.publish(String(data=json_text(result)))
            self.last_export = (frame.session, sequence) if sequence is not None else None
            published_ns = time.monotonic_ns()
            if self.timing.get_subscription_count():
                self.timing.publish(String(data=json_text(dict(
                    session=frame.session, frame_seq=frame.sequence,
                    source=metadata['source'],
                    host_bundle_received_monotonic_ns=metadata.get('host_bundle_received_monotonic_ns'),
                    callback_monotonic_ns=callback_ns, processed_monotonic_ns=processed_ns,
                    result_publish_returned_monotonic_ns=published_ns))))
            if self.debug.get_subscription_count():
                self.debug.publish(image_message(debug, 'bgr8', frame.header))
            if self.top.get_subscription_count():
                self.top.publish(image_message(top, 'bgr8', frame.header))
            self.last_frame, self.last_identity = time.monotonic(), identity
            self.acknowledge(frame, sequence)
        except (ValueError, TypeError, KeyError) as exc:
            self.engine.reset()
            self.unavailable(str(exc))
            self.get_logger().error(str(exc))

    def acknowledge(self, frame, sequence):
        self.ack.publish(String(data=json_text(dict(session=frame.session,
            frame_seq=frame.sequence, export_seq=sequence))))


def main(args=None):
    rclpy.init(args=args)
    node = Processor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
