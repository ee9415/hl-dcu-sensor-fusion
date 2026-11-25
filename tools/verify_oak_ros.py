"""Verify real ROS frame/result/image delivery; optional synthetic transport fixture."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from hl_oak_interfaces.msg import PerceptionFrame
from hl_oak_perception.bridge import Bridge, QOS
from hl_oak_perception.processor import Processor
from hl_oak_perception.common import image_array


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--drop-last-seconds', type=float, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    synthetic = None
    if args.synthetic:
        os.environ['ROS_DOMAIN_ID'] = '91'  # Isolate fixture traffic from the live camera.
    rclpy.init(args=['--ros-args', '-p', 'device_ip:=127.0.0.1'] if args.synthetic else [])
    executor = rclpy.executors.SingleThreadedExecutor()
    nodes = []
    if args.synthetic:
        root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root/'apps/yolopv2_standalone'))
        sys.path.insert(0, str(root/'apps/yolopv2_standalone/tests'))
        from perception_server import PerceptionServer
        from test_ros_offload import fixture
        synthetic = PerceptionServer()
        metadata, arrays = fixture()
        metadata['frame_id'] = 'cam01_rgb_optical_frame'
        nodes = [Bridge(), Processor()]
        for node in nodes:
            executor.add_node(node)
    probe = Node('oak_offload_verification')
    executor.add_node(probe)
    counts = dict(frames=0, results=0, debug=0, top=0)
    report = dict(counts=counts, synthetic=args.synthetic)
    sequences = []
    errors = []
    def frame(message):
        counts['frames'] += 1
        sequences.append(message.sequence)
        metadata = json.loads(message.metadata_json)
        depth = image_array(message.depth)
        report['last_frame'] = dict(sequence=message.sequence, session=message.session,
            rgb_shape=list(image_array(message.rgb).shape), mask_shape=list(image_array(message.lane_mask).shape),
            depth_shape=list(depth.shape), depth_encoding=message.depth.encoding,
            valid_depth_pixels=int(np.count_nonzero(depth)), depth_available=metadata['depth_available'],
            sync_error_s=abs(metadata['capture_timestamp_s']-metadata['depth_timestamp_s']),
            camera_postprocess=metadata.get('camera_postprocess'),
            rgb_k=list(message.rgb_camera_info.k), depth_k=list(message.depth_camera_info.k),
            source=metadata['source'].get('input_source', 'camera'))
    def result(message):
        payload = json.loads(message.data)
        if not payload.get('stale', True):
            counts['results'] += 1
            report['last_result'] = payload
        else:
            errors.append(payload.get('reason'))
    def picture(name, message):
        counts[name] += 1
        if counts[name] == 3:
            cv2.imwrite(str(args.output/(name+'.png')), image_array(message))
    subscriptions = [
        probe.create_subscription(PerceptionFrame, 'oak/frame', frame, QOS),
        probe.create_subscription(String, 'oak/postprocess/result', result, 1),
        probe.create_subscription(Image, 'oak/postprocess/debug_image', lambda m: picture('debug', m), QOS),
        probe.create_subscription(Image, 'oak/postprocess/top_image', lambda m: picture('top', m), QOS)]
    start = time.monotonic()
    next_frame = start
    while time.monotonic()-start < args.seconds:
        now = time.monotonic()
        if synthetic and now >= next_frame and now-start < args.seconds-args.drop_last_seconds:
            index = int((now-start)*5)+1
            metadata.update(frame_seq=index, nn_seq=index, capture_timestamp_s=now,
                            nn_timestamp_s=now, depth_timestamp_s=now)
            synthetic.publish(metadata, arrays)
            next_frame = now+.2
        executor.spin_once(timeout_sec=.02)
    report.update(elapsed_s=round(time.monotonic()-start, 2),
                  unique_sequences=len(set(sequences)), stale_messages=len(errors),
                  passed=all(value >= 3 for value in counts.values()) and
                         (args.drop_last_seconds <= 0 or len(errors) > 0))
    (args.output/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({key:value for key,value in report.items() if key != 'last_result'}), flush=True)
    for node in nodes+[probe]:
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
