"""Display host postprocessing using reliable ROS subscriptions (WSLg)."""
import time
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def main():
    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE',
        str(Path(__file__).with_name('ros_fastdds_large_images.xml')))
    rclpy.init()
    node = rclpy.create_node('oak_reliable_result_viewer')
    pending = {}
    subscriptions = []
    counts = {}
    start = time.monotonic()
    reported = False
    for topic, title, width, height in (
        ('/oak/postprocess/debug_image', 'ROS Postprocess', 960, 540),
        ('/oak/postprocess/top_image', 'ROS TOP', 700, 450),
    ):
        counts[title] = 0

        def receive(message, title=title):
            if message.encoding not in ('bgr8', 'rgb8'):
                return
            array = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            array = array[:, :message.width*3].reshape(message.height, message.width, 3)
            if message.encoding == 'rgb8':
                array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            pending[title] = array.copy()
            counts[title] += 1

        subscriptions.append(node.create_subscription(Image, topic, receive,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)))
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, width, height)
    print('Reliable ROS viewer started', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            # Draw each newly received image once, rather than repainting the
            # same large buffers on every event-loop iteration.
            for title, image in pending.items():
                cv2.imshow(title, image)
            pending.clear()
            if cv2.waitKey(1) & 255 == 27:
                break
            if not reported and time.monotonic()-start >= 10:
                print('First 10 seconds: '+str(counts), flush=True)
                reported = True
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
