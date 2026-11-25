"""Lossless bundle flow control and recovery at the ROS boundary."""
import json
import queue
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from std_msgs.msg import Header, String
from hl_oak_interfaces.msg import PerceptionFrame
from hl_oak_perception.bridge import Bridge
from hl_oak_perception.common import image_message
from hl_oak_perception.processor import Processor


class DeliveryTests(unittest.TestCase):
    def bridge(self):
        return SimpleNamespace(pending_frame=object(), pending_identity=('s', 10, 5),
            pending_sent=time.monotonic(), frame_pub=Mock(), frames=queue.Queue(),
            retransmitted_count=0, acknowledged_count=0)

    def test_unacknowledged_frame_blocks_dequeue_and_is_retried(self):
        node = self.bridge()
        node.frames.put('next frame')
        Bridge.publish_next(node)
        node.frame_pub.publish.assert_not_called()
        node.pending_sent -= 1
        Bridge.publish_next(node)
        node.frame_pub.publish.assert_called_once_with(node.pending_frame)
        self.assertEqual(node.frames.qsize(), 1)
        self.assertEqual(node.retransmitted_count, 1)

    def test_only_matching_ack_releases_frame_and_duplicate_ack_is_ignored(self):
        node = self.bridge()
        for payload in ('bad json', '{}', json.dumps(dict(session='s', frame_seq=9, export_seq=5))):
            Bridge.acknowledge(node, String(data=payload))
            self.assertIsNotNone(node.pending_frame)
        message = String(data=json.dumps(dict(session='s', frame_seq=10, export_seq=5)))
        Bridge.acknowledge(node, message)
        Bridge.acknowledge(node, message)
        self.assertIsNone(node.pending_frame)
        self.assertEqual(node.acknowledged_count, 1)

    def test_no_processor_means_no_dequeue(self):
        node = self.bridge()
        node.pending_frame = None
        node.frame_pub.get_subscription_count.return_value = 0
        node.frames.put('next frame')
        Bridge.publish_next(node)
        self.assertEqual(node.frames.qsize(), 1)

    def processor(self):
        image = np.zeros((2, 2, 3), np.uint8)
        engine = Mock()
        engine.process.return_value = ({}, image, image)
        node = SimpleNamespace(engine=engine, last_identity=('s', 9), last_export=('s', 4),
            last_frame=0, sequence_gaps=0, results=Mock(), timing=Mock(), debug=Mock(), top=Mock(),
            ack=Mock(), unavailable=Mock(), get_logger=Mock())
        node.acknowledge = lambda frame, seq: Processor.acknowledge(node, frame, seq)
        return node

    def frame(self, seq=6):
        frame = PerceptionFrame(header=Header(), session='s', sequence=10)
        frame.metadata_json = json.dumps(dict(session='s', frame_seq=10, export_seq=seq,
            source={}, objects=[], depth_available=False))
        for name, encoding, data in (
            ('rgb', 'bgr8', np.zeros((2, 2, 3), np.uint8)),
            ('drivable_mask', 'mono8', np.zeros((2, 2), np.uint8)),
            ('lane_mask', 'mono8', np.zeros((2, 2), np.uint8)),
            ('depth', '16UC1', np.zeros((2, 2), np.uint16))):
            setattr(frame, name, image_message(data, encoding, frame.header))
        return frame

    def test_gap_resets_tracking_reports_discontinuity_and_continues(self):
        node = self.processor()
        Processor.process(node, self.frame())
        node.engine.reset.assert_called_once()
        node.unavailable.assert_called_once()
        result = json.loads(node.results.publish.call_args[0][0].data)
        self.assertEqual(result['continuity_error'], dict(previous=4, current=6))
        self.assertEqual(result['sequence_gaps'], 1)
        self.assertEqual(node.last_export, ('s', 6))
        node.ack.publish.assert_called_once()

    def test_retry_is_acknowledged_without_duplicate_processing(self):
        node = self.processor()
        node.last_export = ('s', 5)
        frame = self.frame()
        Processor.process(node, frame)
        Processor.process(node, frame)
        node.engine.process.assert_called_once()
        self.assertEqual(node.ack.publish.call_count, 2)
        node.engine.reset.assert_not_called()

    def test_invalid_bundle_is_not_acknowledged(self):
        node = self.processor()
        frame = self.frame()
        frame.session = 'wrong'
        Processor.process(node, frame)
        node.ack.publish.assert_not_called()
        node.engine.process.assert_not_called()
        node.unavailable.assert_called_once()


if __name__ == '__main__':
    unittest.main()
