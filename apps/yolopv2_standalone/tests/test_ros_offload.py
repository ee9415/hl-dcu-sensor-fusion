import copy
import json
import socket
import struct
import queue
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from host_postprocess import HostPostprocess, validate_frame
from perception_wire import MAGIC, MAX_PACKET, decode_bundle, encode_bundle, receive_bundle
from perception_server import PerceptionServer, put_ordered


def fixture():
    rgb = np.zeros((540, 960, 3), np.uint8)
    lane = np.zeros((180, 320), np.uint8)
    # Two long real lane markings, giving the tracker useful observations.
    lane[65:178, 75:81] = 255
    lane[65:178, 238:244] = 255
    arrays = dict(rgb=rgb, drivable=np.full_like(lane, 255), lane=lane,
                  depth=np.full((1080, 1920), 1234, dtype='<u2'))
    metadata = dict(schema_version=1, session='test', frame_seq=1, nn_seq=1, depth_seq=3,
        source=dict(input_source='camera', source_epoch=1), source_size=[1920, 1080],
        capture_timestamp_s=10., nn_timestamp_s=10., depth_timestamp_s=10.01,
        mask_coordinates='source_fov_letterbox_removed',
        box_coordinates='normalized_source_xyxy', depth_unit='mm', depth_aligned_to='rgb',
        depth_available=True, rgb_intrinsics=np.eye(3).tolist(), depth_intrinsics=np.eye(3).tolist(),
        objects=[dict(label=0, confidence=.8, xmin=.4, ymin=.2, xmax=.6, ymax=.4)])
    return metadata, arrays


class ProtocolTests(unittest.TestCase):
    def test_offload_keeps_runtime_health_contract_without_tracking(self):
        import importlib
        import sys
        import types
        from datetime import timedelta
        from test_boundary_pipeline import Frame
        class Host:
            def createOutput(self, **kwargs):
                return MagicMock()
        dai = types.SimpleNamespace(node=types.SimpleNamespace(HostNode=Host, ThreadedHostNode=Host),
            Node=MagicMock(), DatatypeEnum=MagicMock(), ImgFrame=Frame, NNData=Frame,
            ImgDetection=Frame, ImgDetections=Frame, SpatialImgDetections=Frame,
            LengthUnit=MagicMock(), Clock=types.SimpleNamespace(now=lambda: timedelta(seconds=10)))
        with patch.dict(sys.modules, {'depthai': dai}):
            previous = sys.modules.pop('yolopv2_node', None)
            node = importlib.import_module('yolopv2_node')
            if previous is not None:
                sys.modules['yolopv2_node'] = previous
        with (patch.object(node, 'TelemetryServer') as telemetry,
              patch.object(node, 'PerceptionExport') as export,
              patch.object(node, 'decode_heads'),
              patch.object(node, 'non_max_suppression', return_value=[]),
              patch.object(node, 'segmentation_masks', return_value=(np.zeros((180,320),np.uint8),)*2)):
            export.return_value.camera_postprocess = False
            export.return_value.rejected = 0
            export.return_value.server.errors = 0
            export.return_value.server.accepted = 0
            export.return_value.server.encoded = 0
            export.return_value.server.high_water = 0
            processor = node.Yolopv2Postprocess()
            processor._tensor = MagicMock()
            with patch.object(node.ImageBoundaryTracker, 'update_contours') as tracking:
                processor.process(Frame(), Frame(), Frame())
                tracking.assert_not_called()
            payload = telemetry.return_value.publish.call_args.args[0]
            self.assertEqual(payload['processing_mode'], 'full')
            self.assertEqual(payload['postprocess_location'], 'ros')
            self.assertFalse(payload['guidance_valid'])
            self.assertTrue(processor.result.send.called)

    def test_lossless_depth_and_binary_masks(self):
        metadata, arrays = fixture()
        packet = encode_bundle(metadata, arrays)
        decoded, values = decode_bundle(packet[len(MAGIC)+4:])
        self.assertEqual(decoded['frame_seq'], 1)
        for name in arrays:
            np.testing.assert_array_equal(values[name], arrays[name])
        validate_frame(decoded, values)
        self.assertEqual(decoded['arrays']['rgb']['codec'], 'raw')
        self.assertEqual(decoded['arrays']['depth']['codec'], 'zlib')

    def test_full_queue_does_not_evict_and_cancel_releases_waiter(self):
        mailbox = queue.Queue(maxsize=1)
        mailbox.put('first')
        active = threading.Event()
        active.set()
        result = []
        worker = threading.Thread(target=lambda:result.append(put_ordered(mailbox,'second',active)))
        worker.start()
        time.sleep(.03)
        self.assertTrue(worker.is_alive())
        self.assertEqual(mailbox.get(), 'first')
        worker.join(timeout=1)
        self.assertEqual(mailbox.get(), 'second')
        self.assertEqual(result, [True])
        mailbox.put('retained')
        with self.assertRaises(RuntimeError):
            put_ordered(mailbox,'overflow',active,timeout=.02)
        self.assertEqual(mailbox.get(),'retained')
        active.clear()
        self.assertFalse(put_ordered(mailbox,'cancelled',active))

    def test_tcp_server_exports_every_frame_in_order_without_fps_cap(self):
        server = PerceptionServer(port=0, queue_depth=2)
        client = socket.create_connection(server.server.server_address, timeout=2)
        client.settimeout(2)
        try:
            client.sendall(b'OAK_PERCEPTION_1\n')
            deadline=time.monotonic()+2
            while not server.wanted() and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertTrue(server.wanted())
            # Faster than 10FPS; an implicit time throttle would lose frames.
            for index in range(12):
                server.publish(dict(frame_seq=index), dict(rgb=np.full((4,4,3),index,np.uint8)))
            for index in range(12):
                metadata, arrays = receive_bundle(client)
                self.assertEqual(metadata['frame_seq'],index)
                self.assertEqual(metadata['export_seq'],index+1)
                np.testing.assert_array_equal(arrays['rgb'],index)
            self.assertEqual(server.accepted,12)
            self.assertIsNone(server.fault)
        finally:
            client.close()
            server.close()

    def test_raw_array_length_is_validated(self):
        metadata, arrays = fixture()
        body = encode_bundle(metadata,arrays)[len(MAGIC)+4:]
        size=struct.unpack('!I',body[:4])[0]
        header=json.loads(body[4:4+size])
        header['arrays']['rgb']['shape']=[1,1,3]
        raw=json.dumps(header).encode()
        with self.assertRaises(ValueError):
            decode_bundle(struct.pack('!I',len(raw))+raw+body[4+size:])

    def test_truncation_oversized_packet_and_trailing_bytes(self):
        metadata, arrays = fixture()
        body = encode_bundle(metadata, arrays)[len(MAGIC)+4:]
        for invalid in (body[:-1], body+b'x'):
            with self.assertRaises(ValueError):
                decode_bundle(invalid)
        left, right = socket.socketpair()
        try:
            left.sendall(MAGIC+struct.pack('!I', MAX_PACKET+1))
            with self.assertRaises(ValueError):
                receive_bundle(right)
        finally:
            left.close()
            right.close()

    def test_decompression_limit(self):
        metadata, arrays = fixture()
        body = encode_bundle(metadata, arrays)[len(MAGIC)+4:]
        size = struct.unpack('!I', body[:4])[0]
        header = json.loads(body[4:4+size])
        header['arrays']['depth']['shape'] = [1, 1]
        raw = json.dumps(header).encode()
        with self.assertRaises(ValueError):
            decode_bundle(struct.pack('!I', len(raw))+raw+body[4+size:])

    def test_reject_mixed_frames_and_live_depth_on_pc_video(self):
        metadata, arrays = fixture()
        for mutation in (dict(nn_seq=2), dict(depth_timestamp_s=10.3),
                         dict(source=dict(input_source='pc'))):
            invalid = dict(metadata, **mutation)
            with self.assertRaises(ValueError):
                validate_frame(invalid, arrays)
        metadata['source']['input_source'] = 'pc'
        metadata['depth_available'] = False
        validate_frame(metadata, arrays)

    def test_tracking_reset_and_debug_geometry(self):
        metadata, arrays = fixture()
        engine = HostPostprocess()
        for index in range(6):
            metadata.update(frame_seq=index+1, nn_seq=index+1,
                            capture_timestamp_s=10.+index*.2,
                            nn_timestamp_s=10.+index*.2, depth_timestamp_s=10.+index*.2)
            result, debug, top = engine.process(metadata, arrays)
        self.assertEqual(debug.shape, (540, 960, 3))
        self.assertEqual(top.shape, (480, 640, 3))
        self.assertTrue(result['boundary_valid'])
        metadata = copy.deepcopy(metadata)
        metadata['session'] = 'restarted'
        arrays['lane'][:] = 0
        result, _, _ = engine.process(metadata, arrays)
        self.assertFalse(result['boundary_valid'])
        self.assertFalse(result['guidance_valid'])


if __name__ == '__main__':
    unittest.main()
