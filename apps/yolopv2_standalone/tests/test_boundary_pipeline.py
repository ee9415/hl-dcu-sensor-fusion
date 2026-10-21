"""Host-node integration without a camera or the RVC4 DepthAI runtime."""
from datetime import timedelta
import importlib
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np


class Frame:
    Type = types.SimpleNamespace(NV12=0, BGR888i=1)

    def __init__(self):
        self.data = None
        self.timestamp = timedelta(seconds=10)
        self.sequence = 7

    def setCvFrame(self, data, kind):
        self.data = data

    def getCvFrame(self):
        return np.zeros((1080, 1920, 3), np.uint8)

    def getFrame(self):
        return np.zeros((1080, 1920), np.uint16)

    def getTransformation(self):
        return None

    def getTimestamp(self):
        return self.timestamp

    def getSequenceNum(self):
        return self.sequence

    def setTimestamp(self, value):
        self.timestamp = value

    def setSequenceNum(self, value):
        self.sequence = value


class BoundaryPipelineTests(unittest.TestCase):
    def test_result_bev_and_telemetry_use_boundary_pair(self):
        class Host:
            def createOutput(self, **kwargs):
                return MagicMock()

        dai = types.SimpleNamespace(node=types.SimpleNamespace(HostNode=Host, ThreadedHostNode=Host),
                                    Node=MagicMock(), DatatypeEnum=MagicMock(), ImgFrame=Frame,
                                    ImgDetections=Frame, SpatialImgDetections=Frame,
                                    LengthUnit=MagicMock(),
                                    Clock=types.SimpleNamespace(now=lambda: timedelta(seconds=10)))
        # Annotations are evaluated during import in yolopv2_node.py.
        dai.NNData = type('NNData', (), {})
        dai.ImgDetection = type('ImgDetection', (), {})
        with patch.dict(sys.modules, {'depthai': dai}):
            node = importlib.import_module('yolopv2_node')
        road = np.full((180, 320), 255, np.uint8)
        from lane_boundaries import projection_for, transform, draw_polyline
        mask_rgb = np.zeros((180,320,3),np.uint8)
        forward = np.linspace(.5,25,60)
        for side in (-2.5,3.):
            points = transform(np.column_stack(((np.full(60,side)/12+.5)*639,
                                                (1-forward/40)*479)),projection_for(road.shape))
            draw_polyline(mask_rgb,points,(255,255,255),thickness=1)
        lane = mask_rgb[:,:,0]
        source = np.repeat(np.repeat(mask_rgb,6,axis=0),6,axis=1)
        with patch.object(node, 'TelemetryServer') as server, \
             patch.object(node, 'decode_heads'), \
             patch.object(node, 'non_max_suppression', return_value=[]), \
             patch.object(node, 'segmentation_masks', return_value=(road, lane)), \
             patch.object(Frame, 'getCvFrame', return_value=source):
            processor = node.Yolopv2Postprocess()
            processor._tensor = MagicMock()
            processor.process(Frame(), Frame(), Frame())
            payload = server.return_value.publish.call_args.args[0]
            self.assertTrue(payload['boundary_valid'])
            self.assertFalse(payload['centerline_valid'])
            self.assertIsNone(payload['coefficients'])
            self.assertEqual(payload['turn_mode'], 'disabled')
            self.assertEqual(payload['frame_seq'], 7)
            self.assertGreater(payload['boundary_width_min_dev'],4.5)
            self.assertTrue(payload['clearance_sufficient'])
            result = processor.result.send.call_args.args[0]
            bev = processor.bev.send.call_args.args[0]
            self.assertEqual(result.data.shape, (540,960,3))
            self.assertEqual(bev.data.shape, (480,640,3))
            self.assertEqual(result.getSequenceNum(), 7)
            self.assertEqual(bev.getTimestamp(), timedelta(seconds=10))
            for color in ((0,220,255), (255,180,0)):
                self.assertTrue(np.any(np.all(result.data == color, axis=2)))
            # A fresh pipeline with only the left stripe must still publish and
            # display that stripe, with no fabricated right edge or center path.
            partial_lane=lane.copy()
            partial_lane[:,160:]=0
            with patch.object(node,'segmentation_masks',return_value=(road,partial_lane)):
                partial=node.Yolopv2Postprocess()
                partial._tensor=MagicMock()
                partial.process(Frame(),Frame(),Frame())
            payload=server.return_value.publish.call_args.args[0]
            self.assertTrue(payload['boundary_valid'])
            self.assertTrue(payload['left_boundary_valid'])
            self.assertFalse(payload['right_boundary_valid'])
            self.assertFalse(payload['boundary_pair_valid'])
            self.assertTrue(all(v is None for v in payload['right_boundary_dev']))
            self.assertFalse(payload['centerline_valid'])
            result=partial.result.send.call_args.args[0]
            self.assertTrue(np.any(np.all(result.data==(0,220,255),axis=2)))
            self.assertFalse(np.any(np.all(result.data==(255,180,0),axis=2)))
