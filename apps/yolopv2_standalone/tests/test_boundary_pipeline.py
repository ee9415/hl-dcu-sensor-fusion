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
        for y in range(78,176):
            for x in (160-(y-65)*.9,160+(y-65)*.9):
                x=int(x)
                mask_rgb[y,x-1:x+2]=230
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
            self.assertIsNone(payload['corridor_width_m'])
            self.assertIsNone(payload['normal_width_error_dev'])
            self.assertEqual(payload['boundary_model'],'image_motion')
            self.assertGreater(len(payload['left_boundary_image']),12)
            self.assertGreater(len(payload['right_boundary_image']),12)
            result = processor.result.send.call_args.args[0]
            bev = processor.bev.send.call_args.args[0]
            self.assertEqual(result.data.shape, (540,960,3))
            self.assertEqual(bev.data.shape, (480,640,3))
            self.assertEqual(result.getSequenceNum(), 7)
            self.assertEqual(bev.getTimestamp(), timedelta(seconds=10))
            for color in ((0,220,255), (255,180,0)):
                self.assertTrue(np.any(np.all(result.data == color, axis=2)))
            with patch.object(node,'segmentation_masks',return_value=(road,np.zeros_like(lane))):
                rgb_only=node.Yolopv2Postprocess()
                rgb_only._tensor=MagicMock()
                rgb_only.process(Frame(),Frame(),Frame())
            self.assertTrue(server.return_value.publish.call_args.args[0]['boundary_pair_valid'])
            # One observed side initializes a low-confidence image perspective prior.
            partial_lane=lane.copy()
            partial_lane[:,160:]=0
            partial_source=source.copy()
            partial_source[:,960:]=0
            with patch.object(node,'segmentation_masks',return_value=(road,partial_lane)), \
                 patch.object(Frame,'getCvFrame',return_value=partial_source):
                partial=node.Yolopv2Postprocess()
                partial._tensor=MagicMock()
                partial.process(Frame(),Frame(),Frame())
            payload=server.return_value.publish.call_args.args[0]
            self.assertTrue(payload['boundary_valid'])
            self.assertTrue(payload['left_boundary_valid'])
            self.assertTrue(payload['right_boundary_valid'])
            self.assertTrue(payload['boundary_pair_valid'])
            self.assertFalse(payload['centerline_valid'])
            result=partial.result.send.call_args.args[0]
            self.assertTrue(np.any(np.all(result.data==(0,220,255),axis=2)))
            self.assertEqual(payload['right_boundary_state'],'inferred')
            self.assertTrue(np.any(np.all(result.data==(114,81,0),axis=2)))
            # Even if a real-looking stereo image reaches the callback, PC
            # provenance must suppress its distances and reset old tracking.
            old_tracker=processor._boundary_tracker
            processor.video_input=MagicMock()
            processor.video_input.metadata.return_value=dict(input_source='pc',
                source_epoch=2,source_frame_index=20,source_media_time_s=2.)
            with patch.object(Frame,'getFrame',return_value=np.full((180,320),2000,np.uint16)), \
                 patch.object(node,'non_max_suppression',return_value=np.array([[80,100,180,220,.9,0]])), \
                 patch.object(node,'estimate_spatial_from_depth',wraps=node.estimate_spatial_from_depth) as spatial:
                processor.process(Frame(),Frame(),Frame())
            self.assertIsNot(old_tracker,processor._boundary_tracker)
            self.assertFalse(spatial.call_args.args[0].any())
            payload=server.return_value.publish.call_args.args[0]
            self.assertFalse(payload['depth_available'])
            self.assertEqual(payload['source_frame_index'],20)
            self.assertFalse(payload['objects'][0]['depth_valid'])
