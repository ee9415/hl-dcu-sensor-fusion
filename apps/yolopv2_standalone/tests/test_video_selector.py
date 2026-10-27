from datetime import timedelta
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np


class Frame:
    def __init__(self):
        self.timestamp=timedelta(seconds=12)
        self.sequence=25
    def getTimestamp(self):return self.timestamp
    def setTimestamp(self,value):self.timestamp=value
    def getSequenceNum(self):return self.sequence
    def setSequenceNum(self,value):self.sequence=value
    def setCvFrame(self,value,kind):self.data=value;self.kind=kind
    def setFrame(self,value):self.data=value
    def setType(self,value):self.kind=value
    def setWidth(self,value):self.width=value
    def setHeight(self,value):self.height=value


class SelectorTests(unittest.TestCase):
    def node(self):
        class Host:
            def createOutput(self,**kwargs):return Mock()
        dai=types.SimpleNamespace(node=types.SimpleNamespace(HostNode=Host),
            Node=Mock(),DatatypeEnum=Mock(),ImgFrame=Frame)
        Frame.Type=types.SimpleNamespace(NV12='NV12',RAW16='RAW16')
        spec=importlib.util.spec_from_file_location('selector_fixture',
                         Path(__file__).resolve().parents[1]/'video_input_node.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'depthai':dai}):spec.loader.exec_module(module)
        node=module.VideoInputSelector();node.hub=Mock()
        return node

    def test_pc_pair_timestamps_and_zero_depth(self):
        node=self.node();camera,depth=Frame(),Frame()
        image=np.full((540,960,3),(10,20,230),np.uint8)
        payload=cv2.imencode('.jpg',image)[1].tobytes()
        meta=dict(input_source='pc',source_epoch=2,source_frame_index=7)
        node.hub.take.return_value=(payload,meta)
        node.process(camera,depth)
        color=node.video_out.send.call_args.args[0];invalid=node.depth_out.send.call_args.args[0]
        self.assertEqual(color.kind,'NV12');self.assertEqual(color.data.shape,(1080,1920,3))
        self.assertGreater(int(color.data[50,50,2]),200)
        self.assertEqual(invalid.kind,'RAW16');self.assertFalse(invalid.data.any())
        self.assertEqual(color.getTimestamp(),invalid.getTimestamp())
        self.assertEqual(color.getSequenceNum(),invalid.getSequenceNum())
        node.hub.remember.assert_called_once_with(25,meta)

    def test_camera_passthrough_and_disconnected_pc_wait(self):
        node=self.node();camera,depth=Frame(),Frame()
        node.hub.take.return_value=(None,dict(input_source='camera',source_epoch=0))
        node.process(camera,depth)
        self.assertIs(node.video_out.send.call_args.args[0],camera)
        self.assertIs(node.depth_out.send.call_args.args[0],depth)
        node.video_out.send.reset_mock();node.depth_out.send.reset_mock()
        node.hub.take.return_value=(None,None)
        node.process(camera,depth)
        node.video_out.send.assert_not_called();node.depth_out.send.assert_not_called()


if __name__=='__main__':unittest.main()
