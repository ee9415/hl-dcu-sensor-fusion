import unittest
from lane_boundaries import BoundaryConfig
from skeleton_response import response_config, validate_speed
from video_transport import VideoInputServer
from video_file_sender import set_skeleton_response_speed


class ResponseTests(unittest.TestCase):
    def test_scaling_is_reversible_and_does_not_compound(self):
        base=BoundaryConfig.load()
        fast=response_config(base,2)
        self.assertEqual(fast.skeleton_retained_max_rate_px_s,80)
        self.assertEqual(fast.skeleton_retained_tau_s,.1)
        self.assertEqual(response_config(base,1),base)
        self.assertEqual(response_config(base,.5).skeleton_observed_max_rate_px_s,60)
        self.assertEqual(base.skeleton_retained_max_rate_px_s,40)

    def test_invalid_values(self):
        for value in (True,None,'2',float('nan'),float('inf'),.49,3.01):
            with self.subTest(value=value), self.assertRaises(ValueError):validate_speed(value)

    def test_live_control_preserves_source_and_frame_snapshot(self):
        hub=VideoInputServer(0,'127.0.0.1')
        try:
            port=hub.server.server_address[1]
            _,old=hub.take()
            hub.remember(1,old)
            result=set_skeleton_response_speed('127.0.0.1',2,port)
            _,new=hub.take()
            self.assertEqual(result['skeleton_response_speed'],2)
            self.assertEqual(new['skeleton_response_speed'],2)
            self.assertEqual(hub.metadata(1)['skeleton_response_speed'],1)
            self.assertEqual(new['source_epoch'],old['source_epoch'])
            self.assertEqual(new['processing_epoch'],old['processing_epoch'])
            epoch=new['skeleton_response_epoch']
            self.assertEqual(set_skeleton_response_speed('127.0.0.1',2,port)['skeleton_response_epoch'],epoch)
            with self.assertRaises(ValueError):hub.handle(None,dict(op='skeleton_response',speed=2),b'x')
        finally:hub.close()
