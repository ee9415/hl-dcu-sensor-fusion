"""skeleton_response(경계선 skeleton 추종 속도 배율) 설정 및 실시간 제어 테스트."""
import unittest
from lane_boundaries import BoundaryConfig
from skeleton_response import response_config, validate_speed
from video_transport import VideoInputServer
from video_file_sender import set_skeleton_response_speed


class ResponseTests(unittest.TestCase):
    def test_scaling_is_reversible_and_does_not_compound(self):
        """배율은 항상 기본 설정을 기준으로 적용되어 누적되지 않고, 1배는 원래 설정과 같다."""
        base=BoundaryConfig.load()
        fast=response_config(base,2)
        self.assertEqual(fast.skeleton_retained_max_rate_px_s,80)
        self.assertEqual(fast.skeleton_retained_tau_s,.1)
        self.assertEqual(response_config(base,1),base)
        self.assertEqual(response_config(base,.5).skeleton_observed_max_rate_px_s,60)
        self.assertEqual(base.skeleton_retained_max_rate_px_s,40)

    def test_invalid_values(self):
        """bool/None/문자열/NaN/inf 및 0.5~3 범위 밖 값은 거부한다."""
        for value in (True,None,'2',float('nan'),float('inf'),.49,3.01):
            with self.subTest(value=value), self.assertRaises(ValueError):validate_speed(value)

    def test_live_control_preserves_source_and_frame_snapshot(self):
        """실시간 속도 변경은 입력/처리 epoch를 유지하고, 과거 프레임 메타데이터는 바꾸지 않으며, 같은 값 재요청은 epoch를 올리지 않는다."""
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
