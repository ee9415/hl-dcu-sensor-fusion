"""RecorderApp._update_video_connection_ui(영상 연결 상태 버튼/재시도 표시) 테스트."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from oak_recorder import RecorderApp


class VideoConnectionTests(unittest.TestCase):
    def setUp(self):
        """연결 대기 시작 시각 100초, 4개 소스 모두 120초에 프레임을 받은 상태로 앱을 흉내 낸다."""
        self.app = SimpleNamespace(connect_requested=True, connection_launch_pending=False,
            video_wait_started_at=100., video_retry_available=False, active_input='camera',
            sources={key:SimpleNamespace(last_frame_at=120.) for key in ('rgb','yolopv2','bev','depth')},
            connect_button=Mock(), ip_entry=Mock(), password_entry=Mock(),
            discover_button=Mock(), status_text=Mock())
        self.frames={key:(True,object()) for key in self.app.sources}

    def update(self, now=121.):
        """현재 시각을 now로 고정하고 UI 갱신을 실행한다."""
        with patch('oak_recorder.time.monotonic', return_value=now):
            RecorderApp._update_video_connection_ui(self.app, self.frames)

    def test_all_frames_completes(self):
        """모든 소스에 최신 프레임이 있으면 '영상 연결됨'으로 표시한다."""
        self.update()
        self.assertEqual(self.app.connect_button.configure.call_args.kwargs['text'],'영상 연결됨')

    def test_ports_without_frames_allow_retry(self):
        """포트는 열렸지만 프레임이 없으면 재시도 버튼을 켜되, 백그라운드 연결 시도는 계속 유지한다."""
        self.frames={key:(True,None) for key in self.frames}
        self.update()
        self.assertTrue(self.app.video_retry_available)
        self.assertEqual(self.app.connect_button.configure.call_args.kwargs['state'],'normal')
        self.assertTrue(self.app.connect_requested)  # Background recovery remains observed.

    def test_partial_stream_has_deadline(self):
        """일부 스트림만 들어오는 상태도 대기 시간을 넘기면 재시도를 허용한다."""
        self.frames['depth']=(False,None)
        self.update()
        self.assertTrue(self.app.video_retry_available)

    def test_stale_frames_are_not_connected(self):
        """마지막 프레임이 오래되면(10초 경과) 연결된 것으로 보지 않는다."""
        self.update(130.)
        self.assertTrue(self.app.video_retry_available)

    def test_pc_does_not_require_depth(self):
        """PC 입력 모드에서는 depth 스트림이 없어도 정상이다."""
        self.app.active_input='pc'; self.frames['depth']=(False,None)
        self.update()
        self.assertFalse(self.app.video_retry_available)

    def test_background_recovery_clears_retry(self):
        """백그라운드에서 스트림이 복구되면 재시도 표시를 끈다."""
        self.frames['rgb']=(False,None)
        self.update()
        self.assertTrue(self.app.video_retry_available)
        self.frames['rgb']=(True,object())
        self.update()
        self.assertFalse(self.app.video_retry_available)

    def test_initial_wait_has_no_premature_failure(self):
        """초기 대기 시간 안에는 실패로 판단하지 않고 '연결 중'으로 표시한다."""
        self.frames={key:(False,None) for key in self.frames}
        self.update(110.)
        self.assertFalse(self.app.video_retry_available)
        self.assertEqual(self.app.connect_button.configure.call_args.kwargs['text'],'영상 연결 중...')
