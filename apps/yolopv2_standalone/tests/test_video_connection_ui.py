from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from oak_recorder import RecorderApp


class VideoConnectionTests(unittest.TestCase):
    def setUp(self):
        self.app = SimpleNamespace(connect_requested=True, connection_launch_pending=False,
            video_wait_started_at=100., video_retry_available=False, active_input='camera',
            sources={key:SimpleNamespace(last_frame_at=120.) for key in ('rgb','yolopv2','bev','depth')},
            connect_button=Mock(), ip_entry=Mock(), password_entry=Mock(),
            discover_button=Mock(), status_text=Mock())
        self.frames={key:(True,object()) for key in self.app.sources}

    def update(self, now=121.):
        with patch('oak_recorder.time.monotonic', return_value=now):
            RecorderApp._update_video_connection_ui(self.app, self.frames)

    def test_all_frames_completes(self):
        self.update()
        self.assertEqual(self.app.connect_button.configure.call_args.kwargs['text'],'영상 연결됨')

    def test_ports_without_frames_allow_retry(self):
        self.frames={key:(True,None) for key in self.frames}
        self.update()
        self.assertTrue(self.app.video_retry_available)
        self.assertEqual(self.app.connect_button.configure.call_args.kwargs['state'],'normal')
        self.assertTrue(self.app.connect_requested)  # Background recovery remains observed.

    def test_partial_stream_has_deadline(self):
        self.frames['depth']=(False,None)
        self.update()
        self.assertTrue(self.app.video_retry_available)

    def test_stale_frames_are_not_connected(self):
        self.update(130.)
        self.assertTrue(self.app.video_retry_available)

    def test_pc_does_not_require_depth(self):
        self.app.active_input='pc'; self.frames['depth']=(False,None)
        self.update()
        self.assertFalse(self.app.video_retry_available)

    def test_background_recovery_clears_retry(self):
        self.frames['rgb']=(False,None)
        self.update()
        self.assertTrue(self.app.video_retry_available)
        self.frames['rgb']=(True,object())
        self.update()
        self.assertFalse(self.app.video_retry_available)

    def test_initial_wait_has_no_premature_failure(self):
        self.frames={key:(False,None) for key in self.frames}
        self.update(110.)
        self.assertFalse(self.app.video_retry_available)
        self.assertEqual(self.app.connect_button.configure.call_args.kwargs['text'],'영상 연결 중...')
