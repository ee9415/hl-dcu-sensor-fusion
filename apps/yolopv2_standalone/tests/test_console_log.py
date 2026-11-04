import contextlib
import io
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from console_log import action, event, install_dialog_logging, register_secret, state_event, progress_task


class ConsoleLogTests(unittest.TestCase):
    def test_background_progress_can_be_silent(self):
        @progress_task(None, interval=.001)
        def work():
            time.sleep(.02)
            return 42
        with patch('console_log.event') as logged:
            self.assertEqual(work(), 42)
        logged.assert_not_called()

    def test_idle_poll_does_not_query_device(self):
        import oak_recorder
        app = SimpleNamespace(source_closing=threading.Event(),
            six_camera_tab=SimpleNamespace(extra_sources={}),
            connect_requested=False, sources={}, ip_text=Mock(get=lambda:'192.0.2.1'),
            _camera_app_action=Mock(), root=Mock(), _poll_camera_app=Mock())
        oak_recorder.RecorderApp._poll_camera_app(app)
        app._camera_app_action.assert_not_called()
        app.connect_requested=True
        oak_recorder.RecorderApp._poll_camera_app(app)
        app._camera_app_action.assert_called_once_with('poll')

    def test_progress_repeats_until_completion_then_stops(self):
        seen = []
        enough = threading.Event()
        def record(message, *args):
            seen.append(message)
            if sum('처리 중' in x for x in seen) >= 2:
                enough.set()
        @progress_task('연결', interval=.01)
        def work():
            self.assertTrue(enough.wait(1))
            return 42
        with patch('console_log.event', side_effect=record):
            self.assertEqual(work(), 42)
            count = len(seen)
            time.sleep(.04)
        self.assertEqual(len(seen), count)
        self.assertIn('처리 종료', seen[-1])

    def test_progress_does_not_duplicate_nested_tasks_and_stops_on_error(self):
        enough = threading.Event()
        seen = []
        def record(message, *args):
            seen.append(message)
            enough.set()
        @progress_task('내부', interval=.01)
        def inner():
            self.assertTrue(enough.wait(1))
            raise ValueError('test')
        @progress_task('외부', interval=.01)
        def outer():
            inner()
        with patch('console_log.event', side_effect=record):
            with self.assertRaises(ValueError):
                outer()
            count = len(seen)
            time.sleep(.04)
        self.assertEqual(len(seen), count)
        self.assertTrue(all('외부' in message for message in seen))
        self.assertIn('오류로 종료', seen[-1])

    def test_multiview_start_never_connects_even_with_legacy_flag(self):
        import oak_recorder
        for legacy_flag in (False, True):
            args = SimpleNamespace(device_ip='192.0.2.1', device_serial='test',
                rgb_url=None, result_url=None, bev_url=None, output_dir=Path('unused'),
                camera_settings=None, auto_connect=legacy_flag)
            root = Mock()
            app = Mock()
            with (patch.object(oak_recorder, 'parse_args', return_value=args),
                  patch.object(oak_recorder.tk, 'Tk', return_value=root),
                  patch.object(oak_recorder, 'RecorderApp', return_value=app),
                  contextlib.redirect_stdout(io.StringIO())):
                oak_recorder.main()
            root.after.assert_not_called()
            app.connect_video.assert_not_called()
            root.mainloop.assert_called_once()

    def test_repeated_polling_emits_only_transitions(self):
        owner = SimpleNamespace()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            for _ in range(100):
                state_event(owner, 'camera', False, 'CAM01 연결 대기')
            state_event(owner, 'camera', True, 'CAM01 연결됨')
            state_event(owner, 'camera', True, 'CAM01 연결됨')
        self.assertEqual(len(output.getvalue().splitlines()), 2)

    def test_passwords_and_url_credentials_are_redacted(self):
        register_secret('test-console-secret-123')
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            event('test-console-secret-123 --password other-secret rtsp://user:camera-secret@host/rgb\nfailed')
        result = output.getvalue()
        for secret in ('test-console-secret-123', 'other-secret', 'camera-secret'):
            self.assertNotIn(secret, result)
        self.assertEqual(len(result.splitlines()), 1)

    def test_action_preserves_results_and_does_not_dump_arguments(self):
        @action('녹화 요청')
        def callback(secret):
            return secret
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(callback('not-to-be-logged'), 'not-to-be-logged')
        self.assertIn('[ACTION]', output.getvalue())
        self.assertNotIn('not-to-be-logged', output.getvalue())

    def test_dialog_logging_is_not_duplicated(self):
        original = Mock(return_value='ok')
        dialogs = SimpleNamespace(showerror=original, showwarning=Mock())
        install_dialog_logging(dialogs)
        install_dialog_logging(dialogs)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(dialogs.showerror('연결', '실패'), 'ok')
        original.assert_called_once_with('연결', '실패')
        self.assertEqual(len(output.getvalue().splitlines()), 1)
