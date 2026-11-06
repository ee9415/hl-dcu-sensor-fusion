"""PC input settings can be applied before a camera connection exists."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from oak_recorder import RecorderApp


class SourceApplyTests(unittest.TestCase):
    def app(self, path):
        app = RecorderApp.__new__(RecorderApp)
        app.six_camera_tab = Mock(is_cam01_recording=False)
        app.recording = app.source_change_pending = app.connect_requested = False
        for name, value in dict(input_mode='PC 영상 파일', input_file=str(path),
                                input_loop=True, input_start='1:20', input_end='100',
                                input_fps='10').items():
            setattr(app, name, Mock(get=Mock(return_value=value)))
        app.source_start_button = Mock()
        app.source_status = Mock()
        app.file_sender = Mock()
        app._switch_input_source = Mock()
        return app

    @patch('oak_recorder.messagebox.showwarning')
    @patch('oak_recorder.video_input_settings.save')
    def test_offline_pc_apply_saves_settings_without_interrupting_sender(self, save, warn):
        with tempfile.NamedTemporaryFile() as video:
            app = self.app(video.name)
            sender = app.file_sender
            app._apply_input_source()
        save.assert_called_once()
        warn.assert_not_called()
        self.assertEqual(app.applied_input_settings[3:], (80., 100., 10))
        self.assertIs(app.file_sender, sender)
        app._switch_input_source.assert_not_called()

    @patch('oak_recorder.messagebox.showwarning')
    @patch('oak_recorder.video_input_settings.save')
    def test_offline_camera_selection_still_requires_connection(self, save, warn):
        app = self.app('unused')
        app.input_mode.get.return_value = '카메라'
        app._apply_input_source()
        warn.assert_called_once()
        save.assert_not_called()
        app._switch_input_source.assert_not_called()

    @patch('oak_recorder.messagebox.showwarning')
    @patch('oak_recorder.video_input_settings.save')
    def test_offline_transfer_start_still_requires_connection(self, save, warn):
        app = self.app('unused')
        app._start_input_transfer()
        warn.assert_called_once()
        save.assert_not_called()
        app._switch_input_source.assert_not_called()
