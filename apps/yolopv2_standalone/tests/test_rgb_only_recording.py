"""Mode ownership and recording path regressions; no physical camera required."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, Mock, patch
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
import oak4_rgb_source as raw
from six_camera_tab import SixCameraTab
from oakd_rgb_source import OakdRgbSource
from shared_rgb_recorder import RecordingGroup

spec = importlib.util.spec_from_file_location('rgb_only_main', ROOT/'apps/rgb_only/main.py')
rgb_app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rgb_app)


class RawModeTests(unittest.TestCase):
    def test_rgb_connection_starts_installed_app_only(self):
        stop = threading.Event()
        with (patch.object(raw.oak_app_control, 'start_installed', return_value='installed-rgb') as start,
              patch.object(raw, 'wait_rgb_ready') as wait):
            raw.prepare_rgb('cli', 'ip', '', stop)
            start.assert_called_once_with('cli', 'ip', raw.IDENTIFIER, '', stop_event=stop)
            wait.assert_called_once()

    def test_cancelled_rgb_connection_never_starts_an_app(self):
        stop = threading.Event()
        stop.set()
        with patch.object(raw.oak_app_control, 'start_installed') as start:
            raw.prepare_rgb('cli', 'ip', '', stop)
            start.assert_not_called()


    def test_running_without_frames_still_times_out(self):
        clock = [0.]
        stop = Mock()
        stop.is_set.return_value = False
        def wait(seconds):
            clock[0] += 61
            return False
        stop.wait.side_effect = wait
        with (patch.object(raw.time, 'monotonic', side_effect=lambda: clock[0]),
              patch.object(raw, 'rgb_status', side_effect=OSError()),
              patch.object(raw.oak_app_control, 'status', return_value=dict(state='running'))):
            with self.assertRaisesRegex(RuntimeError, '카메라 프레임'):
                raw.wait_rgb_ready('cli', 'ip', '', stop, Mock())


    def test_building_status_is_explicitly_available_to_rgb_connection(self):
        payload = [dict(identifier=raw.IDENTIFIER, app_id='rgb', status='building')]
        with patch.object(raw.oak_app_control, 'command', return_value=json.dumps(payload)):
            result = raw.oak_app_control.status('cli', 'ip', identifier=raw.IDENTIFIER,
                                                allow_preparing=True)
            self.assertEqual(result['state'], 'preparing')

    def test_status_distinguishes_ai_and_rgb_apps(self):
        payload = [dict(identifier=raw.oak_app_control.IDENTIFIER, app_id='ai', status='stopped'),
                   dict(identifier=raw.IDENTIFIER, app_id='rgb', status='running')]
        with patch.object(raw.oak_app_control, 'command', return_value=json.dumps(payload)):
            self.assertEqual(raw.oak_app_control.status('cli', 'ip')['state'], 'stopped')
            self.assertEqual(raw.oak_app_control.status('cli', 'ip', identifier=raw.IDENTIFIER)['app_id'], 'rgb')

    def test_pipeline_creates_only_camera_and_encoder(self):
        dai = Mock()
        pipeline, queue = rgb_app.build_pipeline(dai, object())
        self.assertEqual([call.args[0] for call in pipeline.create.call_args_list],
                         [dai.node.Camera, dai.node.VideoEncoder])
        camera = pipeline.create.return_value.build.return_value
        self.assertEqual(camera.requestOutput.call_args.kwargs['fps'], 30)
        self.assertEqual(pipeline.create.return_value.out.createOutputQueue.call_args.kwargs,
                         dict(maxSize=4, blocking=True))

    def test_live_status_marks_stalled_camera_unhealthy(self):
        state = dict(mode='rgb_only', source='camera', ai_enabled=False,
                     source_epoch='epoch', mxid='cam01', frames=1, last_frame_at=time.monotonic())
        lock = threading.Lock()
        server = rgb_app.status_server(state, lock, port=0)
        try:
            url = f'http://127.0.0.1:{server.server_port}/status'
            with urlopen(url) as response:
                self.assertTrue(json.load(response)['ok'])
            with lock:
                state['last_frame_at'] = time.monotonic()-4
            with urlopen(url) as response:
                self.assertFalse(json.load(response)['ok'])
        finally:
            server.shutdown()
            server.server_close()

    def test_ai_service_or_wrong_camera_cannot_be_used_as_raw_input(self):
        good = dict(ok=True, mode='rgb_only', source='camera', ai_enabled=False,
                    source_epoch='epoch', mxid='cam01')
        for change in (dict(mode='full'), dict(ai_enabled=True), dict(source='pc'),
                       dict(mxid='wrong'), dict(source_epoch=''), dict(ok=False)):
            opener = MagicMock()
            opener.open.return_value.__enter__.return_value.read.return_value = json.dumps(dict(good, **change)).encode()
            with self.subTest(change=change), patch.object(raw, 'build_opener', return_value=opener):
                with self.assertRaises(RuntimeError):
                    raw.rgb_status('192.0.2.1', 'cam01')



    def test_recording_blocks_return_to_ai(self):
        tab = object.__new__(SixCameraTab)
        tab.extra_sources = {0: Mock()}
        tab.recorder = Mock(active=True)
        with patch('six_camera_tab.messagebox.showwarning'):
            self.assertFalse(tab.release_cam01_for_perception())
        tab.extra_sources[0].close.assert_not_called()

    def test_cam01_preview_never_uses_perception_rgb(self):
        # No CAM01 receiver: even a connected AI frame must not enable raw recording.
        tab = object.__new__(SixCameraTab)
        tab.owner = Mock()
        tab._drain_discovery = Mock()
        tab.assign_button = Mock()
        tab.group = None
        tab.extra_sources = {}
        tab.cameras = [dict(name='CAM01', preview=Mock(), status=Mock(), kind=Mock(),
                            ip=Mock(get=lambda: ''), serial=Mock(get=lambda: ''), selected=Mock(get=lambda: True),
                            check=Mock(), connect=Mock(), disconnect=Mock())]
        tab._render_preview = Mock()
        tab.cameras[0]['kind'].get.return_value = 'OAK4 공유'
        tab.start_button = Mock()
        tab.stop_button = Mock()
        tab.connect_button = Mock()
        tab.selection_status = Mock()
        tab.update_preview(True, object())
        self.assertEqual(tab._render_preview.call_args.args[1:], (False, None))
        self.assertEqual(tab.start_button.configure.call_args.kwargs['state'], 'disabled')

    def test_six_virtual_workers_record_and_report_rates(self):
        sources = [OakdRgbSource(f'CAM{i:02d}', virtual=True, width=160, height=96, fps=15)
                   for i in range(1, 7)]
        group = None
        try:
            for source in sources:
                source.start()
            deadline = time.monotonic()+15
            while not all(s.snapshot()[0] for s in sources) and time.monotonic()<deadline:
                time.sleep(.02)
            self.assertTrue(all(s.snapshot()[0] for s in sources))
            with tempfile.TemporaryDirectory() as directory:
                specs = [dict(camera=s.name, model='VIRTUAL TEST', input_kind='virtual',
                              source=s, guard=s.guard, ip='') for s in sources]
                group = RecordingGroup(specs, Path(directory)/'six', fps=10)
                group.start()
                deadline = time.monotonic()+6
                while min(r.snapshot()['frames'] for r in group.recorders.values())<12 and time.monotonic()<deadline:
                    time.sleep(.02)
                self.assertTrue(group.stop(timeout=6))
                self.assertFalse(group.error)
                self.assertEqual(len(list(group.session.glob('*/*.mp4'))), 6)
                for path in group.session.glob('*/session.json'):
                    result = json.loads(path.read_text())
                    self.assertEqual(result['verification'], 'passed')
                    self.assertGreater(result['receive_fps'], 0)
                    self.assertGreater(result['saved_fps'], 0)
                    self.assertGreaterEqual(result['skipped_source_frames'], 0)
        finally:
            if group:
                group.stop(timeout=6)
            for source in sources:
                source.close()
