"""Recorder controls share the video and telemetry recording lifecycle."""
import sys
import unittest
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from oak_recorder import RecorderApp, CaptureSource


class RecorderLoggingTests(unittest.TestCase):
    def test_high_rate_recording_keeps_ten_fps_wall_time(self):
        for input_fps in (10,15,20):
            source=CaptureSource('test','test','')
            source.writer=Mock();source.record_started_at=100.
            for i in range(input_fps+1):
                source._write_recording_frame(object(),100.+i/input_fps)
            self.assertEqual(source.writer.write.call_count,11)
            self.assertEqual(source.frames_received,0)  # Never creates inference frames.

    def app(self):
        app=RecorderApp.__new__(RecorderApp)
        app.device_ip='192.0.2.1';app.output_dir=Path('unused-recordings')
        app.csv_logger=None;app.recording=False;app.recording_keys=[];app.unsaved_paths={}
        app.rgb_url_override=app.result_url_override=app.bev_url_override=None
        app.sources={key:Mock(label=key) for key in ('rgb','yolopv2','bev','depth')}
        for source in app.sources.values():
            source.snapshot.return_value=(True,object())
            source.start_recording.return_value=True
            source.stop_recording.return_value=None
        for key in ('root','start_button','stop_button','save_button','connect_button',
                    'rgb_check','result_check','status_text'):
            setattr(app,key,Mock())
        app._selected_keys=lambda:['rgb','yolopv2']
        return app

    @patch('oak_recorder.FrameCsvLogger')
    def test_connect_does_not_start_csv(self,logger):
        app=self.app();app._begin_receivers(app.device_ip)
        logger.assert_not_called()
        self.assertIsNone(app.csv_logger)
        for source in app.sources.values():source.start.assert_called_once()

    @patch('oak_recorder.FrameCsvLogger')
    def test_success_stop_and_new_recording(self,logger):
        first,second=Mock(),Mock();logger.side_effect=[first,second]
        app=self.app();app.start_recording();app.start_recording()
        self.assertEqual(logger.call_count,1)
        first.start.assert_called_once()
        app.stop_recording();first.close.assert_called_once()
        self.assertFalse(app.recording)
        app.start_recording();second.start.assert_called_once()
        self.assertIs(app.csv_logger,second)

    @patch('oak_recorder.messagebox')
    @patch('oak_recorder.FrameCsvLogger')
    def test_failed_second_video_rolls_back_without_csv(self,logger,dialogs):
        app=self.app();app.sources['yolopv2'].start_recording.return_value=False
        app.start_recording()
        logger.assert_not_called()
        app.sources['rgb'].stop_recording.assert_called_once()
        self.assertFalse(app.recording)

    @patch('oak_recorder.messagebox')
    @patch('oak_recorder.FrameCsvLogger')
    def test_unavailable_video_creates_no_csv(self,logger,dialogs):
        app=self.app();app.sources['rgb'].snapshot.return_value=(False,None)
        app.start_recording();logger.assert_not_called()

    @patch('oak_recorder.FrameCsvLogger')
    def test_window_close_stops_active_recording(self,logger):
        app=self.app();app.start_recording();app._close()
        self.assertTrue(logger.return_value.close.called)
        self.assertFalse(app.recording)
        app.root.destroy.assert_called_once()

    @patch('oak_recorder.FrameCsvLogger')
    def test_stop_saves_both_videos_beside_csv(self,logger):
        root=Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory(dir=root/'artifacts') as folder:
            folder=Path(folder);session=folder/'session';session.mkdir()
            (session/'frames.csv').write_text('frame_seq\n1\n')
            logger.return_value.path=session
            app=self.app();app.start_recording()
            for key in ('rgb','yolopv2'):
                source=folder/f'{key}.mp4';source.write_bytes(key.encode())
                app.sources[key].stop_recording.return_value=source
            app.stop_recording()
            self.assertFalse(app.unsaved_paths)
            for key in ('rgb','yolopv2'):
                self.assertEqual((session/f'oak4dpro_{app.recording_stamp}_{key}.mp4').read_bytes(),key.encode())
            self.assertTrue((session/'frames.csv').exists())
            self.assertEqual(logger.call_args.args[1],root/'artifacts'/'telemetry')

    @patch('oak_recorder.messagebox')
    @patch('oak_recorder.shutil.move',side_effect=OSError('disk unavailable'))
    @patch('oak_recorder.FrameCsvLogger')
    def test_auto_save_failure_retains_video_for_retry(self,logger,move,dialogs):
        root=Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory(dir=root/'artifacts') as folder:
            folder=Path(folder);source=folder/'rgb.mp4';source.write_bytes(b'video')
            logger.return_value.path=folder/'session'
            app=self.app();app.start_recording()
            app.sources['rgb'].stop_recording.return_value=source
            app.stop_recording()
            self.assertEqual(app.unsaved_paths,{'rgb':source})
            self.assertTrue(source.exists())
            self.assertFalse(app.recording)


if __name__=='__main__':unittest.main()
