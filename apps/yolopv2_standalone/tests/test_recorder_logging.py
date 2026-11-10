"""Recorder controls share the video and telemetry recording lifecycle."""
import sys
import unittest
import tempfile
import time
import threading
import numpy as np
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from oak_recorder import RecorderApp, CaptureSource


class RecorderLoggingTests(unittest.TestCase):
    def test_owner_catchup_write_does_not_block_shared_frame_snapshot(self):
        """녹화 파일 쓰기가 느려도(따라잡기 쓰기 중) 다른 소비자의 프레임 스냅샷 조회는 막히지 않는다."""
        source=CaptureSource('test','test','')
        source.connected=True;source.latest_frame=np.zeros((48,64,3),np.uint8)
        source.last_frame_at=time.monotonic()
        writing=threading.Event()
        def slow(frame):writing.set();time.sleep(.03)
        source.writer=Mock();source.writer.write.side_effect=slow
        # 녹화 시작을 2.9초 전으로 두어, 첫 쓰기에서 밀린 프레임을 따라잡게 만든다.
        now=time.monotonic();source.record_started_at=now-2.9
        capture=Mock();capture.isOpened.return_value=True
        pixels=source.latest_frame
        capture.read.return_value=(True,pixels)
        with patch('oak_recorder.cv2.VideoCapture',return_value=capture):
            source.start()
            try:
                self.assertTrue(writing.wait(1))
                began=time.monotonic();connected,frame,_,_=source.record_snapshot();elapsed=time.monotonic()-began
                self.assertTrue(connected);self.assertIsNotNone(frame)
                self.assertLess(elapsed,.05)
                self.assertTrue(source.thread.is_alive())
            finally:source.close()

    def test_short_shared_receiver_reconnect_is_a_source_change(self):
        """공유 수신기가 잠깐이라도 재연결되면 녹화는 source_changed 오류로 끝난다(끊긴 구간을 이어 붙이지 않음)."""
        from shared_rgb_recorder import SharedRgbRecorder
        source=CaptureSource('test','test','')
        source.connected=True;source.latest_frame=np.zeros((48,64,3),np.uint8)
        source.last_frame_at=time.monotonic()
        with tempfile.TemporaryDirectory() as folder:
            record=SharedRgbRecorder(source,Path(folder)/'session','test',guard=lambda ip:{})
            record.start()
            deadline=time.monotonic()+2
            while record.snapshot()['frames']<2 and time.monotonic()<deadline:time.sleep(.01)
            with source.lock:source.reconnects+=1
            record.thread.join(2)
            self.assertEqual(record.snapshot()['error_kind'],'source_changed')

    def test_high_rate_recording_keeps_ten_fps_wall_time(self):
        """입력이 10/15/20fps여도 녹화는 실제 시간 기준 10fps(1초에 11장)로 저장된다."""
        for input_fps in (10,15,20):
            source=CaptureSource('test','test','')
            source.writer=Mock();source.record_started_at=100.
            for i in range(input_fps+1):
                source._write_recording_frame(object(),100.+i/input_fps)
            self.assertEqual(source.writer.write.call_count,11)
            self.assertEqual(source.frames_received,0)  # Never creates inference frames.

    def app(self):
        """Tk 없이 RecorderApp을 만들고 4개 소스와 버튼을 Mock으로 채운다. rgb/yolopv2만 녹화 대상으로 선택."""
        app=RecorderApp.__new__(RecorderApp)
        app.device_ip='192.0.2.1';app.output_dir=Path('unused-recordings')
        app.csv_logger=None;app.recording=False;app.recording_keys=[];app.unsaved_paths={}
        app.rgb_url_override=app.result_url_override=app.bev_url_override=None
        app.sources={key:Mock(label=key) for key in ('rgb','yolopv2','bev','depth')}
        for source in app.sources.values():
            source.record_error=''
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
        """연결만 해서는 CSV 텔레메트리 기록을 시작하지 않는다."""
        app=self.app();app._begin_receivers(app.device_ip)
        logger.assert_not_called()
        self.assertIsNone(app.csv_logger)
        for source in app.sources.values():source.start.assert_called_once()

    @patch('oak_recorder.FrameCsvLogger')
    def test_success_stop_and_new_recording(self,logger):
        """녹화 시작은 한 번만 적용되고, 정지 후 다시 시작하면 새 CSV 로거를 쓴다."""
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
        """두 번째 영상 녹화 시작이 실패하면 먼저 시작한 영상도 되돌리고 CSV는 만들지 않는다."""
        app=self.app();app.sources['yolopv2'].start_recording.return_value=False
        app.start_recording()
        logger.assert_not_called()
        app.sources['rgb'].stop_recording.assert_called_once()
        self.assertFalse(app.recording)

    @patch('oak_recorder.messagebox')
    @patch('oak_recorder.FrameCsvLogger')
    def test_unavailable_video_creates_no_csv(self,logger,dialogs):
        """영상이 연결되지 않은 상태에서는 CSV도 만들지 않는다."""
        app=self.app();app.sources['rgb'].snapshot.return_value=(False,None)
        app.start_recording();logger.assert_not_called()

    @patch('oak_recorder.FrameCsvLogger')
    def test_window_close_stops_active_recording(self,logger):
        """창을 닫으면 진행 중인 녹화를 정지한 뒤 창을 없앤다."""
        app=self.app();app.start_recording();app._close()
        self.assertTrue(logger.return_value.close.called)
        self.assertFalse(app.recording)
        app.root.destroy.assert_called_once()

    @patch('oak_recorder.FrameCsvLogger')
    def test_stop_saves_both_videos_beside_csv(self,logger):
        """정지 시 두 영상을 CSV와 같은 세션 폴더에 정해진 이름으로 옮긴다."""
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
        """자동 저장(이동)이 실패하면 원본 영상을 남겨 두고 재시도용 목록에 기록한다."""
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
