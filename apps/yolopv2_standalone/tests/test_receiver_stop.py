"""App stop ends receiver retries; a new explicit connection creates fresh receivers."""
from pathlib import Path
import queue
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from oak_recorder import CaptureSource, RecorderApp


class ReceiverStopTests(unittest.TestCase):
    def app(self):
        """Tk 없이 RecorderApp을 만들고 4개 수신기(rgb/yolopv2/bev/depth)와 위젯을 Mock으로 채운다."""
        app=object.__new__(RecorderApp)
        app.sources={key:CaptureSource(key,key,'unused') for key in ('rgb','yolopv2','bev','depth')}
        app.recording=False
        app.file_sender=None
        app.connect_requested=True
        app.connection_launch_pending=False
        app.connection_events=queue.SimpleQueue()
        app.six_camera_tab=Mock()
        app.six_camera_tab.extra_sources={}
        for name in ('connect_button','start_button','ip_entry','password_entry','discover_button',
                     'status_text','app_led','app_state_text'):
            setattr(app,name,Mock())
        app.ip_text=Mock(get=lambda:'192.0.2.1')
        return app

    def result(self,app,state,**extra):
        """카메라 앱 폴링 결과 이벤트를 넣고 이벤트 큐를 처리한다."""
        app.connection_events.put(('app_result',dict(ip='192.0.2.1',action='poll',state=state,**extra)))
        app._drain_connection_events()

    def test_confirmed_stop_retires_all_four_receivers_once(self):
        """앱 정지가 확인되면 기존 수신기 4개를 모두 멈추고 새 수신기로 교체한다. 다시 stopped가 와도 재교체하지 않는다."""
        app=self.app()
        old=list(app.sources.values())
        for source in old:source.started=True
        self.result(app,'stopped')
        self.assertTrue(all(s.stop_event.is_set() for s in old))
        self.assertFalse(app.connect_requested)
        self.assertTrue(all(not s.started and not s.stop_event.is_set() for s in app.sources.values()))
        fresh=list(app.sources.values())
        self.result(app,'stopped')
        self.assertEqual(fresh,list(app.sources.values()))

    def test_manual_stop_signals_receivers_before_remote_command(self):
        """수동 정지 시 원격 명령을 보내기 전에 수신기에 먼저 정지 신호를 준다."""
        app=self.app()
        old=list(app.sources.values())
        app.app_action_busy=False
        app.password_text=Mock(get=lambda:'')
        app.six_camera_tab.is_recording=False
        checked=threading.Event()
        observed=[]
        def query(*args,**kwargs):
            observed.append(all(s.stop_event.is_set() for s in old))
            checked.set()
            return dict(state='stopped',app_id='ai')
        with (patch('oak_recorder.load_password',return_value=''),
              patch('oak_recorder.oak_app_control.status',side_effect=query),
              patch('oak_recorder.messagebox.showinfo')):
            app._camera_app_action('stop')
            self.assertTrue(checked.wait(1))
            deadline=time.monotonic()+1
            while app.connection_events.empty() and time.monotonic()<deadline:time.sleep(.01)
            app._drain_connection_events()
        self.assertEqual(observed,[True])
        self.assertFalse(app.connect_requested)

    def test_unknown_status_does_not_stop_a_running_receiver(self):
        """상태를 알 수 없으면 실행 중인 수신기를 멈추지 않는다."""
        app=self.app()
        self.result(app,'unknown')
        self.assertTrue(app.connect_requested)
        self.assertTrue(all(not s.stop_event.is_set() for s in app.sources.values()))

    def test_rgb_stop_is_independent_of_ai_receivers(self):
        """RGB 전용 앱 정지는 AI 수신기에 영향을 주지 않고, 추가 소스 close는 한 번만 호출된다."""
        app=self.app()
        app.connect_requested=False
        raw=Mock(stop_event=threading.Event())
        raw.close.side_effect=lambda **kwargs:raw.stop_event.set()
        app.six_camera_tab.extra_sources={0:raw}
        old=list(app.sources.values())
        self.result(app,'stopped',raw_state='stopped')
        raw.close.assert_called_once_with(timeout=0)
        self.assertEqual(old,list(app.sources.values()))
        self.result(app,'stopped',raw_state='stopped')
        raw.close.assert_called_once()

    def test_failed_open_retry_stops_and_manual_fresh_start_works(self):
        """열기 실패 재시도 루프는 close() 후 멈추고, 새로 만든 수신기는 다시 정상 시도한다."""
        opened=threading.Event()
        failed=Mock()
        failed.isOpened.return_value=False
        def factory(*args):
            opened.set()
            return failed
        first=CaptureSource('rgb','rgb','unused')
        second=CaptureSource('rgb','rgb','unused')
        with patch('oak_recorder.cv2.VideoCapture',side_effect=factory) as constructor:
            try:
                first.start()
                self.assertTrue(opened.wait(1))
                first.close()
                count=constructor.call_count
                time.sleep(1.1)  # Longer than the real retry interval.
                self.assertEqual(constructor.call_count,count)
                opened.clear()
                second.start()
                self.assertTrue(opened.wait(1))
                second.close()
                self.assertGreater(constructor.call_count,count)
                self.assertGreaterEqual(failed.release.call_count,2)
            finally:
                first.close();second.close()

    def test_stopping_during_open_prevents_read_and_retry(self):
        """열기 도중 정지 신호가 오면 열린 캡처를 읽지 않고 바로 해제한다."""
        opening=threading.Event()
        finish=threading.Event()
        capture=Mock()
        def factory(*args):
            opening.set()
            finish.wait(1)
            return capture
        source=CaptureSource('rgb','rgb','unused')
        with patch('oak_recorder.cv2.VideoCapture',side_effect=factory) as constructor:
            try:
                source.start()
                self.assertTrue(opening.wait(1))
                source.stop_event.set()
                finish.set()
                source.close()
                self.assertFalse(source.thread.is_alive())
                constructor.assert_called_once()
                capture.read.assert_not_called()
                capture.release.assert_called_once()
            finally:
                finish.set();source.close()
