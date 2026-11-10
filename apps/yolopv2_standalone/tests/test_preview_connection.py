"""멀티뷰 앱의 미리보기 연결 준비(_prepare_connection) 테스트."""
from pathlib import Path
import queue
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
import oak_recorder as recorder


class PreviewConnectionTests(unittest.TestCase):
    def app(self):
        """Tk 초기화 없이 RecorderApp 인스턴스를 만들고, RTSP 포트는 항상 열려 있다고 가정한다."""
        app=recorder.RecorderApp.__new__(recorder.RecorderApp)
        app.connection_events=queue.Queue()
        app._rtsp_ports_open=lambda ip:True
        return app

    def test_recovery_ack_precedes_ready_event(self):
        """preview 복구 응답을 받은 뒤에야 ready 이벤트가 올라간다."""
        app=self.app()
        def prepare(ip):
            self.assertTrue(app.connection_events.empty())
            return dict(preview_recovered=True)
        with patch.object(recorder.oak_app_control,'start_installed'),patch.object(recorder,'prepare_preview',side_effect=prepare) as control:
            self.assertTrue(app._prepare_connection('192.0.2.1',''))
        control.assert_called_once_with('192.0.2.1')
        self.assertEqual(app.connection_events.get_nowait(),('ready','192.0.2.1'))

    def test_failed_recovery_does_not_report_ready(self):
        """복구 요청이 실패하면 ready 대신 error 이벤트 하나만 보고한다."""
        app=self.app()
        with patch.object(recorder.oak_app_control,'start_installed'),patch.object(recorder,'prepare_preview',side_effect=RuntimeError('unsupported')):
            self.assertFalse(app._prepare_connection('192.0.2.1',''))
        event,detail=app.connection_events.get_nowait()
        self.assertEqual(event,'error');self.assertIn('unsupported',detail)
        self.assertTrue(app.connection_events.empty())
