"""Independent recording and failure finalization, using decodable MP4 files."""
import json
from pathlib import Path
import sys
import tempfile
import time
import socketserver
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from shared_rgb_recorder import SharedRgbRecorder,camera_input_status,RecorderFault
from video_transport import read_message,send_message


class StatusServer(socketserver.ThreadingTCPServer):
    """카메라 입력 상태 조회에 미리 정한 응답을 순서대로 돌려주는 테스트용 TCP 서버.

    응답 종류: dict=정상 응답, None=응답 없이 끊기, bytes=깨진 원시 데이터, float=그 초만큼 지연 후 정상 응답.
    """
    daemon_threads=True
    allow_reuse_address=True
    def __init__(self,replies):
        self.replies=replies;self.calls=0;self.counter_lock=threading.Lock()
        class Handler(socketserver.StreamRequestHandler):
            def handle(handler):
                read_message(handler.rfile)
                with self.counter_lock:
                    index=self.calls;self.calls+=1
                reply=self.replies[min(index,len(self.replies)-1)]
                if reply is None:return
                if isinstance(reply,bytes):  # raw, e.g. a header cut off mid-line
                    handler.request.sendall(reply);return
                if isinstance(reply,float):
                    time.sleep(reply);reply=dict(ok=True,source='camera',source_epoch=1)
                try:send_message(handler.request,reply)
                except OSError:pass
        super().__init__(('127.0.0.1',0),Handler)
        self.thread=threading.Thread(target=self.serve_forever,daemon=True);self.thread.start()
    def close(self):self.shutdown();self.server_close();self.thread.join(2)


class Source:
    """공유 RGB 수신기를 흉내 내는 가짜 소스. 호출마다 시퀀스가 1씩 증가한다."""
    url='rtsp://example/rgb'
    def __init__(self):
        self.connected=True
        self.writer=object()
        self.count=0
    def record_snapshot(self):
        self.count+=1
        return self.connected,np.full((48,64,3),100,np.uint8),self.count,time.monotonic()


class SharedRgbRecorderTests(unittest.TestCase):
    def test_writer_is_released_before_waiting_for_status_monitor(self):
        """정지 시 상태 확인(guard)이 지연 중이어도 영상 writer는 먼저 해제된다."""
        in_guard=threading.Event();finish_guard=threading.Event();released=threading.Event();calls=[0]
        def guard(ip):
            calls[0]+=1
            if calls[0]>1:in_guard.set();finish_guard.wait(2)
            return {}
        writer=Mock();writer.isOpened.return_value=True;writer.release.side_effect=released.set
        with tempfile.TemporaryDirectory() as folder,patch('shared_rgb_recorder.cv2.VideoWriter',return_value=writer):
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=guard,
                                     guard_interval=.05,verify_output=False)
            record.start()
            try:
                self.assertTrue(in_guard.wait(1));record.stop()
                self.assertTrue(released.wait(.3))
            finally:
                finish_guard.set();record.stop(timeout=3)

    def tcp_record(self,folder,replies):
        """StatusServer를 띄우고 실제 TCP로 상태를 확인하는 녹화기를 만든다."""
        server=StatusServer(replies);self.addCleanup(server.close)
        guard=lambda ip:camera_input_status(ip,server.server_address[1],timeout=.05)
        return SharedRgbRecorder(Source(),Path(folder)/'session','127.0.0.1',guard=guard,
                                 guard_interval=.03,guard_grace_s=1.)

    def test_real_tcp_one_timeout_recovers_without_stopping_video(self):
        """상태 조회가 한 번 지연/끊김/깨짐이어도 녹화는 계속되고, 경고와 복구 이벤트가 기록된다."""
        status=dict(ok=True,source='camera',source_epoch=1)
        for interruption in (.2,None,b'{"ok": tr'):
          with self.subTest(interruption=interruption),tempfile.TemporaryDirectory() as folder:
            record=self.tcp_record(folder,[status,interruption,status])
            record.start()
            try:self.wait(record,lambda s:s['frames']>=8 or s['error'])
            finally:record.stop(timeout=3)
            result=record.snapshot()
            self.assertEqual(result['state'],'저장 완료',result)
            self.assertTrue(result['source_verified'])
            events=[json.loads(x) for x in (record.session/'recording_events.jsonl').read_text().splitlines()]
            self.assertTrue(any(x['event']=='source_status_warning' for x in events))
            self.assertTrue(any(x['event']=='source_status_recovered' for x in events))

    def test_unverified_time_budget_stops_even_before_retry_count_limit(self):
        """재시도 횟수 한도 전이라도 미확인 상태가 허용 시간을 넘으면 녹화를 멈춘다."""
        with tempfile.TemporaryDirectory() as folder:
            record=self.tcp_record(folder,[dict(ok=True,source='camera',source_epoch=1),.2])
            record.guard_failure_limit=99;record.guard_grace_s=.15
            record.start();record.thread.join(3)
            self.assertEqual(record.snapshot()['error_kind'],'source_verification_expired')

    def test_real_tcp_three_timeouts_stop_with_precise_reason(self):
        """연속 3회 타임아웃이면 source_status_timeout 사유로 멈춘다."""
        with tempfile.TemporaryDirectory() as folder:
            record=self.tcp_record(folder,[dict(ok=True,source='camera',source_epoch=1),.2])
            record.start();record.thread.join(3)
            result=record.snapshot()
            self.assertEqual(result['error_kind'],'source_status_timeout',result)
            self.assertEqual(result['consecutive_guard_failures'],3)
            self.assertFalse(result['source_verified'])
            self.assertIn('source_status_failed',self.event_names(record))

    def test_repeated_corrupted_replies_stop_as_network_failure(self):
        """깨진 응답이 반복되면 네트워크 실패(source_status_network)로 멈춘다."""
        with tempfile.TemporaryDirectory() as folder:
            record=self.tcp_record(folder,[dict(ok=True,source='camera',source_epoch=1),b'{"ok": tr'])
            record.start();record.thread.join(3)
            result=record.snapshot()
            self.assertEqual(result['error_kind'],'source_status_network',result)
            self.assertEqual(result['consecutive_guard_failures'],3)
            self.assertIn('source_status_failed',self.event_names(record))

    def test_real_tcp_epoch_change_and_explicit_rejection_are_immediate(self):
        """source epoch 변경이나 명시적 거부(ok=False)는 재시도 없이 즉시 source_changed로 멈춘다."""
        for changed in (dict(ok=True,source='camera',source_epoch=2),dict(ok=False,source='camera')):
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as folder:
                record=self.tcp_record(folder,[dict(ok=True,source='camera',source_epoch=1),changed])
                record.start();record.thread.join(3)
                self.assertEqual(record.snapshot()['error_kind'],'source_changed')
                self.assertEqual(record.snapshot()['consecutive_guard_failures'],0)
                names=self.event_names(record)
                self.assertIn('source_status_failed',names)
                self.assertLess(names.index('source_status_failed'),names.index('finish'))

    def event_names(self,record):
        """녹화 이벤트 로그(recording_events.jsonl)의 이벤트 이름 목록."""
        return [json.loads(x)['event'] for x in (record.session/'recording_events.jsonl').read_text().splitlines()]

    def test_first_error_survives_release_and_metadata_failures(self):
        """마무리 과정에서 추가 오류가 나도 최초 오류 원인이 유지되고, 마무리 오류는 따로 기록된다."""
        writer=Mock();writer.isOpened.return_value=True;writer.release.side_effect=RuntimeError('release failed')
        with tempfile.TemporaryDirectory() as folder,patch('shared_rgb_recorder.cv2.VideoWriter',return_value=writer):
            source=Source();record=SharedRgbRecorder(source,Path(folder)/'session','test',guard=lambda ip:{})
            record.start();self.wait(record,lambda s:s['frames']>=2)
            with patch.object(Path,'write_text',side_effect=OSError('metadata failed')):
                source.connected=False;record.thread.join(3)
            result=record.snapshot()
            self.assertEqual(result['error_kind'],'source_receive')
            self.assertEqual([e['kind'] for e in result['finalize_errors']],['storage_finalize','metadata_write'])

    def test_low_disk_space_refuses_start(self):
        """디스크 공간이 부족하면 파일을 만들지 않고 시작을 거부한다."""
        with tempfile.TemporaryDirectory() as folder,patch('shared_rgb_recorder.shutil.disk_usage',return_value=SimpleNamespace(free=0)):
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=lambda ip:{})
            record.start();record.thread.join(3)
            self.assertEqual(record.snapshot()['error_kind'],'storage_space')
            self.assertFalse((record.session/record.filename).exists())

    def test_writer_without_file_progress_is_not_reported_as_saved(self):
        """writer가 열렸어도 파일 크기가 늘지 않으면 저장 성공으로 보고하지 않는다."""
        writer=Mock();writer.isOpened.return_value=True
        with tempfile.TemporaryDirectory() as folder,patch('shared_rgb_recorder.cv2.VideoWriter',return_value=writer):
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=lambda ip:{},
                                     health_interval=.03,storage_startup_grace_s=.15)
            record.start();record.thread.join(3)
            self.assertEqual(record.snapshot()['error_kind'],'storage_no_progress')

    def test_slow_successful_control_reply_does_not_stop_video(self):
        """상태 응답이 느려도(1.2초) 성공이면 녹화를 멈추지 않는다."""
        calls=[0]
        def guard(ip):
            calls[0]+=1
            if calls[0]==2:time.sleep(1.2)
            return dict(source='camera',source_epoch=1)
        with tempfile.TemporaryDirectory() as folder:
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=guard)
            record.start()
            try:self.wait(record,lambda s:s['frames']>=30 or s['error'])
            finally:record.stop(timeout=3)
            self.assertEqual(record.snapshot()['state'],'저장 완료',record.snapshot())
            self.assertGreaterEqual(record.snapshot()['max_guard_ms'],1200)
            self.assertGreaterEqual(record.snapshot()['frames'],30)

    def test_actual_write_stall_has_specific_metrics_and_duration(self):
        """실제 프레임 쓰기가 멈추면 일정 초과(recording_schedule_overrun)로 중단하고 지표와 길이를 기록한다."""
        writer=Mock()
        writer.isOpened.return_value=True
        writer.write.side_effect=lambda frame:time.sleep(1.2)
        with tempfile.TemporaryDirectory() as folder,patch('shared_rgb_recorder.cv2.VideoWriter',return_value=writer):
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=lambda ip:{})
            record.start();record.thread.join(4)
            result=record.snapshot()
            self.assertEqual(result['error_kind'],'recording_schedule_overrun')
            self.assertGreaterEqual(result['max_write_ms'],1200)
            manifest=json.loads((Path(folder)/'session/session.json').read_text())
            self.assertGreater(manifest['duration_s'],1)
            events=[json.loads(line) for line in (Path(folder)/'session/recording_events.jsonl').read_text().splitlines()]
            self.assertEqual(events[-1]['event'],'finish')

    def wait(self, recorder, predicate):
        """predicate(snapshot)가 참이 될 때까지 최대 4초 기다린다."""
        deadline=time.monotonic()+4
        while not predicate(recorder.snapshot()) and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertTrue(predicate(recorder.snapshot()),recorder.snapshot())

    def test_independent_video_and_manifest_without_touching_shared_writer(self):
        """공유 소스의 기존 writer를 건드리지 않고 별도 MP4와 manifest를 만들며, 프레임 수가 일치한다."""
        with tempfile.TemporaryDirectory() as folder:
            source=Source();existing=source.writer
            record=SharedRgbRecorder(source,Path(folder)/'session','127.0.0.1',
                size=(32,24),guard=lambda ip:dict(source='camera'))
            record.start()
            try:self.wait(record,lambda s:s['frames']>=4)
            finally:self.assertTrue(record.stop(timeout=3))
            self.assertIs(source.writer,existing)
            self.assertEqual(record.snapshot()['state'],'저장 완료')
            cap=cv2.VideoCapture(record.snapshot()['path'])
            ok,frame=cap.read();count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
            self.assertTrue(ok);self.assertEqual(frame.shape[:2],(24,32))
            self.assertGreaterEqual(count,4)
            manifest=json.loads((Path(folder)/'session/session.json').read_text())
            self.assertEqual(manifest['frames'],count)
            self.assertFalse(manifest['synchronized'])

    def test_pc_source_rejected_before_file_creation(self):
        """PC 입력이면 파일을 만들기 전에 거부한다."""
        with tempfile.TemporaryDirectory() as folder:
            guard=Mock(side_effect=RuntimeError('PC source'))
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=guard)
            record.start();record.thread.join(3)
            self.assertEqual(record.snapshot()['state'],'오류')
            self.assertFalse((Path(folder)/'session').exists())

    def test_disconnect_finalizes_partial_file(self):
        """연결이 끊기면 오류로 끝나지만 부분 파일은 재생 가능하게 마무리된다."""
        with tempfile.TemporaryDirectory() as folder:
            source=Source()
            record=SharedRgbRecorder(source,Path(folder)/'session','test',guard=lambda ip:{})
            record.start()
            self.wait(record,lambda s:s['frames']>=3)
            source.connected=False;record.thread.join(3)
            self.assertEqual(record.snapshot()['state'],'오류')
            cap=cv2.VideoCapture(record.snapshot()['path']);ok,_=cap.read();cap.release()
            self.assertTrue(ok)

    def test_existing_session_is_not_overwritten(self):
        """이미 있는 세션 폴더는 덮어쓰지 않는다."""
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'session';path.mkdir()
            sentinel=path/'session.json';sentinel.write_text('original')
            record=SharedRgbRecorder(Source(),path,'test',guard=lambda ip:{})
            record.start();record.thread.join(3)
            self.assertEqual(record.snapshot()['state'],'오류')
            self.assertEqual(sentinel.read_text(),'original')

    def test_external_source_switch_stops_recording(self):
        """녹화 중 외부에서 입력 소스가 바뀌면 source_changed로 멈춘다."""
        with tempfile.TemporaryDirectory() as folder:
            calls=[0]
            def guard(ip):
                calls[0]+=1
                if calls[0]>1:raise RecorderFault('source_changed','PC input changed')
                return dict(source='camera')
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=guard)
            self.addCleanup(record.stop,3)
            record.start();record.thread.join(3)
            self.assertFalse(record.active)
            self.assertIn('PC input changed',record.snapshot()['error'])
            self.assertEqual(record.snapshot()['error_kind'],'source_changed')
            self.assertGreater(record.snapshot()['frames'],0)

    def test_brief_reconnection_is_detected_even_when_frames_are_available(self):
        """프레임이 계속 와도 재연결 횟수가 늘었으면 소스 변경으로 감지한다."""
        with tempfile.TemporaryDirectory() as folder:
            calls=[0]
            def guard(ip):
                calls[0]+=1
                return dict(source='camera',reconnects=0 if calls[0]==1 else 1)
            record=SharedRgbRecorder(Source(),Path(folder)/'session','test',guard=guard)
            self.addCleanup(record.stop,3)
            record.start();record.thread.join(3)
            self.assertFalse(record.active)
            self.assertEqual(record.snapshot()['state'],'오류')
            self.assertEqual(record.snapshot()['error_kind'],'source_changed')
            self.assertGreater(record.snapshot()['frames'],0)


if __name__=='__main__':unittest.main()
