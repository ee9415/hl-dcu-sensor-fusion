"""PC 영상 입력 경로(video_transport 서버 + tools/video_file_sender 송신기) 단위 테스트.

- VideoInputServer: 입력 소스(camera/pc) 선택, 소유권(owner), epoch, 최신 프레임 보관,
  표시/처리 모드 토글 등 서버 측 상태 관리를 검증한다.
- VideoFileSender: 실제 영상 파일을 읽어 서버로 전송하는 송신기의 속도 제어(pacing),
  구간 재생, 반복(loop), 일시정지, 탐색(seek), FPS 제한 동작을 검증한다.
"""
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np
from video_transport import (VideoInputServer, jpeg_size, read_message, send_message,
                             MAX_PACKET)

# 저장소 루트(…/hl-dcu-sensor-fusion). tools/ 아래 송신기 모듈을 import 하기 위해 경로에 추가한다.
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from video_file_sender import VideoFileSender, fit_frame, select_camera, parse_video_time, set_processing_mode, set_boundary_display, set_contour_display, set_lane_overlay


def jpeg(value=100):
    """단색(value) 320x180 테스트용 JPEG 바이트를 만든다."""
    return cv2.imencode('.jpg', np.full((180,320,3),value,np.uint8))[1].tobytes()


class TransportTests(unittest.TestCase):
    def test_explicit_preview_recovers_only_idle_pc_and_keeps_active_owner(self):
        """명시적 preview 요청은 '소유자가 없는 pc 모드'일 때만 camera로 복구하고,
        활성 송신자가 있으면 소유권/epoch를 건드리지 않는다."""
        from video_file_sender import prepare_preview
        token=object()
        self.command(token,'select',source='pc')
        epoch=self.hub.epoch
        # 송신자가 살아 있는 동안에는 복구하지 않는다.
        response=prepare_preview('127.0.0.1',self.port)
        self.assertFalse(response['preview_recovered'])
        self.assertEqual(response['source'],'pc')
        self.assertIs(self.hub.owner,token)
        self.assertEqual(self.hub.epoch,epoch)
        # 송신자 연결이 끊겨도 자동으로 camera로 돌아가지 않는다.
        self.hub.release(token)
        self.assertEqual(self.hub.mode,'pc')  # No silent background fallback.
        # 유휴 pc 상태에서 preview 요청 → camera 복구 + epoch 증가.
        epoch=self.hub.epoch
        response=prepare_preview('127.0.0.1',self.port)
        self.assertTrue(response['preview_recovered'])
        self.assertEqual(response['source'],'camera')
        self.assertEqual(self.hub.epoch,epoch+1)
        self.assertEqual(self.hub.take()[1]['input_source'],'camera')
        # 이미 camera면 다시 요청해도 아무 변화 없음(멱등).
        self.assertFalse(prepare_preview('127.0.0.1',self.port)['preview_recovered'])
        self.assertEqual(self.hub.epoch,epoch+1)
        # prepare_preview는 payload를 받으면 안 된다.
        with self.assertRaises(ValueError):self.hub.handle(token,dict(op='prepare_preview'),b'x')

    def setUp(self):
        # 포트 0 → OS가 빈 포트를 할당. 테스트마다 새 서버를 띄운다.
        self.hub = VideoInputServer(0, '127.0.0.1')
        self.port = self.hub.server.server_address[1]

    def tearDown(self):
        self.hub.close()

    def command(self, token, op, **values):
        """소켓을 거치지 않고 서버 핸들러를 직접 호출하는 헬퍼. frame 명령이면 JPEG를 첨부한다."""
        return self.hub.handle(token, dict(op=op, **values), jpeg() if op=='frame' else b'')

    def test_latest_frame_ownership_epoch_and_no_fallback(self):
        """pc 소스 소유권 배타성, 최신 프레임만 유지(이전 프레임 drop), media_epoch 변경 시
        source epoch 증가, 소유자 해제 후에도 camera로 자동 복귀하지 않음을 확인한다."""
        a,b=object(),object()
        self.command(a,'select',source='pc')
        # 다른 클라이언트는 이미 점유된 pc 소스를 선택할 수 없다.
        with self.assertRaises(ValueError):self.command(b,'select',source='pc')
        # 소비 전에 프레임 두 개가 들어오면 앞의 것은 버려진다(dropped=1).
        first=self.command(a,'frame',frame_index=3,media_time_s=.1,media_epoch=0)
        self.command(a,'frame',frame_index=3,media_time_s=.1,media_epoch=0)
        self.assertEqual(self.hub.epoch,first['source_epoch'])
        self.assertEqual(self.hub.dropped,1)
        _,meta=self.hub.take();self.assertEqual(meta['source_frame_index'],3)
        # 한 번 꺼낸 프레임은 다시 나오지 않는다.
        self.assertEqual(self.hub.take(),(None,None))
        # media_epoch가 바뀌면(루프/seek) 서버 epoch도 증가한다.
        self.command(a,'frame',frame_index=0,media_time_s=0,media_epoch=1)
        self.assertGreater(self.hub.epoch,first['source_epoch'])
        # 소유자 해제 후에도 pc 모드 유지, 남은 프레임 없음.
        self.hub.release(a)
        self.assertEqual(self.hub.mode,'pc')
        self.assertEqual(self.hub.take(),(None,None))
        # 다른 클라이언트가 camera를 선택하면 전환된다.
        self.command(b,'select',source='camera')
        self.assertEqual(self.hub.take()[1]['input_source'],'camera')
        # 소유권을 잃은 a는 더 이상 프레임을 보낼 수 없다.
        with self.assertRaises(ValueError):
            self.command(a,'frame',frame_index=1,media_time_s=.1,media_epoch=1)

    def test_protocol_partial_frame_and_size_limit(self):
        """프로토콜 파싱 오류(잘못된 JPEG, 최대 크기 초과, 잘린 payload, NaN 시간)와
        프레임 메타데이터 보관 개수 상한(256)을 검증한다."""
        self.assertEqual(jpeg_size(jpeg()),(320,180))
        with self.assertRaises(ValueError):jpeg_size(b'not a jpeg')
        # 헤더에 선언된 길이가 MAX_PACKET을 넘으면 거부.
        with self.assertRaises(ValueError):
            read_message(io.BytesIO((json.dumps(dict(protocol=1,length=MAX_PACKET+1))+'\n').encode()))
        # 선언 길이(10)보다 payload가 짧으면 EOF.
        with self.assertRaises(EOFError):
            read_message(io.BytesIO(b'{"protocol":1,"length":10}\nabc'))
        token=object();self.command(token,'select',source='pc')
        with self.assertRaises(ValueError):
            self.command(token,'frame',frame_index=1,media_time_s=float('nan'),media_epoch=0)
        # 메타데이터 캐시는 최근 256개만 유지한다.
        for i in range(300):self.hub.remember(i,dict(source_frame_index=i))
        self.assertEqual(len(self.hub.frames),256)
        self.assertEqual(self.hub.metadata(299)['source_frame_index'],299)

    def test_socket_fragmentation_and_camera_control(self):
        """실제 TCP 소켓에서 헤더가 쪼개져 도착해도 정상 처리되는지, 그리고
        select_camera 헬퍼로 camera 복귀가 되는지 확인한다."""
        with socket.create_connection(('127.0.0.1',self.port),timeout=2) as c:
            data=b'{"protocol":1,"length":0,"op":"select","source":"pc"}\n'
            # 헤더를 두 조각으로 나눠 전송(TCP 단편화 모사).
            for chunk in (data[:9],data[9:]):c.sendall(chunk)
            with c.makefile('rb') as stream:
                response,_=read_message(stream);self.assertTrue(response['ok'])
                send_message(c,dict(op='frame',frame_index=0,media_time_s=0.,media_epoch=0),jpeg())
                response,_=read_message(stream);self.assertEqual(response['received_frames'],1)
        response=select_camera('127.0.0.1',self.port)
        self.assertEqual(response['source'],'camera')

    def test_boundary_display_is_live_and_does_not_reset_input_or_processing(self):
        """경계선 표시 토글은 즉시 반영되지만 입력/처리 epoch, 처리 모드는 바꾸지 않으며,
        이미 기록된 프레임 메타데이터는 소급 변경되지 않는다."""
        publisher=object()
        self.command(publisher,'select',source='pc')
        self.command(publisher,'processing',mode='full')
        before=self.command(publisher,'frame',frame_index=8,media_time_s=.8,media_epoch=0)
        reply=set_boundary_display('127.0.0.1',False,self.port)
        self.assertFalse(reply['boundary_visible'])
        self.assertEqual(reply['source_epoch'],before['source_epoch'])
        self.assertEqual(reply['processing_epoch'],before['processing_epoch'])
        self.assertEqual(reply['processing_mode'],'full')
        # 대기 중이던 프레임은 버려지지 않고, 새 표시 설정이 반영된다.
        payload,meta=self.hub.take()
        self.assertIsNotNone(payload)
        self.assertEqual(meta['source_frame_index'],8)
        self.assertFalse(meta['boundary_visible'])
        # 과거 프레임 메타데이터는 토글 이후에도 당시 값 유지.
        self.hub.remember(1,meta)
        set_boundary_display('127.0.0.1',True,self.port)
        self.assertFalse(self.hub.metadata(1)['boundary_visible'])
        self.command(object(),'select',source='camera')
        self.assertTrue(self.hub.take()[1]['boundary_visible'])
        # visible 값은 bool만 허용.
        for bad in (1,'false',None):
            with self.assertRaises(ValueError):self.command(object(),'boundary_display',visible=bad)

    def test_contour_toggle_preserves_input_and_old_frame_metadata(self):
        """윤곽선(contour) 표시 토글도 경계선과 동일하게 입력/처리 상태를 보존하고
        과거 메타데이터를 바꾸지 않는다. 기본값은 비표시."""
        publisher=object()
        self.command(publisher,'select',source='pc')
        before=self.command(publisher,'frame',frame_index=8,media_time_s=.8,media_epoch=0)
        self.assertFalse(before['contour_visible'])
        enabled=set_contour_display('127.0.0.1',True,self.port)
        self.assertTrue(enabled['contour_visible'])
        self.assertEqual(enabled['source_epoch'],before['source_epoch'])
        self.assertEqual(enabled['processing_epoch'],before['processing_epoch'])
        self.assertEqual(enabled['boundary_visible'],before['boundary_visible'])
        _,metadata=self.hub.take()
        self.assertTrue(metadata['contour_visible'])
        self.hub.remember(42,metadata)
        set_contour_display('127.0.0.1',False,self.port)
        self.assertTrue(self.hub.metadata(42)['contour_visible'])
        for bad in (1,'false',None):
            with self.assertRaises(ValueError):self.command(object(),'contour_display',visible=bad)

    def make_video(self, folder, fps=30, count=30):
        """테스트용 MJPG AVI 생성. i번째 프레임의 밝기는 40+i 이므로
        디코딩된 픽셀 값으로 어떤 프레임이 전송됐는지 역추적할 수 있다."""
        path=Path(folder)/'input.avi'
        writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'MJPG'),fps,(320,180))
        self.assertTrue(writer.isOpened())
        for i in range(count):writer.write(np.full((180,320,3),40+i,np.uint8))
        writer.release()
        return path

    def test_lane_preview_changes_no_input_or_perception_state(self):
        """차선 오버레이(미리보기) 모드/임계값 변경은 표시 전용이다.
        입력·처리 epoch와 인지용 임계값(lane_perception_threshold=0.5)은 그대로여야 한다."""
        token=object();self.command(token,'select',source='pc')
        self.command(token,'processing',mode='full')
        before=self.command(token,'frame',frame_index=12,media_time_s=1.2,media_epoch=0)
        for mode,threshold in (('lane_mask',.2),('lane_mask',.8),('lane_score',.5),
                               ('raw_rgb',.5),('lane_bands',.05),('lane_bands_rgb',.05),
                               ('lane_compare',.05),('normal',.5)):
            reply=set_lane_overlay('127.0.0.1',mode,threshold,self.port)
            self.assertEqual(reply['source_epoch'],before['source_epoch'])
            self.assertEqual(reply['processing_epoch'],before['processing_epoch'])
            self.assertEqual(reply['lane_perception_threshold'],.5)
            self.assertEqual(reply['lane_overlay_mode'],mode)
            self.assertEqual(reply['lane_overlay_threshold'],threshold)
        # 마지막 설정(normal)이 대기 프레임 메타데이터에 반영된다.
        payload,meta=self.hub.take()
        self.assertIsNotNone(payload)
        self.assertEqual(meta['source_frame_index'],12)
        self.assertEqual(meta['lane_overlay_mode'],'normal')
        self.assertEqual(meta['lane_perception_threshold'],.5)
        # 알 수 없는 모드, bool 임계값, 범위(0~1) 밖 값은 거부.
        for mode,threshold in (('bad',.5),('lane_mask',False),('lane_score',2)):
            with self.assertRaises(ValueError):self.command(token,'lane_overlay',mode=mode,threshold=threshold)

    def test_real_file_sender_is_paced_skips_frames_and_finishes(self):
        """30fps·30프레임(1초) 영상을 실제 송신: 실시간 속도로 재생(≥0.9초)되고,
        기본 송신 FPS 제한 때문에 일부 프레임은 건너뛰며(7~11장), 끝나면 '재생 완료'."""
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder),loop=False,port=self.port)
            start=time.monotonic();sender.start();sender.thread.join(5)
            self.assertFalse(sender.thread.is_alive())
            status=sender.snapshot()
            self.assertEqual(status['state'],'재생 완료',status)
            self.assertTrue(7<=status['sent']<=11,status)
            self.assertGreaterEqual(time.monotonic()-start,.9)
            # 원본 프레임 인덱스가 전송 개수보다 크다 → 프레임을 건너뛰었다는 뜻.
            self.assertGreater(self.hub.last_frame_index,status['sent'])
            self.assertEqual(self.hub.mode,'pc')

    def test_loop_pause_stop_and_invalid_file(self):
        """반복 재생 시 media_epoch 증가, 일시정지 중 전송 중단/재개, close()로 스레드 종료,
        존재하지 않는 파일은 '오류' 상태가 되는지 확인한다."""
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=10,count=3),port=self.port)
            sender.start()
            # 짧은 영상이 한 바퀴 돌아 media_epoch>=1이 될 때까지 대기.
            end=time.monotonic()+4
            while self.hub.last_media_epoch in (None,0) and time.monotonic()<end:time.sleep(.02)
            self.assertGreaterEqual(self.hub.last_media_epoch,1)
            # 일시정지 중에는 전송 개수가 늘지 않아야 한다.
            sender.toggle_pause();time.sleep(.3)
            count=sender.snapshot()['sent'];time.sleep(.3)
            self.assertEqual(sender.snapshot()['sent'],count)
            # 재개하면 다시 증가.
            sender.toggle_pause()
            end=time.monotonic()+3
            while sender.snapshot()['sent']<=count and time.monotonic()<end:time.sleep(.02)
            self.assertGreater(sender.snapshot()['sent'],count)
            sender.close();self.assertFalse(sender.thread.is_alive())
            bad=VideoFileSender('127.0.0.1',Path(folder)/'missing.mp4',port=self.port)
            bad.start();bad.thread.join(3)
            self.assertEqual(bad.snapshot()['state'],'오류')

    def test_aspect_ratio_padding(self):
        """정사각형 입력을 960x540으로 맞출 때 비율을 유지하고 좌우를 검은색으로 채운다."""
        frame=fit_frame(np.full((200,200,3),200,np.uint8))
        self.assertEqual(frame.shape,(540,960,3))
        self.assertFalse(frame[:,0].any())          # 왼쪽 끝 열은 패딩(0)
        self.assertEqual(int(frame[270,480,0]),200)  # 중앙은 원본 값

    def test_time_formats_and_invalid_ranges(self):
        """시간 문자열 파싱(mm:ss.s, h:mm:ss)과 잘못된 형식/재생 구간 거부를 검증한다."""
        self.assertEqual(parse_video_time('01:02.5'),62.5)
        self.assertEqual(parse_video_time('1:02:03'),3723)
        for value in ('','nan','inf','-1','1:60','1::2','1.5:20'):
            with self.assertRaises(ValueError):parse_video_time(value)
        # 끝<=시작, 음수 시작, NaN 끝은 생성자에서 거부.
        for start,end in ((2,1),(1,1),(-1,None),(0,float('nan'))):
            with self.assertRaises(ValueError):
                VideoFileSender('127.0.0.1','unused',start_s=start,end_s=end)

    def test_selected_range_pixels_and_loop(self):
        """구간 재생(0.55~1.05초)에서 전송된 프레임 인덱스/시간이 구간 안에 있고,
        실제 픽셀 값이 해당 인덱스 프레임(40+i)과 일치하는지 단발/반복 모두 확인한다."""
        # 서버 핸들러를 감싸서 수신된 frame의 헤더와 디코딩 픽셀 평균을 기록한다.
        original=self.hub.handle
        observed=[]
        def handle(token,header,payload):
            response=original(token,header,payload)
            if header.get('op')=='frame':
                image=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
                observed.append((header.copy(),float(image[100:400,100:800].mean())))
            return response
        self.hub.handle=handle
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            path=self.make_video(folder,fps=10,count=30)
            for loop in (False,True):
                observed.clear()
                sender=VideoFileSender('127.0.0.1',path,loop=loop,port=self.port,
                                       start_s=.55,end_s=1.05)
                sender.start()
                try:
                    # 단발: 스레드 종료까지, 반복: 한 바퀴(media_epoch>=1)까지 대기.
                    end=time.monotonic()+4
                    while (sender.thread.is_alive() and
                           (not loop or self.hub.last_media_epoch in (None,0)) and time.monotonic()<end):
                        time.sleep(.02)
                    if loop:self.assertGreaterEqual(self.hub.last_media_epoch,1)
                    else:self.assertEqual(sender.snapshot()['state'],'재생 완료')
                finally:sender.close()
                self.assertTrue(observed)
                # 10fps에서 0.55초 이후 첫 프레임은 인덱스 6.
                self.assertEqual(observed[0][0]['frame_index'],6)
                for header,pixel in observed:
                    self.assertTrue(6<=header['frame_index']<11,header)
                    self.assertTrue(.55<=header['media_time_s']<1.05,header)
                    # JPEG 손실을 고려해 ±3 허용.
                    self.assertAlmostEqual(pixel,40+header['frame_index'],delta=3)

    def test_range_beyond_file_does_not_switch_source(self):
        """재생 구간이 파일 길이(1초)를 벗어나면 '오류'로 끝나고,
        서버 소스는 camera 그대로이며 프레임도 하나도 전송되지 않아야 한다."""
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            path=self.make_video(folder,fps=10,count=10)
            for start,end in ((1,None),(0,2)):
                sender=VideoFileSender('127.0.0.1',path,port=self.port,start_s=start,end_s=end)
                sender.start();sender.thread.join(3)
                self.assertFalse(sender.thread.is_alive())
                self.assertEqual(sender.snapshot()['state'],'오류')
                self.assertEqual(self.hub.mode,'camera')
                self.assertEqual(self.hub.received,0)

    def test_seek_while_paused_preserves_loop_count_and_resets_epoch(self):
        """일시정지 상태에서 seek: 상태는 일시정지 유지, 위치만 이동, loop_count 보존,
        서버 epoch 증가. 구간 밖 seek 값은 구간 시작/마지막 프레임으로 클램프된다."""
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=10,count=30),
                                   port=self.port,start_s=.5,end_s=2.)
            def wait(condition):
                """조건이 참이 될 때까지 최대 3초 폴링."""
                end=time.monotonic()+3
                while not condition() and time.monotonic()<end:time.sleep(.01)
                self.assertTrue(condition(),sender.snapshot())
            sender.start()
            try:
                wait(lambda:sender.snapshot()['sent']>=2)
                sender.toggle_pause();wait(lambda:sender.snapshot()['state']=='일시정지')
                # Let the in-flight frame settle before checking pause retention.
                time.sleep(.15)
                before=sender.snapshot();epoch=self.hub.epoch
                request=sender.seek(1.4)
                wait(lambda:sender.snapshot().get('seek_applied')==request)
                self.assertEqual(sender.snapshot()['state'],'일시정지')
                self.assertAlmostEqual(sender.snapshot()['media_time_s'],1.4)
                self.assertEqual(sender.snapshot()['loop_count'],before['loop_count'])
                self.assertGreater(self.hub.epoch,epoch)
                # seek 후에도 일시정지 중이므로 추가 전송 없음.
                count=sender.snapshot()['sent'];time.sleep(.15)
                self.assertEqual(sender.snapshot()['sent'],count)
                # 구간 시작(0.5) 이전 → 0.5로 클램프.
                request=sender.seek(0)
                wait(lambda:sender.snapshot().get('seek_applied')==request)
                self.assertEqual(sender.snapshot()['media_time_s'],.5)
                # 구간 끝 이후 → 마지막 프레임(1.9)으로 클램프.
                request=sender.seek(999)
                wait(lambda:sender.snapshot().get('seek_applied')==request)
                self.assertEqual(sender.snapshot()['media_time_s'],1.9)
                # 재개하면 곧 구간 끝에 도달해 loop_count가 증가한다.
                sender.toggle_pause()
                wait(lambda:sender.snapshot()['loop_count']>before['loop_count'])
            finally:sender.close()

    def test_default_end_is_reported_for_progress_and_seek_requires_active_sender(self):
        """end_s 미지정 시 파일 길이(0.5초)가 range_end_s로 보고되고,
        시작 전/종료 후에는 seek가 거부된다."""
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=10,count=5),
                                   port=self.port,loop=False)
            with self.assertRaises(ValueError):sender.seek(0)
            sender.start();sender.thread.join(3)
            self.assertEqual(sender.snapshot()['range_end_s'],.5)
            self.assertEqual(sender.snapshot()['state'],'재생 완료')
            with self.assertRaises(ValueError):sender.seek(.2)

    def test_20fps_unique_source_and_native_rate_cap(self):
        """요청 20fps: 30fps 원본은 20fps로 다운샘플(중복 프레임 없음),
        10fps 원본은 원본 속도(10fps)를 넘지 않는다."""
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            # 수신된 (media_epoch, frame_index) 쌍을 기록해 중복 여부를 확인한다.
            received=[];original=self.hub.handle
            def handle(token,header,payload):
                result=original(token,header,payload)
                if header.get('op')=='frame':received.append((header['media_epoch'],header['frame_index']))
                return result
            self.hub.handle=handle
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=30,count=30),
                                   loop=False,fps=20,port=self.port)
            sender.start();sender.thread.join(5)
            self.assertEqual(sender.snapshot()['state'],'재생 완료')
            self.assertEqual(sender.snapshot()['effective_fps'],20)
            self.assertTrue(15<=len(received)<=21,len(received))
            self.assertEqual(len(received),len(set(received)))
            received.clear()
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=10,count=10),
                                   loop=False,fps=20,port=self.port)
            sender.start();sender.thread.join(5)
            self.assertEqual(sender.snapshot()['effective_fps'],10)
            self.assertEqual(len(received),len(set(received)))
            self.assertLessEqual(len(received),10)

    def test_requested_rate_respects_device_limit(self):
        """장치(서버)의 입력 FPS 한도(15)가 요청 FPS(20)보다 낮으면 장치 한도를 따른다.
        20fps 초과 요청은 생성자에서 거부."""
        self.hub.input_fps=15
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=30,count=15),
                                   loop=False,fps=20,port=self.port)
            sender.start();sender.thread.join(5)
            self.assertEqual(sender.snapshot()['effective_fps'],15)
            self.assertEqual(sender.snapshot()['device_fps'],15)
        with self.assertRaises(ValueError):VideoFileSender('127.0.0.1','unused',fps=21)

    def test_processing_switch_keeps_publisher_and_frame_provenance(self):
        """처리 모드(basic/full) 전환은 송신자 소유권과 source epoch를 유지하고,
        과거 프레임의 processing_mode 기록은 그대로 두며, 새 프레임에는 새 모드와
        증가한 processing_epoch가 붙는다."""
        token=object()
        self.command(token,'select',source='pc')
        self.command(token,'frame',frame_index=0,media_time_s=0.,media_epoch=0)
        source_epoch=self.hub.epoch
        set_processing_mode('127.0.0.1','full',self.port)
        self.assertIs(self.hub.owner,token)
        self.assertEqual(self.hub.epoch,source_epoch)
        _,meta=self.hub.take()
        self.assertEqual(meta['processing_mode'],'full')
        self.hub.remember(9,meta)
        set_processing_mode('127.0.0.1','basic',self.port)
        # 이미 기록된 프레임(9)의 출처 정보는 바뀌지 않는다.
        self.assertEqual(self.hub.metadata(9)['processing_mode'],'full')
        self.command(token,'frame',frame_index=1,media_time_s=.1,media_epoch=0)
        _,new=self.hub.take()
        self.assertEqual(new['processing_mode'],'basic')
        self.assertGreater(new['processing_epoch'],meta['processing_epoch'])
        with self.assertRaises(ValueError):set_processing_mode('127.0.0.1','invalid',self.port)


if __name__=='__main__':unittest.main()
