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

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from video_file_sender import VideoFileSender, fit_frame, select_camera, parse_video_time, set_processing_mode


def jpeg(value=100):
    return cv2.imencode('.jpg', np.full((180,320,3),value,np.uint8))[1].tobytes()


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.hub = VideoInputServer(0, '127.0.0.1')
        self.port = self.hub.server.server_address[1]

    def tearDown(self):
        self.hub.close()

    def command(self, token, op, **values):
        return self.hub.handle(token, dict(op=op, **values), jpeg() if op=='frame' else b'')

    def test_latest_frame_ownership_epoch_and_no_fallback(self):
        a,b=object(),object()
        self.command(a,'select',source='pc')
        with self.assertRaises(ValueError):self.command(b,'select',source='pc')
        first=self.command(a,'frame',frame_index=3,media_time_s=.1,media_epoch=0)
        self.command(a,'frame',frame_index=3,media_time_s=.1,media_epoch=0)
        self.assertEqual(self.hub.epoch,first['source_epoch'])
        self.assertEqual(self.hub.dropped,1)
        _,meta=self.hub.take();self.assertEqual(meta['source_frame_index'],3)
        self.assertEqual(self.hub.take(),(None,None))
        self.command(a,'frame',frame_index=0,media_time_s=0,media_epoch=1)
        self.assertGreater(self.hub.epoch,first['source_epoch'])
        self.hub.release(a)
        self.assertEqual(self.hub.mode,'pc')
        self.assertEqual(self.hub.take(),(None,None))
        self.command(b,'select',source='camera')
        self.assertEqual(self.hub.take()[1]['input_source'],'camera')
        with self.assertRaises(ValueError):
            self.command(a,'frame',frame_index=1,media_time_s=.1,media_epoch=1)

    def test_protocol_partial_frame_and_size_limit(self):
        self.assertEqual(jpeg_size(jpeg()),(320,180))
        with self.assertRaises(ValueError):jpeg_size(b'not a jpeg')
        with self.assertRaises(ValueError):
            read_message(io.BytesIO((json.dumps(dict(protocol=1,length=MAX_PACKET+1))+'\n').encode()))
        with self.assertRaises(EOFError):
            read_message(io.BytesIO(b'{"protocol":1,"length":10}\nabc'))
        token=object();self.command(token,'select',source='pc')
        with self.assertRaises(ValueError):
            self.command(token,'frame',frame_index=1,media_time_s=float('nan'),media_epoch=0)
        for i in range(300):self.hub.remember(i,dict(source_frame_index=i))
        self.assertEqual(len(self.hub.frames),256)
        self.assertEqual(self.hub.metadata(299)['source_frame_index'],299)

    def test_socket_fragmentation_and_camera_control(self):
        with socket.create_connection(('127.0.0.1',self.port),timeout=2) as c:
            data=b'{"protocol":1,"length":0,"op":"select","source":"pc"}\n'
            for chunk in (data[:9],data[9:]):c.sendall(chunk)
            with c.makefile('rb') as stream:
                response,_=read_message(stream);self.assertTrue(response['ok'])
                send_message(c,dict(op='frame',frame_index=0,media_time_s=0.,media_epoch=0),jpeg())
                response,_=read_message(stream);self.assertEqual(response['received_frames'],1)
        response=select_camera('127.0.0.1',self.port)
        self.assertEqual(response['source'],'camera')

    def make_video(self, folder, fps=30, count=30):
        path=Path(folder)/'input.avi'
        writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'MJPG'),fps,(320,180))
        self.assertTrue(writer.isOpened())
        for i in range(count):writer.write(np.full((180,320,3),40+i,np.uint8))
        writer.release()
        return path

    def test_real_file_sender_is_paced_skips_frames_and_finishes(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder),loop=False,port=self.port)
            start=time.monotonic();sender.start();sender.thread.join(5)
            self.assertFalse(sender.thread.is_alive())
            status=sender.snapshot()
            self.assertEqual(status['state'],'재생 완료',status)
            self.assertTrue(7<=status['sent']<=11,status)
            self.assertGreaterEqual(time.monotonic()-start,.9)
            self.assertGreater(self.hub.last_frame_index,status['sent'])
            self.assertEqual(self.hub.mode,'pc')

    def test_loop_pause_stop_and_invalid_file(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=10,count=3),port=self.port)
            sender.start()
            end=time.monotonic()+4
            while self.hub.last_media_epoch in (None,0) and time.monotonic()<end:time.sleep(.02)
            self.assertGreaterEqual(self.hub.last_media_epoch,1)
            sender.toggle_pause();time.sleep(.3)
            count=sender.snapshot()['sent'];time.sleep(.3)
            self.assertEqual(sender.snapshot()['sent'],count)
            sender.toggle_pause()
            end=time.monotonic()+3
            while sender.snapshot()['sent']<=count and time.monotonic()<end:time.sleep(.02)
            self.assertGreater(sender.snapshot()['sent'],count)
            sender.close();self.assertFalse(sender.thread.is_alive())
            bad=VideoFileSender('127.0.0.1',Path(folder)/'missing.mp4',port=self.port)
            bad.start();bad.thread.join(3)
            self.assertEqual(bad.snapshot()['state'],'오류')

    def test_aspect_ratio_padding(self):
        frame=fit_frame(np.full((200,200,3),200,np.uint8))
        self.assertEqual(frame.shape,(540,960,3))
        self.assertFalse(frame[:,0].any())
        self.assertEqual(int(frame[270,480,0]),200)

    def test_time_formats_and_invalid_ranges(self):
        self.assertEqual(parse_video_time('01:02.5'),62.5)
        self.assertEqual(parse_video_time('1:02:03'),3723)
        for value in ('','nan','inf','-1','1:60','1::2','1.5:20'):
            with self.assertRaises(ValueError):parse_video_time(value)
        for start,end in ((2,1),(1,1),(-1,None),(0,float('nan'))):
            with self.assertRaises(ValueError):
                VideoFileSender('127.0.0.1','unused',start_s=start,end_s=end)

    def test_selected_range_pixels_and_loop(self):
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
                    end=time.monotonic()+4
                    while (sender.thread.is_alive() and
                           (not loop or self.hub.last_media_epoch in (None,0)) and time.monotonic()<end):
                        time.sleep(.02)
                    if loop:self.assertGreaterEqual(self.hub.last_media_epoch,1)
                    else:self.assertEqual(sender.snapshot()['state'],'재생 완료')
                finally:sender.close()
                self.assertTrue(observed)
                self.assertEqual(observed[0][0]['frame_index'],6)
                for header,pixel in observed:
                    self.assertTrue(6<=header['frame_index']<11,header)
                    self.assertTrue(.55<=header['media_time_s']<1.05,header)
                    self.assertAlmostEqual(pixel,40+header['frame_index'],delta=3)

    def test_range_beyond_file_does_not_switch_source(self):
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
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=10,count=30),
                                   port=self.port,start_s=.5,end_s=2.)
            def wait(condition):
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
                count=sender.snapshot()['sent'];time.sleep(.15)
                self.assertEqual(sender.snapshot()['sent'],count)
                request=sender.seek(0)
                wait(lambda:sender.snapshot().get('seek_applied')==request)
                self.assertEqual(sender.snapshot()['media_time_s'],.5)
                request=sender.seek(999)
                wait(lambda:sender.snapshot().get('seek_applied')==request)
                self.assertEqual(sender.snapshot()['media_time_s'],1.9)
                sender.toggle_pause()
                wait(lambda:sender.snapshot()['loop_count']>before['loop_count'])
            finally:sender.close()

    def test_default_end_is_reported_for_progress_and_seek_requires_active_sender(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=10,count=5),
                                   port=self.port,loop=False)
            with self.assertRaises(ValueError):sender.seek(0)
            sender.start();sender.thread.join(3)
            self.assertEqual(sender.snapshot()['range_end_s'],.5)
            self.assertEqual(sender.snapshot()['state'],'재생 완료')
            with self.assertRaises(ValueError):sender.seek(.2)

    def test_20fps_unique_source_and_native_rate_cap(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
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
        self.hub.input_fps=15
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            sender=VideoFileSender('127.0.0.1',self.make_video(folder,fps=30,count=15),
                                   loop=False,fps=20,port=self.port)
            sender.start();sender.thread.join(5)
            self.assertEqual(sender.snapshot()['effective_fps'],15)
            self.assertEqual(sender.snapshot()['device_fps'],15)
        with self.assertRaises(ValueError):VideoFileSender('127.0.0.1','unused',fps=21)

    def test_processing_switch_keeps_publisher_and_frame_provenance(self):
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
        self.assertEqual(self.hub.metadata(9)['processing_mode'],'full')
        self.command(token,'frame',frame_index=1,media_time_s=.1,media_epoch=0)
        _,new=self.hub.take()
        self.assertEqual(new['processing_mode'],'basic')
        self.assertGreater(new['processing_epoch'],meta['processing_epoch'])
        with self.assertRaises(ValueError):set_processing_mode('127.0.0.1','invalid',self.port)


if __name__=='__main__':unittest.main()
