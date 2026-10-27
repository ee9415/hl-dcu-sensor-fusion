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
from video_file_sender import VideoFileSender, fit_frame, select_camera


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


if __name__=='__main__':unittest.main()
