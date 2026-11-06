"""Exercise transport reference continuity without requiring GStreamer."""
import ast
from collections import deque
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace
import unittest


def factory():
    tree = ast.parse((Path(__file__).parents[1]/'rtsp_server.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == '_H264MediaFactory')
    class Buffer:
        def fill(self, _offset, data):
            self.data = data
    gst = SimpleNamespace(Buffer=SimpleNamespace(new_allocate=lambda *_: Buffer()),
                          FlowReturn=SimpleNamespace(OK=0))
    namespace = dict(deque=deque, threading=threading, re=re, Gst=gst,
                     GstRtspServer=SimpleNamespace(RTSPMediaFactory=object))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), 'rtsp_server.py', 'exec'), namespace)
    return namespace['_H264MediaFactory'](20)


SPS = b'\x00\x00\x00\x01\x67sps'
PPS = b'\x00\x00\x01\x68pps'
IDR = b'\x00\x00\x01\x65idr'
PFRAME = b'\x00\x00\x01\x41pframe'


class RtspContinuityTests(unittest.TestCase):
    def test_empty_input_callback_returns_without_waiting_for_unprepared(self):
        f = factory()
        callback = threading.Thread(target=f._on_need_data, args=(None, 0, 0), daemon=True)
        callback.start()
        try:
            callback.join(.2)
            self.assertFalse(callback.is_alive(), 'Empty input blocks media teardown')
        finally:
            f._on_unprepared(None, 0)
            callback.join(1)

    def test_worker_preserves_order_and_obeys_backpressure(self):
        f = factory()
        received = []
        def emit(_signal, buffer):
            received.append(buffer.data)
            f._on_enough_data(None, f._generation)
            return 0
        f._source = SimpleNamespace(emit=emit)
        f.send_data(SPS+PPS+IDR)
        f.send_data(PFRAME)
        for count in (1, 2):
            f._on_need_data(f._source, 0, f._generation)
            deadline = time.monotonic()+1
            while len(received) < count and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(len(received), count)
        self.assertEqual(received, [SPS+PPS+IDR, PFRAME])
        f._on_unprepared(None, f._generation)

    def test_new_media_discards_backlog_and_waits_for_fresh_idr(self):
        f = factory()
        f.send_data(SPS+PPS+IDR)
        callbacks = []
        source = SimpleNamespace(connect=lambda *args: callbacks.append(args))
        media = SimpleNamespace(
            get_element=lambda: SimpleNamespace(get_child_by_name=lambda _: source),
            connect=lambda *args: callbacks.append(args))
        f.do_configure(media)
        self.assertFalse(f._queue)
        f.send_data(PFRAME)
        self.assertFalse(f._queue)
        f.send_data(IDR)
        self.assertEqual(f._queue.popleft(), SPS+PPS+IDR)
        self.assertTrue(all(c[-1] == f._generation for c in callbacks))

    def test_decoder_starts_only_with_parameter_sets_and_idr(self):
        f = factory()
        f.send_data(PFRAME)
        f.send_data(IDR)
        self.assertFalse(f._queue)
        f.send_data(SPS+PPS)
        f.send_data(PFRAME)
        self.assertFalse(f._queue)
        f.send_data(IDR)
        self.assertEqual(f._queue.popleft(), SPS+PPS+IDR)
        f.send_data(PFRAME)
        self.assertEqual(f._queue.popleft(), PFRAME)

    def test_overflow_drops_entire_broken_gop_then_recovers(self):
        f = factory()
        f.send_data(SPS+PPS+IDR)
        for _ in range(40):
            f.send_data(PFRAME)
        self.assertFalse(f._queue)
        self.assertTrue(f._waiting_idr)
        f.send_data(IDR)
        self.assertEqual(f._queue.popleft(), SPS+PPS+IDR)

    def test_old_media_cannot_steal_new_media_frames_or_clear_them(self):
        f = factory()
        f._generation = 3
        f.send_data(SPS+PPS+IDR)
        f._on_need_data(None, 0, 2)
        self.assertEqual(len(f._queue), 1)
        f._on_unprepared(None, 2)
        self.assertEqual(len(f._queue), 1)
        f._on_unprepared(None, 3)
        self.assertEqual(f._generation, 4)
        self.assertFalse(f._queue)
