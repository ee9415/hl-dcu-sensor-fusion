"""Exercise transport reference continuity without requiring GStreamer."""
import ast
from collections import deque
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace
import unittest


def factory(media_factory=False):
    tree = ast.parse((Path(__file__).parents[1]/'rtsp_server.py').read_text())
    classes = [n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name in ('_H264Feed', '_H264MediaFactory')]
    class Buffer:
        def fill(self, _offset, data):
            self.data = data
    gst = SimpleNamespace(Buffer=SimpleNamespace(new_allocate=lambda *_: Buffer()),
                          FlowReturn=SimpleNamespace(OK=0))
    namespace = dict(deque=deque, threading=threading, re=re, time=time, Gst=gst,
                     GstRtspServer=SimpleNamespace(RTSPMediaFactory=object))
    exec(compile(ast.Module(body=classes, type_ignores=[]), 'rtsp_server.py', 'exec'), namespace)
    return namespace['_H264MediaFactory' if media_factory else '_H264Feed'](20)


SPS = b'\x00\x00\x00\x01\x67sps'
PPS = b'\x00\x00\x01\x68pps'
IDR = b'\x00\x00\x01\x65idr'
PFRAME = b'\x00\x00\x01\x41pframe'


class RtspContinuityTests(unittest.TestCase):
    def test_overlapping_media_have_independent_queues_and_lifetimes(self):
        f = factory(media_factory=True)
        f.send_data(SPS+PPS)
        deliveries = [[], []]
        media = []
        for index in range(2):
            source = SimpleNamespace(connect=lambda *_: None,
                emit=lambda _signal, buffer, i=index: (deliveries[i].append(buffer), 0)[1])
            item = SimpleNamespace(set_reusable=lambda value: self.assertFalse(value),
                connect=lambda *_: None,
                get_element=lambda s=source: SimpleNamespace(get_child_by_name=lambda _: s))
            media.append(item)
            f.do_configure(item)
            feed = f._feeds[id(item)]
            feed._on_prepared(item, feed._generation)
        def wait_counts(first, second):
            deadline = time.monotonic()+1
            while (len(deliveries[0])<first or len(deliveries[1])<second) and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertEqual([len(x) for x in deliveries], [first,second])
        try:
            f.send_data(IDR)
            wait_counts(1,1)
            self.assertEqual([d[0].data for d in deliveries], [SPS+PPS+IDR]*2)
            self.assertEqual([d[0].pts for d in deliveries], [0,0])
            first = f._feeds[id(media[0])]
            first._on_enough_data(first._source, first._generation)
            f.send_data(PFRAME)
            wait_counts(1,2)
            f._remove_feed(media[0],first)
            f.send_data(PFRAME)
            wait_counts(1,3)
            self.assertTrue(first._closed)
        finally:
            for item in media:
                if id(item) in f._feeds:
                    f._remove_feed(item,f._feeds[id(item)])

    def test_transient_flushing_can_resume_at_next_idr(self):
        f = factory()
        interrupted = threading.Event()
        received = []
        def emit(_signal, buffer):
            if not interrupted.is_set():
                interrupted.set()
                return -2
            received.append(buffer.data)
            f._on_enough_data(None, f._generation)
            return 0
        source = SimpleNamespace(emit=emit)
        f._source = source
        f.send_data(SPS+PPS+IDR)
        f._on_need_data(source, 0, 0)
        self.assertTrue(interrupted.wait(1))
        deadline = time.monotonic()+1
        while not f._waiting_idr and time.monotonic() < deadline:
            time.sleep(.005)
        f.send_data(IDR)
        while not received and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(received, [SPS+PPS+IDR])

    def test_prepared_restarts_supply_without_another_need_data(self):
        f = factory()
        received = threading.Event()
        def emit(_signal, _buffer):
            f._on_enough_data(None, 0)
            received.set()
            return 0
        f._source = SimpleNamespace(emit=emit)
        f._on_unprepared(None, 0)
        f.send_data(SPS+PPS+IDR)
        f._on_prepared(None, 0)
        self.assertTrue(received.wait(1))

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
        self.assertEqual(f._generation, 3)
        self.assertFalse(f._queue)

    def test_shared_media_can_resume_after_unprepared_without_reconfiguration(self):
        f = factory()
        received = []
        def emit(_signal, buffer):
            received.append(buffer.data)
            f._on_enough_data(None, f._generation)
            return 0
        source = SimpleNamespace(emit=emit)
        f._source = source
        f.send_data(SPS+PPS+IDR)
        f._on_unprepared(None, 0)
        self.assertIs(f._source, source)
        f.send_data(PFRAME)
        self.assertFalse(f._queue)
        f.send_data(IDR)
        f._on_need_data(source, 0, 0)
        deadline = time.monotonic()+1
        while not received and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(received, [SPS+PPS+IDR])
