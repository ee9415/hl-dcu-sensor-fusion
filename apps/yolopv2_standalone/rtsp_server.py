from collections import deque
import threading
import re
import time

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")

from gi.repository import GLib, Gst, GstRtspServer  # noqa: E402


class _H264Feed:
    def __init__(self, fps: int, name: str = 'stream') -> None:
        super().__init__()
        self._launch_string = (
            "appsrc name=source is-live=true do-timestamp=false format=time "
            f"caps=video/x-h264,stream-format=byte-stream,alignment=au,framerate={fps}/1 ! "
            "h264parse ! rtph264pay name=pay0 pt=96 config-interval=1"
        )
        self._condition = threading.Condition()
        self._queue: deque[bytes] = deque(maxlen=max(20, fps * 2))
        self._generation = 0
        self._headers: dict[int, bytes] = {}
        self._waiting_idr = True
        self._source = None
        self._demand = False
        self._last_flow = None
        self._closed = False
        self._origin = None
        self._last_pts = -1
        self._duration = 1_000_000_000 // fps
        self._name = name
        self._in_count = self._out_count = self._idr_count = 0
        self._last_stats = time.monotonic()
        self._worker = threading.Thread(target=self._pump, daemon=True, name='rtsp-appsrc')
        self._worker.start()

    def send_data(self, data: bytes) -> None:
        with self._condition:
            if self._closed:
                return
            self._in_count += 1
            starts = list(re.finditer(b'\x00\x00(?:\x00)?\x01', data))
            types = set()
            for index, start in enumerate(starts):
                if start.end() >= len(data):
                    continue
                kind = data[start.end()] & 31
                types.add(kind)
                if kind in (7, 8):
                    end = starts[index+1].start() if index+1 < len(starts) else len(data)
                    self._headers[kind] = data[start.start():end]
            self._idr_count += int(5 in types)
            if time.monotonic()-self._last_stats > 5:
                print(f'RTSP {self._name} in={self._in_count} out={self._out_count} idr={self._idr_count} queued={len(self._queue)} demand={self._demand} wait_idr={self._waiting_idr} generation={self._generation} flow={self._last_flow}',flush=True)
                self._last_stats = time.monotonic()
            # A skipped P frame invalidates subsequent reference frames. Resume
            # only at an IDR, with the parameter sets needed by a new decoder.
            if len(self._queue) == self._queue.maxlen:
                self._queue.clear()
                self._waiting_idr = True
            if self._waiting_idr:
                if 5 not in types or not all(k in self._headers for k in (7, 8)):
                    return
                data = b''.join(self._headers[k] for k in (7, 8) if k not in types)+data
                self._waiting_idr = False
            self._queue.append(data)
            self._condition.notify()

    def _on_need_data(self, source, _length: int, generation: int) -> None:
        with self._condition:
            if self._closed or generation != self._generation:
                return
            self._demand = True
            self._condition.notify_all()

    def _on_enough_data(self, _source, generation: int) -> None:
        with self._condition:
            if generation == self._generation:
                self._demand = False

    def _pump(self) -> None:
        # Never wait for a camera/PC frame inside a GStreamer callback. Media
        # teardown can wait for that callback before emitting 'unprepared'.
        # One worker preserves access-unit ordering while callbacks return now.
        while True:
            with self._condition:
                while not self._closed and (self._source is None or not self._demand or not self._queue):
                    self._condition.wait()
                if self._closed:
                    return
                source = self._source
                generation = self._generation
                data = self._queue.popleft()
                now = time.monotonic()
                if self._origin is None:
                    self._origin = now
                pts = max(self._last_pts+1, int((now-self._origin)*1_000_000_000))
                self._last_pts = pts
            buffer = Gst.Buffer.new_allocate(None, len(data), None)
            buffer.fill(0, data)
            buffer.pts = buffer.dts = pts
            buffer.duration = self._duration
            flow = source.emit('push-buffer', buffer)
            self._out_count += 1
            if flow != Gst.FlowReturn.OK:
                with self._condition:
                    if generation == self._generation:
                        # FLUSHING during a media state transition is temporary.
                        # Do not permanently consume the earlier need-data event.
                        self._queue.clear()
                        self._waiting_idr = True
                        self._condition.wait(timeout=.05)
                if flow != self._last_flow:
                    print(f'RTSP push interrupted: {flow}', flush=True)
            self._last_flow = flow

    def do_configure(self, media) -> None:
        source = media.get_element().get_child_by_name("source")
        with self._condition:
            self._generation += 1
            generation = self._generation
            self._queue.clear()
            self._waiting_idr = True
            self._source = source
            self._demand = False
            self._origin = None
            self._last_pts = -1
            self._condition.notify_all()
        source.connect("need-data", self._on_need_data, generation)
        source.connect("enough-data", self._on_enough_data, generation)
        media.connect("unprepared", self._on_unprepared, generation)
        media.connect("prepared", self._on_prepared, generation)

    def _on_prepared(self, _media, generation: int) -> None:
        with self._condition:
            if not self._closed and generation == self._generation:
                self._demand = True
                self._condition.notify_all()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._source = None
            self._queue.clear()
            self._condition.notify_all()

    def _on_unprepared(self, _media, generation: int) -> None:
        with self._condition:
            if not self._closed and generation == self._generation:
                self._queue.clear()
                self._demand = False
                self._waiting_idr = True
                self._origin = None
                self._last_pts = -1
                self._condition.notify_all()


class _H264MediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, fps: int, name: str = 'stream') -> None:
        super().__init__()
        self._fps, self._name = fps, name
        self._lock = threading.Lock()
        self._feeds = {}
        self._headers = {}
        self._next_feed = 0

    def do_create_element(self, _url):
        return Gst.parse_launch(
            'appsrc name=source is-live=true do-timestamp=false format=time '
            f'caps=video/x-h264,stream-format=byte-stream,alignment=au,framerate={self._fps}/1 ! '
            'h264parse ! rtph264pay name=pay0 pt=96 config-interval=1')

    def do_configure(self, media) -> None:
        # Even a shared factory can have overlapping media objects while an
        # old client tears down and a new one prepares. Each needs its own feed.
        media.set_reusable(False)
        with self._lock:
            self._next_feed += 1
            feed = _H264Feed(self._fps, f'{self._name}#{self._next_feed}')
            feed._headers.update(self._headers)
            self._feeds[id(media)] = feed
        media.connect('unprepared', self._remove_feed, feed)
        feed.do_configure(media)

    def _remove_feed(self, media, feed) -> None:
        with self._lock:
            if self._feeds.get(id(media)) is feed:
                self._feeds.pop(id(media))
        feed.close()

    def send_data(self, data: bytes) -> None:
        starts = list(re.finditer(b'\x00\x00(?:\x00)?\x01', data))
        with self._lock:
            for index, start in enumerate(starts):
                if start.end() < len(data) and data[start.end()] & 31 in (7, 8):
                    kind = data[start.end()] & 31
                    end = starts[index+1].start() if index+1 < len(starts) else len(data)
                    self._headers[kind] = data[start.start():end]
            headers = self._headers.copy()
            feeds = list(self._feeds.values())
        for feed in feeds:
            with feed._condition:
                feed._headers.update(headers)
            feed.send_data(data)

class RTSPServer(GstRtspServer.RTSPServer):
    def __init__(self, fps: int, port: int, mount_path: str) -> None:
        super().__init__()
        Gst.init(None)
        self.set_service(str(port))
        self._factory = _H264MediaFactory(fps, mount_path)
        self._factory.set_shared(True)
        self.get_mount_points().add_factory(mount_path, self._factory)
        self._start()

    def _start(self) -> None:
        def run() -> None:
            context = GLib.MainContext()
            loop = GLib.MainLoop(context=context)
            self.attach(context)
            loop.run()

        threading.Thread(target=run, daemon=True, name="rtsp-server").start()

    def send_data(self, data: bytes) -> None:
        self._factory.send_data(data)
