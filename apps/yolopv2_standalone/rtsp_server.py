from collections import deque
import threading

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")

from gi.repository import GLib, Gst, GstRtspServer  # noqa: E402


class _H264MediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, fps: int) -> None:
        super().__init__()
        self._launch_string = (
            "appsrc name=source is-live=true do-timestamp=true format=time "
            f"caps=video/x-h264,stream-format=byte-stream,alignment=au,framerate={fps}/1 ! "
            "h264parse ! rtph264pay name=pay0 pt=96 config-interval=1"
        )
        self._condition = threading.Condition()
        self._queue: deque[bytes] = deque(maxlen=max(20, fps * 2))

    def send_data(self, data: bytes) -> None:
        with self._condition:
            self._queue.append(data)
            self._condition.notify()

    def _on_need_data(self, source, _length: int) -> None:
        with self._condition:
            while not self._queue:
                self._condition.wait(timeout=1.0)
            data = self._queue.popleft()

        buffer = Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        source.emit("push-buffer", buffer)

    def do_create_element(self, _url):
        return Gst.parse_launch(self._launch_string)

    def do_configure(self, media) -> None:
        source = media.get_element().get_child_by_name("source")
        source.connect("need-data", self._on_need_data)


class RTSPServer(GstRtspServer.RTSPServer):
    def __init__(self, fps: int, port: int, mount_path: str) -> None:
        super().__init__()
        Gst.init(None)
        self.set_service(str(port))
        self._factory = _H264MediaFactory(fps)
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
