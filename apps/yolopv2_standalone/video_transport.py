"""Bounded PC JPEG input/control on the device LAN (TCP 8558, protocol v1)."""
from collections import OrderedDict
import json
import math
import socketserver
import threading
import time

PORT = 8558
MAX_PACKET = 2 * 1024 * 1024
MAX_HEADER = 4096


def send_message(connection, message, payload=b''):
    header = dict(message, length=len(payload), protocol=1)
    data = json.dumps(header, allow_nan=False).encode() + b'\n'
    if len(data) > MAX_HEADER or len(payload) > MAX_PACKET:
        raise ValueError('Video input packet is too large')
    connection.sendall(data + payload)


def read_message(stream):
    line = stream.readline(MAX_HEADER + 1)
    if not line:
        raise EOFError('Video input disconnected')
    if len(line) > MAX_HEADER or not line.endswith(b'\n'):
        raise ValueError('Invalid video input header')
    header = json.loads(line)
    if not isinstance(header, dict):
        raise ValueError('Expected video input header object')
    size = header.get('length', 0)
    if header.get('protocol') != 1 or type(size) is not int or not 0 <= size <= MAX_PACKET:
        raise ValueError('Unsupported video input packet')
    payload = bytearray()
    while len(payload) < size:
        chunk = stream.read(size - len(payload))
        if not chunk:
            raise EOFError('Truncated video input frame')
        payload.extend(chunk)
    return header, bytes(payload)


def jpeg_size(data):
    """Read SOF before decoding so a small JPEG cannot request a huge image."""
    if data[:2] != b'\xff\xd8':
        raise ValueError('Expected JPEG')
    i = 2
    while i + 4 <= len(data):
        if data[i] != 255:
            raise ValueError('Invalid JPEG marker')
        while i < len(data) and data[i] == 255:
            i += 1
        if i >= len(data):
            break
        marker = data[i]; i += 1
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            continue
        if marker == 0xDA:
            break
        length = int.from_bytes(data[i:i+2], 'big')
        if length < 2 or i + length > len(data):
            break
        if marker in (0xC0, 0xC1, 0xC2):
            if length < 8:
                break
            h = int.from_bytes(data[i+3:i+5], 'big')
            w = int.from_bytes(data[i+5:i+7], 'big')
            if not (16 <= w <= 1920 and 16 <= h <= 1080):
                raise ValueError('JPEG dimensions exceed 1920x1080')
            return w, h
        i += length
    raise ValueError('JPEG has no supported frame header')


class VideoInputServer:
    """One publisher, one pending frame, and bounded per-frame provenance."""
    def __init__(self, port=PORT, host='0.0.0.0', input_fps=20):
        if input_fps not in (10,15,20):
            raise ValueError('Input FPS must be 10, 15 or 20')
        self.input_fps = input_fps
        self.lock = threading.Lock()
        self.mode = 'camera'
        self.epoch = 0
        self.owner = None
        self.latest = None
        self.frames = OrderedDict()
        self.received = self.consumed = self.dropped = 0
        self.last_frame_index = None
        self.last_media_epoch = None
        self.last_received = None
        owner = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(10)
                token = object()
                try:
                    while True:
                        header, payload = read_message(self.rfile)
                        response = owner.handle(token, header, payload)
                        send_message(self.request, dict(response, ok=True))
                except (EOFError, OSError):
                    pass
                except (ValueError, KeyError, TypeError) as exc:
                    try:
                        send_message(self.request, dict(ok=False, error=str(exc)))
                    except OSError:
                        pass
                finally:
                    owner.release(token)

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True
        self.server = Server((host, port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True,
                                       name='pc-video-input')
        self.thread.start()

    def handle(self, token, header, payload):
        op = header.get('op')
        with self.lock:
            if op == 'select':
                source = header.get('source')
                if source not in ('camera', 'pc') or payload:
                    raise ValueError('Select camera or pc')
                if source == 'pc' and self.owner not in (None, token):
                    raise ValueError('Another PC is already publishing')
                self.mode = source
                self.owner = token if source == 'pc' else None
                self.epoch += 1
                self.latest = None
                self.last_frame_index = self.last_media_epoch = None
            elif op == 'frame':
                if self.mode != 'pc' or self.owner is not token:
                    raise ValueError('Select PC input before sending frames')
                index, media = header.get('frame_index'), header.get('media_time_s')
                media_epoch = header.get('media_epoch')
                if (type(index) is not int or index < 0 or type(media_epoch) is not int
                        or media_epoch < 0 or type(media) not in (int, float)
                        or not math.isfinite(media) or media < 0):
                    raise ValueError('Invalid source frame metadata')
                jpeg_size(payload)
                if (self.last_media_epoch != media_epoch or
                        (self.last_frame_index is not None and index < self.last_frame_index)):
                    self.epoch += 1
                if self.latest is not None:
                    self.dropped += 1
                self.latest = (payload, dict(input_source='pc', source_epoch=self.epoch,
                    source_frame_index=index, source_media_time_s=media,
                    source_media_epoch=media_epoch, source_file=str(header.get('file', ''))[:256]))
                self.last_frame_index, self.last_media_epoch = index, media_epoch
                self.received += 1
                self.last_received = time.monotonic()
            elif op == 'stop':
                if self.owner is token:
                    self.latest = None
                    self.owner = None
                    self.epoch += 1
            elif op != 'status' or payload:
                raise ValueError('Unknown video input command')
            return self._status()

    def _status(self):
        return dict(source=self.mode, source_epoch=self.epoch,
                    input_fps_limit=self.input_fps,
                    publisher_connected=self.owner is not None,
                    received_frames=self.received, consumed_frames=self.consumed,
                    dropped_frames=self.dropped)

    def release(self, token):
        with self.lock:
            if self.owner is token:
                self.owner = None
                self.latest = None
                self.epoch += 1
                # Deliberately wait in PC mode; never silently show camera footage.

    def take(self):
        with self.lock:
            if self.mode == 'camera':
                return None, dict(input_source='camera', source_epoch=self.epoch,
                                  source_frame_index=None, source_media_time_s=None)
            pending, self.latest = self.latest, None
            if pending is None:
                return None, None
            self.consumed += 1
            return pending

    def remember(self, sequence, metadata):
        with self.lock:
            self.frames[sequence] = dict(metadata)
            while len(self.frames) > 256:
                self.frames.popitem(last=False)

    def metadata(self, sequence):
        with self.lock:
            return dict(self.frames.get(sequence, {}))

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
