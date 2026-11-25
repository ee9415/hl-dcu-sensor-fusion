"""Ordered, bounded TCP export; backpressure instead of silent frame eviction."""
import queue
import socketserver
import threading
import time
import uuid

from perception_wire import encode_bundle


def put_ordered(mailbox, value, active, timeout=2.):
    deadline = time.monotonic()+timeout
    while active.is_set():
        try:
            mailbox.put(value, timeout=min(.1, max(.001, deadline-time.monotonic())))
            return True
        except queue.Full:
            if time.monotonic() >= deadline:
                raise RuntimeError('Perception consumer stalled; ordered queue is full')
    return False


class PerceptionServer:
    def __init__(self, port=8560, fps=None, queue_depth=8):
        self.session = str(uuid.uuid4())
        self.clients = set()
        self.lock = threading.Lock()
        self.pending = queue.Queue(maxsize=queue_depth)
        self.active = threading.Event()
        self.active.set()
        self.period = 0. if fps is None else 1 / float(fps)
        self.last_sent = 0.
        self.errors = 0
        self.accepted = 0
        self.encoded = 0
        self.high_water = 0
        self.fault = None
        owner = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(5)
                if self.rfile.readline(32) != b'OAK_PERCEPTION_1\n':
                    return
                mailbox = queue.Queue(maxsize=queue_depth)
                active = threading.Event()
                active.set()
                client = (mailbox, active)
                with owner.lock:
                    owner.clients.add(client)
                try:
                    while owner.active.is_set():
                        try:
                            packet = mailbox.get(timeout=.5)
                        except queue.Empty:
                            continue
                        self.request.sendall(packet)
                except OSError:
                    pass
                finally:
                    active.clear()
                    with owner.lock:
                        owner.clients.discard(client)

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self.server = Server(('0.0.0.0', port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        threading.Thread(target=self._encode, daemon=True).start()

    def wanted(self):
        if self.fault:
            raise RuntimeError(self.fault)
        with self.lock:
            return bool(self.clients) and time.monotonic()-self.last_sent >= self.period

    def publish(self, metadata, arrays):
        if not self.wanted():
            return
        self.last_sent = time.monotonic()
        # Own immutable snapshots: DepthAI may recycle its frame storage.
        snapshot = {name: value.copy() for name, value in arrays.items()}
        self.accepted += 1
        put_ordered(self.pending, (dict(metadata, session=self.session,
            export_seq=self.accepted, delivery_mode='ordered'), snapshot), self.active)
        self.high_water = max(self.high_water, self.pending.qsize())

    def _encode(self):
        while self.active.is_set():
            try:
                metadata, arrays = self.pending.get(timeout=.5)
            except queue.Empty:
                continue
            try:
                packet = encode_bundle(metadata, arrays)
                with self.lock:
                    clients = tuple(self.clients)
                for mailbox, active in clients:
                    put_ordered(mailbox, packet, active)
                self.encoded += 1
            except (ValueError, TypeError, RuntimeError) as exc:
                self.errors += 1
                self.fault = str(exc)
                print(f'PERCEPTION_EXPORT_ERROR {exc}', flush=True)
                return

    def close(self):
        self.active.clear()
        with self.lock:
            for _, active in self.clients:
                active.clear()
        self.server.shutdown()
        self.server.server_close()
