"""Bounded live JSON telemetry on trusted device LAN; no replay or control commands."""
import json
import queue
import socketserver
import threading
import time
import uuid


class TelemetryServer:
    def __init__(self, port=8557):
        self.session = str(uuid.uuid4())
        self.clients = set()
        self.lock = threading.Lock()
        owner = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(5)
                if self.rfile.readline(128) != b'HELLO\n':
                    return
                received = time.time()
                received_mono = time.monotonic()
                self.wfile.write((json.dumps({'clock_receive_s': received,
                                             'clock_receive_mono_s': received_mono,
                                             'clock_send_mono_s': time.monotonic(),
                                             'clock_send_s': time.time()})+'\n').encode())
                messages = queue.Queue(maxsize=100)
                with owner.lock:
                    owner.clients.add(messages)
                try:
                    while True:
                        try:
                            data = messages.get(timeout=5)
                        except queue.Empty:
                            data = b'{"heartbeat":true}\n'
                        self.wfile.write(data)
                except OSError:
                    pass
                finally:
                    with owner.lock:
                        owner.clients.discard(messages)

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self.server = Server(('0.0.0.0', port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def publish(self, payload):
        payload = dict(payload, session=self.session, schema_version=1,
                       sent_monotonic_s=time.monotonic(), sent_wall_s=time.time())
        data = (json.dumps(payload, allow_nan=False)+'\n').encode()
        with self.lock:
            for messages in self.clients:
                try:
                    messages.put_nowait(data)
                except queue.Full:
                    # Never stall perception for a slow reader. Sequence gaps expose loss.
                    try:
                        messages.get_nowait()
                    except queue.Empty:
                        pass
                    messages.put_nowait(data)
