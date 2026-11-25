from pathlib import Path
import socket
import threading
import time
import unittest

from test_core import ROOT
from hl_camera_can.protocol import packet
from hl_camera_can.windows_transport import WindowsCanClient, serve_connection


class MemoryDriver:
    def __init__(self):
        self.frames = []

    def send(self, data, deadline):
        if time.time_ns() >= deadline:
            raise ValueError('Expired test frame')
        self.frames.append((data, deadline))


class WindowsTransportTests(unittest.TestCase):
    def setUp(self):
        self.key = 'test-only-'+'0'*64
        directory = ROOT/'artifacts/camera_can/tests'
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory/'test.key'
        self.path.write_text(self.key, encoding='ascii')
        self.driver = MemoryDriver()
        self.errors = []

    def start_server(self, enabled=True):
        self.server = socket.socket()
        self.server.bind(('127.0.0.1', 0)); self.server.listen(1)
        self.port = self.server.getsockname()[1]
        def serve():
            try:
                connection, _ = self.server.accept()
                with connection:serve_connection(connection, self.key, self.driver if enabled else None)
            except (OSError, ValueError) as exc:self.errors.append(str(exc))
        self.thread = threading.Thread(target=serve, daemon=True)
        self.thread.start()
        self.client = WindowsCanClient('127.0.0.1', self.port, self.path)

    def tearDown(self):
        if hasattr(self, 'client'):self.client.close()
        if hasattr(self, 'server'):
            self.server.close(); self.thread.join(timeout=1.)

    def test_authenticated_frame_is_handed_off_once_with_bounded_deadline(self):
        self.start_server()
        data = packet(1, True, (.1, 0, 0, 0), 3., 80)
        self.client.send(data, time.monotonic_ns()+200_000_000)
        self.assertEqual(len(self.driver.frames), 1)
        self.assertEqual(self.driver.frames[0][0], data)
        self.assertLessEqual(self.driver.frames[0][1]-time.time_ns(), 100_000_000)

    def test_wrong_key_never_reaches_driver(self):
        self.start_server()
        self.client.key = 'wrong-'+'1'*64
        with self.assertRaises((ValueError, OSError)):self.client.connect()
        self.assertEqual(self.driver.frames, [])

    def test_disabled_windows_service_cannot_report_a_hardware_send(self):
        self.start_server(enabled=False)
        with self.assertRaises(ValueError):self.client.send(packet(1), time.monotonic_ns()+100_000_000)
        self.assertEqual(self.driver.frames, [])

    def test_expired_path_never_reaches_windows_driver(self):
        self.start_server()
        with self.assertRaises(ValueError):self.client.send(packet(1), time.monotonic_ns()+5_000_000)
        self.assertEqual(self.driver.frames, [])


if __name__ == '__main__':unittest.main()
