"""Authenticated, bounded, one-request-at-a-time WSL -> Windows CAN handoff."""
import hmac
import json
from pathlib import Path
import socket
import time

from .protocol import check_packet
from .timebase import unix_ns

CLOCK_MARGIN_NS = 10_000_000


def send_json(connection, value):
    connection.sendall(json.dumps(value, separators=(',', ':')).encode()+b'\n')


def read_json(stream):
    line = stream.readline(4097)
    if not line or len(line) > 4096 or not line.endswith(b'\n'):
        raise ValueError('Closed or oversized Windows CAN message')
    value = json.loads(line)
    if not isinstance(value, dict):
        raise ValueError('Expected Windows CAN object')
    return value


class WindowsCanClient:
    def __init__(self, host, port, key_file, allow_virtual=False, monitor_only=False):
        self.host, self.port = host, port
        self.key = Path(key_file).read_text(encoding='ascii').strip()
        if len(self.key) < 64:
            raise ValueError('Windows CAN key must contain at least 64 characters')
        self.allow_virtual = allow_virtual
        self.monitor_only = monitor_only
        self.connection = None
        self.stream = None
        self.synchronized = 0.
        self.server_offset_lower_ns = 0
        self.server_offset_mid_ns = None

    def connect(self):
        self.close()
        self.connection = socket.create_connection((self.host, self.port), timeout=.1)
        self.connection.settimeout(.1)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.stream = self.connection.makefile('rb')
        self.hello()

    def hello(self):
        before = unix_ns()
        send_json(self.connection, dict(op='hello', key=self.key))
        response = read_json(self.stream)
        after = unix_ns()
        server = response.get('server_unix_ns', 0)
        # Network one-way delays are nonnegative. These bounds include RTT
        # uncertainty, so an unverified clock relationship cannot transmit.
        offset_lower, offset_upper = server-after, server-before
        midpoint = (offset_lower+offset_upper)//2
        if (not response.get('ok') or
                bool(response.get('monitor_only', False)) != self.monitor_only or
                (not self.monitor_only and not response.get('enabled')) or
                (response.get('virtual') and not self.allow_virtual) or
                after < before or after-before > CLOCK_MARGIN_NS or
                (self.server_offset_mid_ns is not None and
                 abs(midpoint-self.server_offset_mid_ns) > CLOCK_MARGIN_NS)):
            raise ValueError('Windows driver disabled, virtual mismatch, RTT >10ms, or clock jump')
        self.server_offset_lower_ns = offset_lower
        self.server_offset_mid_ns = midpoint
        self.synchronized = time.monotonic()

    def send(self, data, deadline_monotonic_ns):
        check_packet(data)
        try:
            if self.connection is None:
                self.connect()
            # WSL and Windows clocks can slew independently. Refresh for each
            # 50ms frame instead of letting offset uncertainty grow for 1s.
            if time.monotonic()-self.synchronized > .02:
                self.hello()
            remaining = min(deadline_monotonic_ns-time.monotonic_ns(), 100_000_000)
            if remaining <= CLOCK_MARGIN_NS+20_000_000:
                raise ValueError('Path expired before Windows handoff')
            send_json(self.connection, dict(op='frame', data=bytes(data).hex(),
                deadline_unix_ns=unix_ns()+remaining+self.server_offset_lower_ns-CLOCK_MARGIN_NS))
            response = read_json(self.stream)
            accepted = (response.get('captured') and not response.get('sent') if self.monitor_only
                        else response.get('sent'))
            if not response.get('ok') or not accepted:
                raise OSError(response.get('error', 'Windows CAN send rejected'))
            return not self.monitor_only
        except (OSError, ValueError, TypeError, KeyError):
            self.close()
            raise

    def close(self):
        if self.stream is not None:
            self.stream.close()
        if self.connection is not None:
            self.connection.close()
        self.connection = self.stream = None
        self.server_offset_mid_ns = None


def serve_connection(connection, key, driver, virtual=False, monitor=None):
    if monitor is not None and driver is not None:
        raise ValueError('Monitor mode cannot have a CAN driver')
    connection.settimeout(.2)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    authenticated = False
    last_hello = 0.
    with connection.makefile('rb') as stream:
        while True:
            request = read_json(stream)
            if request.get('op') == 'hello':
                if not hmac.compare_digest(str(request.get('key', '')), key):
                    raise ValueError('Windows CAN authentication failed')
                authenticated, last_hello = True, time.monotonic()
                send_json(connection, dict(ok=True, enabled=driver is not None, virtual=virtual,
                    monitor_only=monitor is not None,
                    server_unix_ns=unix_ns()))
                continue
            if not authenticated or time.monotonic()-last_hello > 1.2:
                raise ValueError('Windows CAN session handshake expired')
            if request.get('op') != 'frame' or (driver is None and monitor is None):
                raise ValueError('Windows CAN is disabled or unknown command')
            data = bytes.fromhex(request['data'])
            deadline = request['deadline_unix_ns']
            error = ''
            try:
                check_packet(data)
                if type(deadline) is not int or not 20_000_000 < deadline-unix_ns() <= 100_000_000:
                    raise ValueError('Windows CAN deadline invalid or expired')
            except (ValueError, TypeError, OverflowError) as exc:
                error = str(exc)
            if monitor is not None:
                monitor(data, deadline, error)
            if error:
                raise ValueError(error)
            if monitor is not None:
                send_json(connection, dict(ok=True, captured=True, sent=False))
                continue
            driver.send(data, deadline)
            send_json(connection, dict(ok=True, sent=True))
