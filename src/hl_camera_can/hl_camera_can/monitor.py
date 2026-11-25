"""CAN relay capture/decoding model. No hardware SDK or ROS dependency."""
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import queue
import struct
import time

from .protocol import CAN_ID, crc8, check_packet
from .timebase import unix_ns


def decode(data):
    data = bytes(data)
    result = dict(can_id=f'0x{CAN_ID:03X}', length=len(data), dlc=13 if len(data) == 32 else None,
        raw=data.hex(' ').upper(), valid=False, error='')
    try:
        check_packet(data)
        result['valid'] = True
    except (ValueError, TypeError, OverflowError) as exc:
        result['error'] = str(exc)
    if len(data) == 32:
        coefficients = struct.unpack_from('<4f', data, 4)
        minimum, maximum, length = struct.unpack_from('<hhH', data, 20)
        result.update(header=f'0x{data[0]:02X}', sequence=data[1], request_status=data[2],
            mode='ACTIVE' if data[0] == 0xd9 else 'INACTIVE' if data[0] == 0x81 else 'INVALID',
            function_id=data[0] & 7, atomic=bool(data[0] & 0x20),
            coefficients=coefficients, min_speed_kph=minimum/100, max_speed_kph=maximum/100,
            valid_length_m=length/100, confidence=data[26],
            crc_received=f'{data[31]:02X}', crc_expected=f'{crc8(bytes.fromhex("03 20")+data[:31]):02X}',
            crc_ok=data[31] == crc8(bytes.fromhex('03 20')+data[:31]),
            application_context=data[3], trajectory_context=data[27], reserved=list(data[28:31]))
    return result


class CaptureLog:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.events = queue.Queue(maxsize=1000)
        self.recent = deque(maxlen=200)
        self.count, self.invalid, self.sequence_errors, self.display_dropped = 0, 0, 0, 0
        self.last_sequence = None
        self.last_time = None

    def capture(self, data, deadline, error=''):
        # Windows Python 3.12 monotonic uses a 15.625ms tick; QPC preserves
        # actual 50ms receive intervals instead of displaying 47/63ms steps.
        now = time.perf_counter()
        event = decode(data)
        if error:
            event.update(valid=False, error=error)
        self.count += 1
        self.invalid += not event['valid']
        event.update(index=self.count, time=datetime.now().isoformat(timespec='milliseconds'),
            interval_ms=(now-self.last_time)*1000 if self.last_time is not None else None,
            deadline_remaining_ms=(deadline-unix_ns())/1e6 if type(deadline) is int else None,
            sequence_note='first', physical_sent=False)
        self.last_time = now
        self.recent.append(now)
        sequence = event.get('sequence')
        if event['valid'] and sequence is not None:
            if self.last_sequence is not None:
                delta = (sequence-self.last_sequence) & 255
                event['sequence_note'] = ('OK' if delta == 1 else 'duplicate' if delta == 0
                    else f'gap +{delta-1}' if delta < 128 else 'backward')
                self.sequence_errors += delta != 1
            self.last_sequence = sequence
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(event, ensure_ascii=False)+'\n')
        if self.events.full():
            try:self.events.get_nowait()
            except queue.Empty:pass
            self.display_dropped += 1
        self.events.put_nowait(event)

    def rate(self):
        rows = list(self.recent)
        if len(rows) < 2 or time.perf_counter()-rows[-1] > 1:
            return 0.
        return (len(rows)-1)/(rows[-1]-rows[0])
