"""Linux SocketCAN FD adapter. byte length 32, never raw DLC code 13."""
import json
import socket
import struct
import subprocess
import time

from .protocol import CAN_ID, check_packet


def socketcan_frame(data):
    check_packet(data)
    # linux/can.h: can_id, len, flags(CANFD_BRS), reserved[2], data[64].
    return struct.pack('=IBBBB64s', CAN_ID, 32, 1, 0, 0, bytes(data))


class SocketCanDriver:
    def __init__(self, interface, allow_vcan=False):
        links = json.loads(subprocess.check_output(
            ['ip', '-details', '-json', 'link', 'show', 'dev', interface], text=True))
        info = links[0]['linkinfo']
        if 'UP' not in links[0].get('flags', []) or links[0].get('mtu') != 72:
            raise ValueError('CAN interface must be UP with FD MTU 72')
        if info['info_kind'] == 'vcan' and allow_vcan:
            pass
        elif info['info_kind'] == 'can':
            details = info['info_data']
            if details.get('state') != 'ERROR-ACTIVE':
                raise ValueError('CAN controller is not ERROR-ACTIVE')
            if (details['bittiming']['bitrate'] != 500000 or
                    details['data_bittiming']['bitrate'] != 2000000):
                raise ValueError('CAN requires arbitration 500k and data 2M')
        else:
            raise ValueError('Expected CAN FD; vcan requires explicit allow_vcan')
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            self.socket.setblocking(False)
            self.socket.bind((interface,))
        except Exception:
            self.socket.close()
            raise

    def send(self, data, deadline_monotonic_ns):
        frame = socketcan_frame(data)
        if time.monotonic_ns() >= deadline_monotonic_ns:
            raise ValueError('Path deadline expired before SocketCAN write')
        if self.socket.send(frame) != len(frame):
            raise OSError('Short CAN FD write')
        return True

    def close(self):
        self.socket.close()
