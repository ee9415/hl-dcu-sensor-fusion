"""Windows Kvaser CANlib FD adapter; importing/enumerating never goes bus-on."""
import ctypes
import os
import time

from .protocol import CAN_ID, check_packet
from .timebase import unix_ns

DEFAULT_DLL = r'C:\Program Files\Kvaser\Drivers\canlib32.dll'


class Canlib:
    def __init__(self, path=DEFAULT_DLL):
        if os.name != 'nt':
            raise OSError('Kvaser Windows CANlib must run on Windows')
        self.dll = ctypes.WinDLL(path)
        signatures = {
            'canInitializeLibrary': (None, []),
            'canGetNumberOfChannels': (ctypes.c_int, [ctypes.POINTER(ctypes.c_int)]),
            'canGetChannelData': (ctypes.c_int, [ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t]),
            'canOpenChannel': (ctypes.c_int, [ctypes.c_int, ctypes.c_int]),
            'canSetBusParams': (ctypes.c_int, [ctypes.c_int, ctypes.c_long]+[ctypes.c_uint]*5),
            'canSetBusParamsFd': (ctypes.c_int, [ctypes.c_int, ctypes.c_long]+[ctypes.c_uint]*3),
            'canSetBusOutputControl': (ctypes.c_int, [ctypes.c_int, ctypes.c_uint]),
            'canBusOn': (ctypes.c_int, [ctypes.c_int]),
            'canBusOff': (ctypes.c_int, [ctypes.c_int]),
            'canClose': (ctypes.c_int, [ctypes.c_int]),
            'canIoCtl': (ctypes.c_int, [ctypes.c_int, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
            'canWriteWait': (ctypes.c_int, [ctypes.c_int, ctypes.c_long, ctypes.c_void_p,
                ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong]),
            'canReadWait': (ctypes.c_int, [ctypes.c_int, ctypes.POINTER(ctypes.c_long), ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint),
                ctypes.POINTER(ctypes.c_ulong), ctypes.c_ulong]),
        }
        for name, (result, arguments) in signatures.items():
            function = getattr(self.dll, name)
            function.restype, function.argtypes = result, arguments
        self.dll.canInitializeLibrary()

    @staticmethod
    def check(status):
        if status < 0:
            raise OSError('Kvaser CANlib status '+str(status))
        return status

    def channels(self):
        count = ctypes.c_int()
        self.check(self.dll.canGetNumberOfChannels(ctypes.byref(count)))
        result = []
        for channel in range(count.value):
            name, caps = ctypes.create_string_buffer(256), ctypes.c_uint32()
            self.check(self.dll.canGetChannelData(channel, 13, name, 256))
            self.check(self.dll.canGetChannelData(channel, 1, ctypes.byref(caps), 4))
            result.append(dict(channel=channel, name=name.value.decode(errors='replace'),
                can_fd=bool(caps.value & 0x80000), virtual=bool(caps.value & 0x10000)))
        return result


class KvaserDriver:
    def __init__(self, channel=0, virtual=False, dll=DEFAULT_DLL):
        self.api = Canlib(dll)
        self.handle = None
        channels = self.api.channels()
        matches = [c for c in channels if c['channel'] == channel]
        if len(matches) != 1 or not matches[0]['can_fd'] or matches[0]['virtual'] != virtual:
            raise ValueError('Selected channel must support FD and match explicit virtual/physical selection')
        self.virtual = virtual
        d = self.api.dll
        # ISO CAN FD, exclusive, require init access; never accept virtual implicitly.
        flags = 0x400 | 0x8 | 0x80 | (0x20 if virtual else 0)
        self.handle = self.api.check(d.canOpenChannel(channel, flags))
        try:
            self.api.check(d.canSetBusParams(self.handle, -1000, 0, 0, 0, 0, 0))
            self.api.check(d.canSetBusParamsFd(self.handle, -1002, 0, 0, 0))
            self.api.check(d.canSetBusOutputControl(self.handle, 4))
            self.api.check(d.canBusOn(self.handle))
        except Exception:
            self.close()
            raise

    def send(self, data, deadline_unix_ns):
        check_packet(data)
        if self.handle is None:
            raise OSError('Kvaser channel is closed')
        # Leave 20ms for the synchronous hardware write/ack. Never keep a
        # timed-out ACTIVE frame in a queue and silently retry it later.
        if unix_ns()+20_000_000 >= deadline_unix_ns:
            raise ValueError('CAN deadline leaves insufficient hardware transmit budget')
        payload = (ctypes.c_ubyte*32).from_buffer_copy(data)
        status = self.api.dll.canWriteWait(self.handle, CAN_ID, payload, 32, 0x30002, 20)
        if status < 0:
            self.close()
            self.api.check(status)

    def close(self):
        if self.handle is not None:
            d, handle = self.api.dll, self.handle
            self.handle = None
            d.canIoCtl(handle, 11, None, 0)  # Flush pending TX before bus-off/close.
            d.canBusOff(handle)
            d.canClose(handle)
