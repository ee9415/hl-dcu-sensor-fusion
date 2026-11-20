import argparse
import ctypes
import os
import sys
import time
from pathlib import Path


PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020
PCAN_MESSAGE_STANDARD = 0x00
DEFAULT_CYCLE_MS = 50.0
DEFAULT_RECONNECT_MS = 200.0

CHANNELS = {
    "PCAN_USBBUS1": 0x51,
    "PCAN_USBBUS2": 0x52,
    "PCAN_USBBUS3": 0x53,
    "PCAN_USBBUS4": 0x54,
    "PCAN_USBBUS5": 0x55,
    "PCAN_USBBUS6": 0x56,
    "PCAN_USBBUS7": 0x57,
    "PCAN_USBBUS8": 0x58,
}

BITRATES = {
    "1M": 0x0014,
    "800K": 0x0016,
    "500K": 0x001C,
    "250K": 0x011C,
    "125K": 0x031C,
    "100K": 0x432F,
    "50K": 0x472F,
}


class TPCANMsg(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint32),
        ("MSGTYPE", ctypes.c_ubyte),
        ("LEN", ctypes.c_ubyte),
        ("DATA", ctypes.c_ubyte * 8),
    ]


def parse_int(value):
    return int(str(value), 0)


def parse_channel(value):
    text = str(value).upper()
    if text in CHANNELS:
        return CHANNELS[text]
    if text.isdigit():
        index = int(text)
        if 1 <= index <= 8:
            return CHANNELS[f"PCAN_USBBUS{index}"]
    return parse_int(value)


def parse_data(value):
    if value is None or value.strip() == "":
        return [0] * 8

    parts = value.replace(",", " ").split()
    data = [parse_int(part) & 0xFF for part in parts]
    if len(data) > 8:
        raise ValueError("CAN data must be 8 bytes or fewer")
    return data + ([0] * (8 - len(data)))


def dll_candidates():
    env_path = os.environ.get("PCANBASIC_DLL")
    if env_path:
        yield Path(env_path)

    yield Path("PCANBasic.dll")
    yield Path(r"C:\Windows\System32\PCANBasic.dll")
    yield Path(r"C:\Windows\SysWOW64\PCANBasic.dll")
    yield Path(r"C:\Program Files\PEAK-System\PCAN-Basic\x64\PCANBasic.dll")
    yield Path(r"C:\Program Files (x86)\PEAK-System\PCAN-Basic\x64\PCANBasic.dll")
    yield Path(r"C:\Infineon\Tools\OneEye\2.56.0.202303101530\bin\bin\PCANBasic.dll")


def load_pcanbasic(path=None):
    candidates = [Path(path)] if path else list(dll_candidates())
    errors = []

    for candidate in candidates:
        try:
            dll = ctypes.WinDLL(str(candidate))
            return dll, str(candidate)
        except OSError as exc:
            errors.append(f"{candidate}: {exc}")

    raise RuntimeError("PCANBasic.dll not found/loadable.\n" + "\n".join(errors))


def configure_api(dll):
    dll.CAN_Initialize.argtypes = [
        ctypes.c_ushort,
        ctypes.c_ushort,
        ctypes.c_ubyte,
        ctypes.c_uint32,
        ctypes.c_ushort,
    ]
    dll.CAN_Initialize.restype = ctypes.c_uint32

    dll.CAN_Uninitialize.argtypes = [ctypes.c_ushort]
    dll.CAN_Uninitialize.restype = ctypes.c_uint32

    dll.CAN_Write.argtypes = [ctypes.c_ushort, ctypes.POINTER(TPCANMsg)]
    dll.CAN_Write.restype = ctypes.c_uint32

    if hasattr(dll, "CAN_GetErrorText"):
        dll.CAN_GetErrorText.argtypes = [ctypes.c_uint32, ctypes.c_ushort, ctypes.c_char_p]
        dll.CAN_GetErrorText.restype = ctypes.c_uint32


def error_text(dll, status):
    if not hasattr(dll, "CAN_GetErrorText"):
        return f"0x{status:08X}"

    buffer = ctypes.create_string_buffer(256)
    result = dll.CAN_GetErrorText(status, 0, buffer)
    if result == PCAN_ERROR_OK:
        return buffer.value.decode(errors="replace")
    return f"0x{status:08X}"


def make_message(can_id, data):
    msg = TPCANMsg()
    msg.ID = can_id
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD
    msg.LEN = 8
    for i, value in enumerate(data[:8]):
        msg.DATA[i] = value
    return msg


def initialize_channel(dll, channel, bitrate):
    return dll.CAN_Initialize(channel, bitrate, 0, 0, 0)


def should_print_retry(now, next_retry_print):
    return now >= next_retry_print


def main():
    parser = argparse.ArgumentParser(
        description="Send cyclic CAN command/alive frame to AURIX via PEAK PCANBasic."
    )
    parser.add_argument("--channel", default="PCAN_USBBUS1", help="PCAN channel, e.g. PCAN_USBBUS1 or 1")
    parser.add_argument("--bitrate", default="500K", choices=sorted(BITRATES), help="CAN bitrate")
    parser.add_argument("--id", default="0x300", help="Standard CAN ID. Firmware watches 0x300 by default.")
    parser.add_argument("--data", default="00 00 00 00 00 00 00 00", help="Up to 8 data bytes")
    parser.add_argument("--cycle-ms", type=float, default=DEFAULT_CYCLE_MS, help="Transmit cycle in milliseconds")
    parser.add_argument("--reconnect-ms", type=float, default=DEFAULT_RECONNECT_MS, help="Reconnect retry cycle in milliseconds")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means until Ctrl+C")
    parser.add_argument("--dll", default=None, help="Optional full path to PCANBasic.dll")
    args = parser.parse_args()

    channel = parse_channel(args.channel)
    bitrate = BITRATES[args.bitrate.upper()]
    can_id = parse_int(args.id)
    data = parse_data(args.data)
    cycle_sec = max(args.cycle_ms, 1.0) / 1000.0
    reconnect_sec = max(args.reconnect_ms, 1.0) / 1000.0

    dll, dll_path = load_pcanbasic(args.dll)
    configure_api(dll)

    msg = make_message(can_id, data)
    print(f"PCANBasic: {dll_path}")
    print(f"Sending ID 0x{can_id:X}, data {' '.join(f'{b:02X}' for b in data)}, every {args.cycle_ms:g} ms")
    print(f"Reconnect retry every {args.reconnect_ms:g} ms when disconnected.")
    print("Stop with Ctrl+C.")

    sent = 0
    errors = 0
    reconnects = 0
    connected = False
    started = time.monotonic()
    next_print = started + 1.0
    next_retry_print = started
    next_action = started

    try:
        while True:
            now = time.monotonic()

            if args.duration > 0.0 and (now - started) >= args.duration:
                break

            if connected:
                if now < next_action:
                    time.sleep(min(next_action - now, cycle_sec))
                    continue

                status = dll.CAN_Write(channel, ctypes.byref(msg))
                if status == PCAN_ERROR_OK:
                    sent += 1
                    next_action = now + cycle_sec
                else:
                    errors += 1
                    print(f"CAN_Write lost connection #{errors}: {error_text(dll, status)}")
                    dll.CAN_Uninitialize(channel)
                    connected = False
                    next_action = now + reconnect_sec
            else:
                if now < next_action:
                    time.sleep(min(next_action - now, reconnect_sec))
                    continue

                status = initialize_channel(dll, channel, bitrate)
                if status == PCAN_ERROR_OK:
                    connected = True
                    reconnects += 1
                    print(f"PCAN connected. reconnects={reconnects}")
                    next_action = now
                else:
                    if should_print_retry(now, next_retry_print):
                        print(f"CAN_Initialize retry failed: {error_text(dll, status)}")
                        next_retry_print = now + 1.0
                    next_action = now + reconnect_sec

            if now >= next_print:
                print(f"sent={sent}, errors={errors}, reconnects={reconnects}, connected={int(connected)}")
                next_print = now + 1.0

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        dll.CAN_Uninitialize(channel)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
