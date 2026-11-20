import ctypes
import struct
import time
from pathlib import Path


PCAN_USBBUS1 = 0x51
PCAN_BAUD_500K = 0x001C
PCAN_MESSAGE_STANDARD = 0x00
PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020


class TPCANMsg(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint32),
        ("MSGTYPE", ctypes.c_ubyte),
        ("LEN", ctypes.c_ubyte),
        ("DATA", ctypes.c_ubyte * 8),
    ]


class TPCANTimestamp(ctypes.Structure):
    _fields_ = [
        ("millis", ctypes.c_uint32),
        ("millis_overflow", ctypes.c_uint16),
        ("micros", ctypes.c_uint16),
    ]


def load_api():
    candidates = [
        Path(r"C:\Windows\System32\PCANBasic.dll"),
        Path(r"C:\Program Files\PEAK-System\PCAN-Basic\x64\PCANBasic.dll"),
        Path(r"C:\Infineon\Tools\OneEye\2.56.0.202303101530\bin\bin\PCANBasic.dll"),
    ]
    for candidate in candidates:
        try:
            api = ctypes.WinDLL(str(candidate))
            api.CAN_Initialize.argtypes = [ctypes.c_ushort, ctypes.c_ushort,
                                           ctypes.c_ubyte, ctypes.c_uint32,
                                           ctypes.c_ushort]
            api.CAN_Initialize.restype = ctypes.c_uint32
            api.CAN_Uninitialize.argtypes = [ctypes.c_ushort]
            api.CAN_Uninitialize.restype = ctypes.c_uint32
            api.CAN_Write.argtypes = [ctypes.c_ushort, ctypes.POINTER(TPCANMsg)]
            api.CAN_Write.restype = ctypes.c_uint32
            api.CAN_Read.argtypes = [ctypes.c_ushort, ctypes.POINTER(TPCANMsg),
                                     ctypes.POINTER(TPCANTimestamp)]
            api.CAN_Read.restype = ctypes.c_uint32
            return api, candidate
        except OSError:
            pass
    raise RuntimeError("PCANBasic.dll not found")


def make_command(sequence, enabled):
    # Zero-speed/zero-steering acceptance test: no traction request is created.
    payload = struct.pack("<hhHBB", 0, 0, 1500, sequence, 0xA1 if enabled else 0xA0)
    msg = TPCANMsg()
    msg.ID = 0x300
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD
    msg.LEN = 8
    for index, value in enumerate(payload):
        msg.DATA[index] = value
    return msg, payload


def drain(api):
    msg = TPCANMsg()
    timestamp = TPCANTimestamp()
    while api.CAN_Read(PCAN_USBBUS1, ctypes.byref(msg), ctypes.byref(timestamp)) == PCAN_ERROR_OK:
        pass


def collect(api, seconds, latest, counts):
    deadline = time.monotonic() + seconds
    msg = TPCANMsg()
    timestamp = TPCANTimestamp()
    while time.monotonic() < deadline:
        status = api.CAN_Read(PCAN_USBBUS1, ctypes.byref(msg), ctypes.byref(timestamp))
        if status == PCAN_ERROR_OK:
            data = bytes(msg.DATA[:msg.LEN])
            latest[msg.ID] = data
            counts[msg.ID] = counts.get(msg.ID, 0) + 1
        elif status == PCAN_ERROR_QRCVEMPTY:
            time.sleep(0.001)
        else:
            raise RuntimeError(f"CAN_Read failed: 0x{status:08X}")


def transmit_phase(api, sequence, enabled, frames, latest, counts):
    for _ in range(frames):
        msg, payload = make_command(sequence, enabled)
        status = api.CAN_Write(PCAN_USBBUS1, ctypes.byref(msg))
        if status != PCAN_ERROR_OK:
            raise RuntimeError(f"CAN_Write failed: 0x{status:08X}")
        print(f"TX 300  {' '.join(f'{byte:02X}' for byte in payload)}")
        sequence = (sequence + 1) & 0xFF
        collect(api, 0.05, latest, counts)
    return sequence


def s16(data, offset):
    return struct.unpack_from("<h", data, offset)[0]


def decode(latest):
    result = {}
    if 0x131 in latest and len(latest[0x131]) == 8:
        data = latest[0x131]
        result["vm_state"] = data[0]
        result["vm_owner"] = data[1]
        result["vm_fault"] = struct.unpack_from("<I", data, 2)[0]
        result["vm_drive"] = data[6] & 0x01
    if 0x132 in latest and len(latest[0x132]) == 8:
        data = latest[0x132]
        result["requested_speed"] = s16(data, 0) / 100.0
        result["target_speed"] = s16(data, 2) / 100.0
        result["requested_steer"] = s16(data, 4) / 10.0
        result["target_steer"] = s16(data, 6) / 10.0
    if 0x130 in latest and len(latest[0x130]) == 8:
        data = latest[0x130]
        result["pc_active"] = (data[7] >> 2) & 0x01
        result["measured_speed"] = s16(data, 0) / 100.0
    if 0x134 in latest and len(latest[0x134]) == 8:
        data = latest[0x134]
        result["graph_mode"] = data[0]
        result["graph_state"] = data[1]
        result["graph_active"] = data[6] & 0x01
    return result


def main():
    api, dll_path = load_api()
    status = api.CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0)
    if status != PCAN_ERROR_OK:
        raise RuntimeError(f"CAN_Initialize failed: 0x{status:08X}")

    latest = {}
    counts = {}
    sequence = int(time.time() * 1000.0) & 0xFF
    print(f"PCANBasic={dll_path}")
    print("Safety test: command speed remains 0.00 km/h for every frame.")
    try:
        drain(api)
        sequence = transmit_phase(api, sequence, False, 5, latest, counts)
        sequence = transmit_phase(api, sequence, True, 10, latest, counts)
        enabled_result = decode(latest)
        sequence = transmit_phase(api, sequence, False, 5, latest, counts)
        stop_result = decode(latest)
    finally:
        # Best-effort final canonical STOP with a fresh sequence.
        try:
            msg, _ = make_command(sequence, False)
            api.CAN_Write(PCAN_USBBUS1, ctypes.byref(msg))
        finally:
            api.CAN_Uninitialize(PCAN_USBBUS1)

    print("RX counts:", " ".join(f"{can_id:03X}={count}" for can_id, count in sorted(counts.items())))
    print("Enabled-zero response:", enabled_result)
    print("Final-stop response:", stop_result)

    required = (0x130, 0x131, 0x132)
    if not all(can_id in latest for can_id in required):
        print("RESULT=FAIL missing TC275 feedback frame")
        return 2
    if (enabled_result.get("vm_state") == 4 and
            enabled_result.get("vm_owner") == 2 and
            enabled_result.get("vm_fault") == 0 and
            enabled_result.get("vm_drive") == 1 and
            enabled_result.get("pc_active") == 1 and
            enabled_result.get("requested_speed") == 0.0 and
            enabled_result.get("target_speed") == 0.0):
        print("RESULT=PASS 0x300 accepted and VM CAN ownership acknowledged")
        return 0

    if enabled_result.get("graph_active") == 1:
        print("RESULT=BLOCKED GraphTest is active and intentionally masks CAN commands")
        return 3
    print("RESULT=FAIL TC275 replied but did not acknowledge CAN ownership")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
