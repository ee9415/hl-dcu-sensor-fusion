"""TC275 1113_fix/1024 camera trajectory, independent of ROS and CAN libraries."""
import math
import struct

CAN_ID = 0x320
PAYLOAD_BYTES = 32
DLC_CODE = 13
PERIOD_S = .05
FRAME_ID = 'vehicle_front_axle'


def crc8(data):
    value = 0xff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x1d if value & 0x80 else 0)) & 0xff
    return value ^ 0xff


def validate_path(coefficients, length, confidence):
    if len(coefficients) != 4 or not all(math.isfinite(c) for c in coefficients):
        raise ValueError('nonfinite_coefficients')
    # Check the float32 values the receiver actually sees as well as input doubles.
    wire = struct.unpack('<4f', struct.pack('<4f', *coefficients))
    if any(abs(c) > limit for c, limit in zip(coefficients, (3, 2, 5, 5))) or any(
            not math.isfinite(c) or abs(c) > limit for c, limit in zip(wire, (3, 2, 5, 5))):
        raise ValueError('coefficient_range')
    if not math.isfinite(length) or not .5 <= length <= 12:
        raise ValueError('length_range')
    if type(confidence) is not int or not 1 <= confidence <= 100:
        raise ValueError('confidence_range')


def packet(sequence, active=False, coefficients=(0., 0., 0., 0.), length=0.,
           confidence=0, min_speed=0., max_speed=3.):
    if type(sequence) is not int or not 0 <= sequence <= 255:
        raise ValueError('sequence_range')
    data = bytearray(PAYLOAD_BYTES)
    data[0], data[1] = (0xd9 if active else 0x81), sequence
    if active:
        validate_path(coefficients, length, confidence)
        if not (math.isfinite(min_speed) and math.isfinite(max_speed) and
                0 <= min_speed <= max_speed <= 3):
            raise ValueError('speed_range')
        data[2] = 2
        struct.pack_into('<4fhhH', data, 4, *coefficients,
            round(min_speed*100), round(max_speed*100), math.floor(length*100))
        data[26] = confidence
    data[31] = crc8(CAN_ID.to_bytes(2, 'big')+data[:31])
    return bytes(data)


def check_packet(data):
    if len(data) != PAYLOAD_BYTES or crc8(b'\x03\x20'+bytes(data[:31])) != data[31]:
        raise ValueError('length_or_crc')
    if any(data[27:31]) or data[3] != 0:
        raise ValueError('context_or_reserved')
    if data[0] == 0x81:
        if any(data[2:31]):
            raise ValueError('inactive_payload')
    elif data[0] == 0xd9 and data[2] == 2:
        values = struct.unpack_from('<4fhhH', data, 4)
        validate_path(values[:4], values[6]/100, int(data[26]))
        if not 0 <= values[4] <= values[5] <= 300:
            raise ValueError('speed_range')
    else:
        raise ValueError('header_or_status')
