"""Versioned, bounded, lossless frame bundles shared by OAK and ROS (no DepthAI)."""
import json
import struct
import zlib

import numpy as np

MAGIC = b'OAKP\x00\x01'
MAX_PACKET = 24 * 1024 * 1024
MAX_HEADER = 64 * 1024
MAX_ARRAY_BYTES = 16 * 1024 * 1024
DTYPES = {'|u1', '<u2'}


def encode_bundle(metadata, arrays):
    header = dict(metadata, schema_version=1, arrays={})
    chunks = []
    for name, value in arrays.items():
        value = np.ascontiguousarray(value)
        if value.dtype.str not in DTYPES or value.nbytes > MAX_ARRAY_BYTES:
            raise ValueError('Unsupported array type or size')
        # RGB is high entropy; the gigabit link can carry it losslessly raw.
        codec = 'raw' if name == 'rgb' else 'zlib'
        chunk = value.tobytes() if codec == 'raw' else zlib.compress(value.tobytes(), 1)
        header['arrays'][name] = dict(shape=list(value.shape), dtype=value.dtype.str,
                                     size=len(chunk), codec=codec)
        chunks.append(chunk)
    raw = json.dumps(header, allow_nan=False, separators=(',', ':')).encode()
    if len(raw) > MAX_HEADER:
        raise ValueError('Header too large')
    body = struct.pack('!I', len(raw)) + raw + b''.join(chunks)
    if len(body) > MAX_PACKET:
        raise ValueError('Packet too large')
    return MAGIC + struct.pack('!I', len(body)) + body


def decode_bundle(body):
    if not 4 <= len(body) <= MAX_PACKET:
        raise ValueError('Invalid packet length')
    size = struct.unpack('!I', body[:4])[0]
    if not 0 < size <= min(MAX_HEADER, len(body)-4):
        raise ValueError('Invalid header length')
    header = json.loads(body[4:4+size])
    if header.get('schema_version') != 1:
        raise ValueError('Unsupported schema')
    offset = 4 + size
    arrays = {}
    for name, spec in header['arrays'].items():
        shape, dtype, length = spec['shape'], spec['dtype'], spec['size']
        if (dtype not in DTYPES or spec.get('codec') not in ('zlib', 'raw') or
                not isinstance(shape, list) or not 1 <= len(shape) <= 3 or
                any(type(n) is not int or not 0 < n <= 4096 for n in shape) or
                type(length) is not int or not 0 < length <= len(body)-offset):
            raise ValueError('Invalid array descriptor')
        expected = int(np.prod(shape)) * np.dtype(dtype).itemsize
        if expected > MAX_ARRAY_BYTES:
            raise ValueError('Expanded array too large')
        if spec['codec'] == 'raw':
            if length != expected:
                raise ValueError('Invalid raw array length')
            raw = body[offset:offset+length]
        else:
            decompressor = zlib.decompressobj()
            raw = decompressor.decompress(body[offset:offset+length], expected+1)
            if (len(raw) != expected or not decompressor.eof or
                    decompressor.unused_data or decompressor.unconsumed_tail):
                raise ValueError('Invalid compressed array')
        arrays[name] = np.frombuffer(raw, dtype=dtype).reshape(shape)
        offset += length
    if offset != len(body):
        raise ValueError('Trailing packet bytes')
    return header, arrays


def recv_exact(connection, count):
    result = bytearray()
    while len(result) < count:
        chunk = connection.recv(count-len(result))
        if not chunk:
            raise EOFError('Camera disconnected')
        result.extend(chunk)
    return bytes(result)


def receive_bundle(connection):
    prefix = recv_exact(connection, len(MAGIC)+4)
    if prefix[:len(MAGIC)] != MAGIC:
        raise ValueError('Invalid protocol magic')
    length = struct.unpack('!I', prefix[len(MAGIC):])[0]
    if not 4 <= length <= MAX_PACKET:
        raise ValueError('Invalid packet length')
    return decode_bundle(recv_exact(connection, length))
