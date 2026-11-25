"""High resolution Unix time on Windows Python versions with a coarse time.time clock."""
import ctypes
import os
import time

if os.name == 'nt':
    _precise_time = ctypes.WinDLL('kernel32').GetSystemTimePreciseAsFileTime
    _precise_time.argtypes = [ctypes.POINTER(ctypes.c_uint64)]
    _precise_time.restype = None


def unix_ns():
    if os.name == 'nt':
        value = ctypes.c_uint64()
        _precise_time(ctypes.byref(value))
        return (value.value-116444736000000000)*100
    return time.time_ns()
