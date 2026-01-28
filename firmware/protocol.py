import struct
import time

SYNC = b'\xAA\x55'
FMT = '<2sHI24fH'   # NO SPACES
SIZE = struct.calcsize(FMT)

_frame_id = 0

def checksum16(buf):
    return sum(buf) & 0xFFFF

def pack_frame(values):
    global _frame_id
    _frame_id = (_frame_id + 1) & 0xFFFF
    ts = time.ticks_ms() & 0xFFFFFFFF

    payload = struct.pack('<HI24f', _frame_id, ts, *values)
    cs = checksum16(payload)
    return struct.pack(FMT, SYNC, _frame_id, ts, *values, cs)
