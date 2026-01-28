import struct

FMT = "<2sHI24fH"
SIZE = struct.calcsize(FMT)
SYNC = b"\xaa\x55"


def checksum16(buf: bytes) -> int:
    return sum(buf) & 0xFFFF


def unpack_frame(frame: bytes):
    if len(frame) != SIZE:
        raise ValueError("Bad frame size")

    sync, fid, ts, *vals, cs = struct.unpack(FMT, frame)
    if sync != SYNC:
        raise ValueError("Bad sync")

    payload = struct.pack("<HI24f", fid, ts, *vals)
    if checksum16(payload) != cs:
        raise ValueError("Bad checksum")

    # return (frame_id, timestamp_ms, np.array shape (6,4))
    import numpy as np

    arr = np.array(vals, dtype=np.float32).reshape(6, 4)
    return fid, ts, arr
