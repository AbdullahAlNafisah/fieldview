import time
import serial
from .protocol import SIZE


class SerialTransport:
    def __init__(self, port, baud=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

        # try to avoid auto-reset toggles
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass

        # give the board time if it rebooted on open
        time.sleep(0.35)

        # drain whatever boot/READY text arrived
        end = time.time() + 0.25
        while time.time() < end:
            _ = self.ser.read(self.ser.in_waiting or 1)
            time.sleep(0.01)


    def write_line(self, s: str):
        self.ser.write((s + "\n").encode())

    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                return b""
            buf.extend(chunk)
        return bytes(buf)

    def read_expected_text(self, prefixes=("OK", "INFO", "ERR"), timeout_s=1.0):
        end = time.time() + timeout_s
        while time.time() < end:
            line = self.ser.readline()
            if not line:
                continue
            s = line.decode("utf-8", errors="ignore").strip()
            if not s:
                continue
            if s.startswith(prefixes):
                return s
        return ""  # timeout

    def read_bin_frame(self):
        while True:
            line = self.ser.readline()
            if not line:
                return None
            if line.startswith(b"BIN "):
                tail = line[4:]
                if len(tail) >= SIZE:
                    frame = tail[:SIZE]
                else:
                    frame = tail + self._read_exact(SIZE - len(tail))
                return frame if len(frame) == SIZE else None

    def close(self):
        self.ser.close()
