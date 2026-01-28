from .transport import SerialTransport
from .protocol import unpack_frame


class FieldView:
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud

    def _open(self):
        return SerialTransport(self.port, self.baud)

    def ping(self):
        t = self._open()
        r = ""
        for _ in range(3):
            t.write_line("PING")
            r = t.read_expected_text(("OK", "ERR"), timeout_s=1.5)
            if r:
                break
        t.close()
        return r

    def info(self):
        t = self._open()
        r = ""
        for _ in range(3):
            t.write_line("INFO")
            r = t.read_expected_text(("INFO", "OK", "ERR"), timeout_s=1.5)
            if r:
                break
        t.close()
        return r

    def read(self):
        t = self._open()
        t.write_line("READ")
        while True:
            raw = t.read_bin_frame()
            if raw:
                _, _, arr = unpack_frame(raw)
                t.close()
                return arr

    def start(self, hz):
        t = self._open()
        t.write_line(f"START {hz}")
        t.ser.readline()
        return t  # streaming handle (user must close)

    def stop(self, t):
        t.write_line("STOP")
        t.ser.readline()
        t.close()
