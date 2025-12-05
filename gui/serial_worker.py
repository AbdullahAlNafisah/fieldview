from __future__ import annotations
from PySide6.QtCore import QThread, Signal
import json
import serial


class SerialWorker(QThread):
    """
    Background thread expects each line to be a JSON array like:
    [
      {"idx":0,"x":...,"y":...,"z":...,"temp":...},
      ...
    ]
    """

    data_received = Signal(list)  # list of dicts
    error = Signal(str)

    def __init__(self, port: str, baud: int, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._stop = False

    def run(self):
        try:
            ser = serial.Serial(self.port, baudrate=self.baud, timeout=1.0)
        except Exception as e:
            self.error.emit(f"Failed to open port {self.port}: {e}")
            return

        while not self._stop:
            try:
                line = ser.readline()
                if not line:
                    continue

                line = line.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                arr = json.loads(line)
                if isinstance(arr, list):
                    self.data_received.emit(arr)

            except json.JSONDecodeError:
                # Bad/partial line -> ignore
                continue
            except Exception as e:
                if not self._stop:
                    self.error.emit(str(e))
                break

        ser.close()

    def stop(self):
        self._stop = True
        self.wait(1000)  # wait up to 1s for thread to finish
