#!/usr/bin/env python3
"""
MAG3110 x4 — Live 3D viewer (PyQtGraph + PyQt6)

- Opens a serial port to your Arduino Mega
- Sends 'S' (START) once, then 'R' (REQ) once per GUI tick to pull one batch
- Parses "BusN: X=.. Y=.. Z=.." lines (even if concatenated)
- Plots 4 x 3D point clouds (one per bus), optional autoscale
- Buttons: Connect/Disconnect, Pause, Reset All, per-bus Reset

Install:
  pip install pyqtgraph PyQt6 pyserial PyOpenGL PyOpenGL_accelerate
Linux OpenGL libs (if needed):
  sudo apt-get install -y libgl1-mesa-glx libglu1-mesa freeglut3 mesa-utils
"""

import sys, re, threading, queue, time, argparse
from collections import deque

import numpy as np
from PyQt6 import QtWidgets, QtCore
from PyQt6.QtGui import QVector3D
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from serial.tools import list_ports
import serial

# -------------------- Config --------------------
DEFAULT_BAUD = 115200  # must match firmware uart0_init(...)
UPDATE_FPS = 60  # GUI update + request rate
MAX_POINTS_PER_BUS = 8000  # rolling history per bus
SHOW_RAW_BY_DEFAULT = False  # set True for debugging

# Accept lines like: Bus0: X=123 Y=-45 Z=678  (spaces/commas flexible)
LINE_RE = re.compile(
    r"Bus\s*(?P<bus>[0-3])\s*[:\-]?\s*"
    r"X\s*[:=]\s*(?P<X>-?\d+)\s*[, ]+\s*"
    r"Y\s*[:=]\s*(?P<Y>-?\d+)\s*[, ]+\s*"
    r"Z\s*[:=]\s*(?P<Z>-?\d+)",
    re.IGNORECASE,
)

COLORS = [
    (1.0, 0.2, 0.2, 0.9),  # Bus 0
    (0.2, 0.8, 0.2, 0.9),  # Bus 1
    (0.2, 0.4, 1.0, 0.9),  # Bus 2
    (1.0, 0.6, 0.2, 0.9),  # Bus 3
]


# -------------------- Background serial reader --------------------
class SerialReader(threading.Thread):
    """
    Runs in a background thread:
    - Opens serial
    - Emits ('RAW', text) for any incoming bytes (optional UI logging)
    - Extracts any "BusN: X= Y= Z=" matches and emits ('XYZ', (bus,x,y,z))
    - Accepts outbound commands via in_q (e.g., "S", "R", "T")
    """

    def __init__(self, port, baud, out_q, stop_event, in_q=None):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_q = out_q
        self.in_q = in_q or queue.Queue()
        self.stop_event = stop_event
        self.ser = None

    def run(self):
        # 1) Open port
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as e:
            self.out_q.put(("ERR", f"Serial open failed: {e!r}"))
            return

        # 2) Let bootloader reset finish
        time.sleep(2.0)
        self.out_q.put(("INFO", f"Connected {self.port}@{self.baud}"))

        buf = ""

        def pump_incoming():
            """Read bytes, emit RAW, and parse any BusN records even if glued."""
            nonlocal buf
            try:
                chunk = self.ser.read(4096)
            except Exception as e:
                self.out_q.put(("ERR", f"Serial read error: {e!r}"))
                return False
            if not chunk:
                return True

            try:
                text = chunk.decode("utf-8", errors="ignore")
            except Exception as e:
                self.out_q.put(("ERR", f"Decode error: {e!r}"))
                return True

            buf += text
            # show raw input (optional, controlled by UI)
            self.out_q.put(("RAW", text))

            # Extract ALL "BusN ..." matches in buf
            last_end = 0
            for m in LINE_RE.finditer(buf):
                last_end = m.end()
                bus = int(m.group("bus"))
                x = int(m.group("X"))
                y = int(m.group("Y"))
                z = int(m.group("Z"))
                self.out_q.put(("XYZ", (bus, x, y, z)))

            # Trim processed prefix to keep buffer small
            if last_end:
                buf = buf[last_end:]
                if len(buf) > 10000:
                    buf = buf[-5000:]

            return True

        # 3) Main loop: send pending commands + read input
        try:
            while not self.stop_event.is_set():
                # Send any queued outbound commands (non-blocking)
                try:
                    cmd = self.in_q.get_nowait()
                    if isinstance(cmd, str):
                        cmd = cmd.encode("utf-8")
                    if not cmd.endswith(b"\n"):
                        cmd += b"\n"
                    self.ser.write(cmd)
                    self.out_q.put(
                        ("INFO", f"-> {cmd.decode('utf-8','ignore').strip()}")
                    )
                except queue.Empty:
                    pass
                except Exception as e:
                    self.out_q.put(("ERR", f"Serial write error: {e!r}"))

                # Read incoming
                if not pump_incoming():
                    break
        finally:
            try:
                if self.ser:
                    try:
                        self.ser.write(b"T\n")  # STOP (single-letter tolerated)
                        self.out_q.put(("INFO", "-> STOP"))
                        self.ser.flush()
                    except Exception:
                        pass
                    self.ser.close()
            except Exception as e:
                self.out_q.put(("ERR", f"Serial close error: {e!r}"))
            self.out_q.put(("INFO", "Serial closed"))


# -------------------- GUI --------------------
class Viewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MAG3110 4×3D Viewer")
        self.resize(1400, 900)

        # state
        self.paused = False
        self.armed = False
        self.buffers = [deque(maxlen=MAX_POINTS_PER_BUS) for _ in range(4)]

        # threading
        self.q = queue.Queue()  # from SerialReader -> GUI
        self.send_q = queue.Queue()  # from GUI -> SerialReader
        self.stop_event = threading.Event()
        self.reader = None

        # UI
        self._build_ui()

        # update timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / UPDATE_FPS))

        self._refresh_ports()

    # ---------- UI build ----------
    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)

        # Controls row
        ctrl = QtWidgets.QHBoxLayout()
        self.port_cb = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        self.baud_cb = QtWidgets.QComboBox()
        for b in [9600, 19200, 38400, 57600, 115200, 250000]:
            self.baud_cb.addItem(str(b))
        self.baud_cb.setCurrentText(str(DEFAULT_BAUD))

        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.reset_all_btn = QtWidgets.QPushButton("Reset All")
        self.autoscale_cb = QtWidgets.QCheckBox("Auto Scale")
        self.autoscale_cb.setChecked(True)
        self.showraw_cb = QtWidgets.QCheckBox("Show raw")
        self.showraw_cb.setChecked(SHOW_RAW_BY_DEFAULT)

        ctrl.addWidget(QtWidgets.QLabel("Port:"))
        ctrl.addWidget(self.port_cb, 1)
        ctrl.addWidget(self.refresh_btn)
        ctrl.addSpacing(8)
        ctrl.addWidget(QtWidgets.QLabel("Baud:"))
        ctrl.addWidget(self.baud_cb)
        ctrl.addSpacing(8)
        ctrl.addWidget(self.connect_btn)
        ctrl.addWidget(self.disconnect_btn)
        ctrl.addStretch(1)
        ctrl.addWidget(self.pause_btn)
        ctrl.addWidget(self.reset_all_btn)
        ctrl.addSpacing(8)
        ctrl.addWidget(self.autoscale_cb)
        ctrl.addWidget(self.showraw_cb)
        root.addLayout(ctrl)

        # Per-bus reset
        perbus = QtWidgets.QHBoxLayout()
        self.bus_reset_btns = []
        for i in range(4):
            btn = QtWidgets.QPushButton(f"Reset Bus {i}")
            btn.clicked.connect(lambda _, k=i: self._reset_bus(k))
            self.bus_reset_btns.append(btn)
            perbus.addWidget(btn)
        perbus.addStretch()
        root.addLayout(perbus)

        # 3D Views grid
        grid = QtWidgets.QGridLayout()
        self.views, self.scatters = [], []
        for i in range(4):
            view = gl.GLViewWidget()
            view.opts["distance"] = 600
            view.setBackgroundColor((30, 30, 30, 255))
            grid_item = gl.GLGridItem()
            grid_item.scale(20, 20, 1)
            view.addItem(grid_item)
            sp = gl.GLScatterPlotItem(
                pos=np.zeros((1, 3)), size=3.5, color=COLORS[i], pxMode=True
            )
            view.addItem(sp)
            self.views.append(view)
            self.scatters.append(sp)
        grid.addWidget(self.views[0], 0, 0)
        grid.addWidget(self.views[1], 0, 1)
        grid.addWidget(self.views[2], 1, 0)
        grid.addWidget(self.views[3], 1, 1)
        root.addLayout(grid, 1)

        # Log box
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(800)
        root.addWidget(self.log, 0)

        # Wire up
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._connect)
        self.disconnect_btn.clicked.connect(self._disconnect)
        self.pause_btn.toggled.connect(self._toggle_pause)
        self.reset_all_btn.clicked.connect(self._reset_all)

    # ---------- helpers ----------
    def _refresh_ports(self):
        self.port_cb.clear()
        ports = list(list_ports.comports())
        for p in ports:
            self.port_cb.addItem(p.device)
        if not ports:
            self.port_cb.addItem("(no ports found)")

    def _connect(self):
        if self.reader:
            return
        port = self.port_cb.currentText()
        if not port or port.startswith("("):
            self._log("Select a valid port.")
            return
        try:
            baud = int(self.baud_cb.currentText())
        except:
            baud = DEFAULT_BAUD

        self.stop_event.clear()
        self.reader = SerialReader(
            port, baud, self.q, self.stop_event, in_q=self.send_q
        )
        self.reader.start()
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self._log(f"Connecting to {port}@{baud} ...")

        # small delay then send START once; armed flips true when we SEE ACK START in input
        QtCore.QTimer.singleShot(300, lambda: self._send_cmd("S"))

    def _disconnect(self):
        if self.reader:
            self.stop_event.set()
            self.reader.join(timeout=1.5)
            self.reader = None
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.armed = False
        self._log("Disconnected")

    def _toggle_pause(self, ok):
        self.paused = ok
        self.pause_btn.setText("Resume" if ok else "Pause")

    def _reset_bus(self, i):
        self.buffers[i].clear()
        self.scatters[i].setData(pos=np.zeros((1, 3)))
        self._log(f"Reset Bus {i}")

    def _reset_all(self):
        for i in range(4):
            self._reset_bus(i)

    def _send_cmd(self, s: str):
        try:
            self.send_q.put_nowait(s)
        except queue.Full:
            pass

    def _autoscale_view(self, idx, arr):
        if arr.size == 0:
            return
        mins, maxs = arr.min(axis=0), arr.max(axis=0)
        center = (mins + maxs) / 2.0
        span = maxs - mins
        extent = float(np.max(span))
        if extent <= 0:
            extent = 1.0
        padding = max(20.0, 0.15 * extent)
        dist = (extent + padding) * 2.0

        state = getattr(self, "_cam_state", {})
        prev = state.get(idx, {"center": center, "dist": dist})
        alpha = 0.25
        new_center = prev["center"] * (1 - alpha) + center * alpha
        new_dist = prev["dist"] * (1 - alpha) + dist * alpha

        self.views[idx].opts["center"] = QVector3D(
            float(new_center[0]), float(new_center[1]), float(new_center[2])
        )
        self.views[idx].setCameraPosition(distance=float(new_dist))

        # retune grid spacing roughly to extent/10
        try:
            for item in self.views[idx].items:
                if isinstance(item, gl.GLGridItem):
                    s = max(5.0, extent / 10.0)
                    item.resetTransform()
                    item.scale(s, s, 1.0)
                    break
        except Exception:
            pass

        state[idx] = {"center": new_center, "dist": new_dist}
        self._cam_state = state

    def _log(self, text):
        self.log.appendPlainText(text)

    # ---------- main GUI tick ----------
    def _tick(self):
        # 1) Drain messages from SerialReader
        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                break

            if kind == "XYZ":
                bus, x, y, z = payload
                if 0 <= bus < 4 and not self.paused:
                    self.buffers[bus].append((x, y, z))

            elif kind == "INFO":
                self._log(payload)

            elif kind == "ERR":
                self._log(payload)

            elif kind == "RAW":
                # Show the raw input only if checkbox is enabled
                if self.showraw_cb.isChecked():
                    self._log(payload.rstrip("\n"))
                low = payload.lower()
                if "ack start" in low:
                    self.armed = True
                elif "ack stop" in low:
                    self.armed = False

        # 2) Update plots
        if not self.paused:
            for i in range(4):
                if self.buffers[i]:
                    arr = np.asarray(self.buffers[i], dtype=np.float32)
                    self.scatters[i].setData(pos=arr, color=COLORS[i], size=3.5)
                    if self.autoscale_cb.isChecked():
                        self._autoscale_view(i, arr)

        # 3) Pull one fresh batch if armed
        if (self.reader is not None) and (not self.paused) and self.armed:
            self._send_cmd("R")  # single-letter 'R' is fine; firmware accepts 'REQ' too

    # ---------- window close ----------
    def closeEvent(self, event):
        try:
            self._disconnect()
        except:
            pass
        event.accept()


# -------------------- Entrypoint --------------------
def main():
    parser = argparse.ArgumentParser(description="Live Viewer (MAG3110 x4)")
    parser.add_argument("--port", help="Serial port (optional)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    v = Viewer()
    if args.port:
        idx = v.port_cb.findText(args.port)
        if idx >= 0:
            v.port_cb.setCurrentIndex(idx)
    v.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
