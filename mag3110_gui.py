"""
MAG3110x4 using PySide6 & VisPy
"""

# ---------------- Environment (GPU / Qt platform) ----------------
import os

os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
os.environ["QT_QPA_PLATFORM"] = "xcb"

# ---------------- Stdlib / third-party imports ----------------
import numpy as np
from scipy.optimize import least_squares
import sys, time, csv, re, threading
import serial, serial.tools.list_ports

# ---------------- Environment (GPU / Qt platform) ----------------
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QPushButton,
    QHBoxLayout,
    QVBoxLayout,
    QComboBox,
    QGroupBox,
    QLineEdit,
    QGridLayout,
    QSpinBox,
    QLabel,
    QCheckBox,
    QPlainTextEdit,
    QFileDialog,
)
from PySide6.QtCore import (
    Qt,
    Signal,
    QObject,
    QTimer,
    QThread,
)
from PySide6 import QtGui

# ---------------- Stdlib / third-party imports ----------------
from vispy import scene
from vispy.scene.visuals import Arrow, GridLines
from vispy.scene.cameras import TurntableCamera


# ======================================================================
#                             Main Application
# ======================================================================
class MainWindow(QMainWindow):
    """
    Main window wiring: serial worker, controls, VisPy canvas and log.
    """

    request_open = Signal(str, int)
    request_close = Signal()
    request_send = Signal(str)

    # Precompiled patterns for fast line parsing
    xyz_re = re.compile(
        r"^XYZ,(?P<ts>\d+),Bus(?P<bus>\d),X=(?P<x>-?\d+)\s+Y=(?P<y>-?\d+)\s+Z=(?P<z>-?\d+)\s+T=(?P<t>-?\d+)"
    )
    info_re = re.compile(
        r"^INFO:\s+Bus(?P<bus>\d).*WHO=0x(?P<who>[0-9A-F]{2}).*SYSMOD=(?P<sys>-?\d+).*CTRL1=0x(?P<c1>[0-9A-F]{2}).*CTRL2=0x(?P<c2>[0-9A-F]{2}).*TEMP=(?P<tmp>-?\d+)"
    )

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Magnetic Field Viewer ( MAG3110 Sensor x4 )")
        self.resize(1000, 800)

        # ---- Serial worker on its own thread
        self.worker = SerialWorker()
        self.worker_thread = QThread(self)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.start()
        self.worker.line_received.connect(self.on_line)
        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)

        self.request_open.connect(self.worker.open, Qt.ConnectionType.QueuedConnection)
        self.request_close.connect(
            self.worker.close, Qt.ConnectionType.QueuedConnection
        )
        self.request_send.connect(self.worker.send, Qt.ConnectionType.QueuedConnection)

        # ---- UI
        self._build_ui()

        # ---- State
        self.port: str | None = None
        self.latest: dict[int, dict[str, object]] = {
            i: {"xyz": np.zeros(3), "temp": 0, "ok": False} for i in range(4)
        }
        self.cal_points: dict[int, list[np.ndarray]] = {i: [] for i in range(4)}
        self.bias: dict[int, np.ndarray] = {i: np.zeros(3) for i in range(4)}
        self.apply_cal: bool = True
        self._cal_collecting: bool = False

        # ---- 3D refresh timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_3d)
        self.timer.start(30)

        # ---- Auto-select a port if possible
        self.refresh_ports()

    # ------------------------------------------------------------------
    # UI building
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)

        # Left control panel
        ctl = QVBoxLayout()
        layout.addLayout(ctl, 0)

        # ---- Port controls
        self.port_combo = QComboBox()
        self.btn_refresh = QPushButton("Refresh Ports")
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setEnabled(False)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect.clicked.connect(self.do_connect)
        self.btn_disconnect.clicked.connect(self.do_disconnect)

        box_port = QGroupBox("Serial")
        layp = QVBoxLayout(box_port)
        layp.addWidget(self.port_combo)
        layp.addWidget(self.btn_refresh)
        layp.addWidget(self.btn_connect)
        layp.addWidget(self.btn_disconnect)
        ctl.addWidget(box_port)

        # ---- Actions
        self.btn_hello = QPushButton("HELLO / INFO")
        self.btn_scan = QPushButton("SCAN")
        self.btn_init_all = QPushButton("INIT ALL")
        self.btn_active_all = QPushButton("ACTIVE ALL")
        self.btn_standby_all = QPushButton("STANDBY ALL")
        for b, cmd in [
            (self.btn_hello, "HELLO\nINFO"),
            (self.btn_scan, "SCAN\nINFO"),
            (self.btn_init_all, "INIT ALL"),
            (self.btn_active_all, "ACTIVE ALL"),
            (self.btn_standby_all, "STANDBY ALL"),
        ]:
            b.clicked.connect(lambda _=None, c=cmd: self.send(c))
        box_actions = QGroupBox("Actions")
        laya = QVBoxLayout(box_actions)
        laya.addWidget(self.btn_hello)
        laya.addWidget(self.btn_scan)
        laya.addWidget(self.btn_init_all)
        laya.addWidget(self.btn_active_all)
        laya.addWidget(self.btn_standby_all)
        ctl.addWidget(box_actions)

        # ---- Direct Register R/W
        box_rw = QGroupBox("Direct Register R/W")
        layrw = QGridLayout(box_rw)
        self.spin_bus = QSpinBox()
        self.spin_bus.setRange(0, 3)
        self.edit_reg = QLineEdit("0x10")
        self.edit_val = QLineEdit("0x01")
        self.edit_len = QLineEdit("6")
        self.btn_wr = QPushButton("WR")
        self.btn_rd = QPushButton("RD")
        self.btn_wr.clicked.connect(self.do_wr)
        self.btn_rd.clicked.connect(self.do_rd)
        r = 0
        layrw.addWidget(QLabel("Bus"), r, 0)
        layrw.addWidget(self.spin_bus, r, 1)
        r += 1
        layrw.addWidget(QLabel("Reg (hex)"), r, 0)
        layrw.addWidget(self.edit_reg, r, 1)
        r += 1
        layrw.addWidget(QLabel("Val (hex)"), r, 0)
        layrw.addWidget(self.edit_val, r, 1)
        r += 1
        layrw.addWidget(self.btn_wr, r, 0)
        layrw.addWidget(self.btn_rd, r, 1)
        r += 1
        layrw.addWidget(QLabel("Len"), r, 0)
        layrw.addWidget(self.edit_len, r, 1)
        r += 1
        ctl.addWidget(box_rw)

        # ---- Stream
        box_stream = QGroupBox("Stream")
        lays = QHBoxLayout(box_stream)
        self.edit_mask = QLineEdit("0xF")
        self.btn_stream_start = QPushButton("Start")
        self.btn_stream_stop = QPushButton("Stop")
        self.btn_once = QPushButton("READXYZ ALL")
        self.btn_stream_start.clicked.connect(
            lambda: self.send(f"STREAM START {self.edit_mask.text()}")
        )
        self.btn_stream_stop.clicked.connect(lambda: self.send("STREAM STOP"))
        self.btn_once.clicked.connect(lambda: self.send("READXYZ ALL"))
        lays.addWidget(QLabel("Mask:"))
        lays.addWidget(self.edit_mask)
        lays.addWidget(self.btn_stream_start)
        lays.addWidget(self.btn_stream_stop)
        lays.addWidget(self.btn_once)
        ctl.addWidget(box_stream)

        # ---- Calibration
        box_cal = QGroupBox("Calibration (host-side bias)")
        layc = QVBoxLayout(box_cal)
        self.chk_apply = QCheckBox("Apply calibration to live data")
        self.chk_apply.setChecked(True)
        self.chk_apply.stateChanged.connect(
            lambda s: setattr(self, "apply_cal", bool(s))
        )
        hl = QHBoxLayout()
        self.btn_cal_start = QPushButton("CAL START 0xF")
        self.btn_cal_stop = QPushButton("CAL STOP + Fit")
        self.btn_cal_clear = QPushButton("Clear Bias")
        self.btn_cal_start.clicked.connect(lambda: self.send("CAL START 0xF"))
        self.btn_cal_stop.clicked.connect(self.finish_calibration)
        self.btn_cal_clear.clicked.connect(self.clear_cal)
        hl.addWidget(self.btn_cal_start)
        hl.addWidget(self.btn_cal_stop)
        hl.addWidget(self.btn_cal_clear)
        layc.addWidget(self.chk_apply)
        layc.addLayout(hl)
        ctl.addWidget(box_cal)

        # ---- Save
        box_save = QGroupBox("Save")
        laysv = QHBoxLayout(box_save)
        self.btn_save = QPushButton("Save CSV (last 10k)")
        self.btn_save.clicked.connect(self.save_csv)
        laysv.addWidget(self.btn_save)
        ctl.addWidget(box_save)
        ctl.addStretch(1)

        # Right: VisPy canvas + log
        right = QVBoxLayout()
        layout.addLayout(right, 1)

        self.canvas = VectorView(self)
        self.wid = self.canvas.native
        right.addWidget(self.wid, 1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        f = self.log.font()
        f.setFamily("DejaVu Sans Mono")
        self.log.setFont(f)
        right.addWidget(self.log, 1)

        # Data ring buffer (ts, bus, x, y, z, temp)
        self.rows: list[tuple[int, int, float, float, float, int]] = []

    # ------------------------------------------------------------------
    # Serial / port handling
    # ------------------------------------------------------------------
    def refresh_ports(self) -> None:
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.port_combo.addItem(p.device)
        if ports:
            self.port_combo.setCurrentIndex(0)

    def do_connect(self) -> None:
        if self.port:
            return
        p = self.port_combo.currentText().strip()
        if not p:
            return
        try:
            # Invoke worker.open() on its thread
            self.request_open.emit(p, 115200)

        except Exception as e:
            self.append_log(f"ERR connect: {e}")

    def do_disconnect(self) -> None:
        if not self.port:
            return
        self.request_close.emit()

    def on_connected(self, port: str) -> None:
        self.port = port
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.append_log(f"Connected to {port}")

    def on_disconnected(self) -> None:
        self.port = None
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.append_log("Disconnected")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def send(self, text: str) -> None:
        if not self.port:
            return
        self.request_send.emit(text)

    def append_log(self, s: str) -> None:
        self.log.appendPlainText(s)

    # ------------------------------------------------------------------
    # Direct R/W
    # ------------------------------------------------------------------
    def do_wr(self) -> None:
        bus = self.spin_bus.value()
        reg = self.edit_reg.text().strip()
        val = self.edit_val.text().strip()
        self.send(f"WR {bus} {reg} {val}")

    def do_rd(self) -> None:
        bus = self.spin_bus.value()
        reg = self.edit_reg.text().strip()
        ln = self.edit_len.text().strip()
        self.send(f"RD {bus} {reg} {ln}")

    # ------------------------------------------------------------------
    # RX parsing
    # ------------------------------------------------------------------
    def on_line(self, line: str) -> None:
        if not line:
            return
        self.append_log(line)

        m = self.xyz_re.match(line)
        if m:
            ts = int(m.group("ts"))
            bus = int(m.group("bus"))
            x = int(m.group("x"))
            y = int(m.group("y"))
            z = int(m.group("z"))
            t = int(m.group("t"))
            v = np.array([x, y, z], dtype=float)
            if self.apply_cal:
                v = v - self.bias.get(bus, np.zeros(3))
            self.latest[bus]["xyz"] = v
            self.latest[bus]["temp"] = t
            self.latest[bus]["ok"] = True
            self.rows.append((ts, bus, float(v[0]), float(v[1]), float(v[2]), t))
            if len(self.rows) > 10000:
                self.rows = self.rows[-10000:]

            # Collect calibration points if enabled (piggyback on stream)
            if self._cal_collecting:
                self.cal_points[bus].append(v.copy())
            return

        m2 = self.info_re.match(line)
        if m2:
            # Example parsing kept; extend as needed
            bus = int(m2.group("bus"))
            who = int(m2.group("who"), 16)
            _ = (bus, who)  # placeholders to avoid linter warnings
            return

        if "READY" in line:
            return

        if line.startswith("ACK CAL START"):
            self._cal_collecting = True
        elif line.startswith("ACK CAL STOP"):
            self._cal_collecting = False

    # ------------------------------------------------------------------
    # Calibration actions
    # ------------------------------------------------------------------
    def finish_calibration(self) -> None:
        """Fit a sphere for each bus and set the bias (center)."""
        for b in range(4):
            P = (
                np.array(self.cal_points[b], dtype=float)
                if self.cal_points[b]
                else np.zeros((0, 3))
            )
            if len(P) < 100:
                self.append_log(
                    f"Bus{b}: not enough points for calibration; need ~100+"
                )
                continue
            c, r = fit_sphere(P)
            self.bias[b] = c
            self.append_log(f"Bus{b}: bias = {c.round(2).tolist()}, radius ~ {r:.2f}")
        self._cal_collecting = False
        self.send("CAL STOP")

    def clear_cal(self) -> None:
        for b in range(4):
            self.bias[b] = np.zeros(3)
            self.cal_points[b] = []
        self.append_log("Calibration cleared (bias zeros)")

    # ------------------------------------------------------------------
    # Saving
    # ------------------------------------------------------------------
    def save_csv(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Save CSV", "mag3110.csv", "CSV Files (*.csv)"
        )
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["ts", "bus", "x", "y", "z", "temp"])
            w.writerows(self.rows)
        self.append_log(f"Saved {len(self.rows)} rows to {path}")

    # ------------------------------------------------------------------
    # 3D update
    # ------------------------------------------------------------------
    def update_3d(self) -> None:
        """Redraw sensor vectors and combined mean vector."""
        vecs = []
        for b in range(4):
            v = self.latest[b]["xyz"]
            pos = np.array([[0, 0, 0], v], dtype=float)
            self.canvas.vectors[b].set_data(pos=pos)
            if self.latest[b]["ok"]:
                vecs.append(v)
        mean = np.mean(np.stack(vecs, axis=0), axis=0) if vecs else np.zeros(3)
        self.canvas.combined.set_data(pos=np.array([[0, 0, 0], mean], dtype=float))

    # ------------------------------------------------------------------
    # Close handling
    # ------------------------------------------------------------------
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Ensure serial + worker thread are stopped before app exits."""
        try:
            self.request_close.emit()
            self.worker_thread.quit()
            self.worker_thread.wait(1000)
        finally:
            super().closeEvent(event)


# ======================================================================
#                               Serial I/O
# ======================================================================
class SerialWorker(QObject):
    """
    Runs in a separate thread. Opens/closes the port and emits received lines.
    """

    line_received = Signal(str)
    connected = Signal(str)
    disconnected = Signal()

    def __init__(self):
        super().__init__()
        self.ser: serial.Serial | None = None
        self._rx_thread: threading.Thread | None = None
        self._rx_stop = threading.Event()

    def open(self, port: str, baud: int = 115200) -> None:
        """Open serial port and start background RX loop."""
        self.close()
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        except Exception as e:
            self.ser = None
            self.line_received.emit(f"ERR open: {e}")
            return

        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.connected.emit(port)
        self.send("HELLO\n")
        self.send("INFO\n")

    def close(self) -> None:
        """Stop RX loop and close the port."""
        self._rx_stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.disconnected.emit()

    def send(self, text: str) -> None:
        """Send a line to the device (ensures newline)."""
        if not self.ser:
            return
        if not text.endswith("\n"):
            text += "\n"
        try:
            self.ser.write(text.encode("ascii", errors="ignore"))
        except Exception as e:
            self.line_received.emit(f"ERR send: {e}")

    # ---------------- internal ----------------
    def _rx_loop(self) -> None:
        """Collect bytes and emit complete ASCII lines."""
        buf = bytearray()
        while not self._rx_stop.is_set() and self.ser:
            try:
                data = self.ser.read(256)
                if data:
                    buf.extend(data)
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        line = line.rstrip(b"\r").decode("ascii", errors="ignore")
                        self.line_received.emit(line)
                else:
                    time.sleep(0.01)
            except Exception:
                break


# ======================================================================
#                                3D View
# ======================================================================
class VectorView(scene.SceneCanvas):
    """
    Minimal VisPy canvas with axes, up to four sensor vectors, and a combined vector.

    Important:
    - Use a real TurntableCamera instance (not the string "turntable") so type
      checking on `.fov`/`.distance` works.
    - Import visuals directly (Arrow, GridLines) for Pylance friendliness.
    """

    def __init__(self, parent=None):
        scene.SceneCanvas.__init__(self, keys=None, size=(600, 400), show=False)
        self.unfreeze()

        # View + camera
        self.view = self.central_widget.add_view()
        self.view.camera = TurntableCamera(fov=45, distance=300)

        # Grid
        grid = GridLines()
        self.view.add(grid)

        # Coordinate axes
        axis_len = 100
        self.axes: list[Arrow] = []
        for color, vec in zip(
            [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)],
            [(axis_len, 0, 0), (0, axis_len, 0), (0, 0, axis_len)],
        ):
            ln = Arrow(
                pos=np.array([[0, 0, 0], vec], dtype=float),
                arrow_size=10,
                width=3,
                color=color,
            )
            self.view.add(ln)
            self.axes.append(ln)

        # Up to 4 sensor vectors
        self.vectors: list[Arrow] = []
        self.colors = [(1, 1, 0, 1), (1, 0, 1, 1), (0, 1, 1, 1), (1, 0.5, 0, 1)]
        for i in range(4):
            ln = Arrow(
                pos=np.array([[0, 0, 0], [0, 0, 0]], dtype=float),
                arrow_size=8,
                width=3,
                color=self.colors[i],
            )
            self.view.add(ln)
            self.vectors.append(ln)

        # Combined vector
        self.combined = Arrow(
            pos=np.array([[0, 0, 0], [0, 0, 0]], dtype=float),
            arrow_size=12,
            width=5,
            color=(1, 1, 1, 1),
        )
        self.view.add(self.combined)

        self.freeze()


# ======================================================================
#                              Calibration
# ======================================================================
def fit_sphere(points: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Fit a sphere to Nx3 points by least squares.
    Returns (center[3], radius).
    """
    x0 = np.zeros(4)  # cx, cy, cz, r

    def resid(p: np.ndarray) -> np.ndarray:
        c = p[:3]
        r = p[3]
        return np.linalg.norm(points - c, axis=1) - r

    res = least_squares(resid, x0, method="lm")
    c = res.x[:3]
    r = abs(res.x[3])
    return c, float(r)


# ======================================================================
#                                  main
# ======================================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
