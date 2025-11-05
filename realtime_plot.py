# realtime_plot.py
import sys, time, threading
from dataclasses import dataclass

from PySide6.QtCore import Qt, QObject, Signal, Slot, QThread, QTimer
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QSpinBox, QMessageBox
)
import pyqtgraph as pg
import numpy as np
import serial
import serial.tools.list_ports

HEADER = "# BMM350 CSV"

@dataclass
class CaptureParams:
    port: str
    baud: int
    maxpts: int
    header_timeout_s: float = 8.0
    read_timeout_s: float = 1.0
    batch_emit: int = 20       # emit UI updates every N samples for smoothness

class SerialWorker(QObject):
    progress = Signal(int)                # samples collected
    data_arrived = Signal(np.ndarray, np.ndarray, np.ndarray)  # x,y,z chunk
    finished = Signal(int)                # final count
    error = Signal(str)

    def __init__(self, params: CaptureParams):
        super().__init__()
        self.params = params
        self._stop_evt = threading.Event()

    @Slot()
    def run(self):
        p = self.params
        try:
            ser = serial.Serial(p.port, p.baud, timeout=p.read_timeout_s)
        except Exception as e:
            self.error.emit(f"Failed to open {p.port}: {e}")
            self.finished.emit(0)
            return

        try:
            # Wait for header
            start = time.time()
            seen_header = False
            while time.time() - start < p.header_timeout_s and not self._stop_evt.is_set():
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith(HEADER):
                    seen_header = True
                    break
            if not seen_header:
                self.error.emit("HEADER not seen. Wrong sketch / wrong COM?")
                ser.close()
                self.finished.emit(0)
                return

            remaining = p.maxpts
            xs, ys, zs = [], [], []
            emitted = 0

            # Read loop
            while remaining > 0 and not self._stop_evt.is_set():
                s = ser.readline().decode("utf-8", errors="ignore").strip()
                if not s or s.startswith("#") or s == "DONE":
                    continue
                parts = s.split(",")
                if len(parts) != 3:
                    continue
                try:
                    x, y, z = map(float, parts)
                except Exception:
                    continue
                xs.append(x); ys.append(y); zs.append(z)
                remaining -= 1

                # Emit chunk updates
                if len(xs) - emitted >= p.batch_emit or remaining == 0:
                    chunk_x = np.array(xs[emitted:], dtype=float)
                    chunk_y = np.array(ys[emitted:], dtype=float)
                    chunk_z = np.array(zs[emitted:], dtype=float)
                    self.data_arrived.emit(chunk_x, chunk_y, chunk_z)
                    emitted = len(xs)
                    self.progress.emit(len(xs))

            count = p.maxpts - remaining
            ser.close()
            self.finished.emit(count)
        except Exception as e:
            try:
                ser.close()
            except Exception:
                pass
            self.error.emit(str(e))
            self.finished.emit(0)

    def stop(self):
        self._stop_evt.set()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BMM350 Realtime Capture (PySide6 + pyqtgraph)")
        self.resize(1000, 650)

        # --- Controls ---
        self.port_combo = QComboBox()
        self.refresh_ports()

        self.baud_combo = QComboBox()
        for b in [115200, 230400, 460800, 921600]:
            self.baud_combo.addItem(str(b))
        self.baud_combo.setCurrentText("115200")

        self.maxpts_spin = QSpinBox()
        self.maxpts_spin.setRange(10, 1_000_000)
        self.maxpts_spin.setValue(1000)

        self.btn_refresh = QPushButton("Refresh Ports")
        self.btn_start = QPushButton("▶ Capture 1000 pts")
        self.btn_stop = QPushButton("■ Stop")
        self.btn_stop.setEnabled(False)

        top = QHBoxLayout()
        top.addWidget(QLabel("Port:"))
        top.addWidget(self.port_combo, 1)
        top.addWidget(self.btn_refresh)
        top.addSpacing(16)
        top.addWidget(QLabel("Baud:"))
        top.addWidget(self.baud_combo)
        top.addSpacing(16)
        top.addWidget(QLabel("Points:"))
        top.addWidget(self.maxpts_spin)
        top.addStretch(1)
        top.addWidget(self.btn_start)
        top.addWidget(self.btn_stop)

        # --- Plot ---
        self.plot = pg.PlotWidget()
        self.plot.setBackground("w")
        self.plot.addLegend()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve_x = self.plot.plot([], [], name="X", pen=pg.mkPen(width=2))
        self.curve_y = self.plot.plot([], [], name="Y", pen=pg.mkPen(width=2))
        self.curve_z = self.plot.plot([], [], name="Z", pen=pg.mkPen(width=2))
        self.plot.setLabel("bottom", "Sample Index")
        self.plot.setLabel("left", "Value")

        # --- Status ---
        self.status_lbl = QLabel("Idle.")
        self.statusBar().addWidget(self.status_lbl)

        root = QVBoxLayout()
        root.addLayout(top)
        root.addWidget(self.plot, 1)
        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)

        # Buffers (preallocate for speed)
        self._maxpts = self.maxpts_spin.value()
        self._ix = 0
        self.buf_x = np.zeros(self._maxpts, dtype=float)
        self.buf_y = np.zeros(self._maxpts, dtype=float)
        self.buf_z = np.zeros(self._maxpts, dtype=float)

        # Connections
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.maxpts_spin.valueChanged.connect(self.on_pts_changed)

        # Thread/worker placeholders
        self.thread: QThread | None = None
        self.worker: SerialWorker | None = None

        # Keep plot autoscaling sensible during capture
        self._autoscale_timer = QTimer(self)
        self._autoscale_timer.setInterval(250)
        self._autoscale_timer.timeout.connect(self.autoscale_view)

    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = ["COM8"]  # fallback default as in your snippet
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def on_pts_changed(self, n):
        self._maxpts = n
        self.buf_x = np.zeros(self._maxpts, dtype=float)
        self.buf_y = np.zeros(self._maxpts, dtype=float)
        self.buf_z = np.zeros(self._maxpts, dtype=float)
        self._ix = 0
        self.update_plot()

    def on_start(self):
        if self.thread is not None:
            return
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "No Port", "Select a COM port first.")
            return
        baud = int(self.baud_combo.currentText())
        maxpts = int(self.maxpts_spin.value())

        # Reset buffers and UI
        self.buf_x.fill(0); self.buf_y.fill(0); self.buf_z.fill(0)
        self._ix = 0
        self.update_plot()
        self.status_lbl.setText(f"Opening {port} @ {baud}…")
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        params = CaptureParams(
            port=port, baud=baud, maxpts=maxpts,
            header_timeout_s=8.0, read_timeout_s=1.0, batch_emit=20
        )
        self.thread = QThread(self)
        self.worker = SerialWorker(params)
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.data_arrived.connect(self.on_data)
        self.worker.progress.connect(self.on_progress)
        self.worker.error.connect(self.on_error)
        self.worker.finished.connect(self.on_finished)

        # Proper cleanup
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)

        self.thread.start()
        self._autoscale_timer.start()

    @Slot(np.ndarray, np.ndarray, np.ndarray)
    def on_data(self, chunk_x, chunk_y, chunk_z):
        # Write chunk into preallocated buffers
        n = len(chunk_x)
        end = min(self._ix + n, self._maxpts)
        slice_len = end - self._ix
        if slice_len <= 0:
            return
        self.buf_x[self._ix:end] = chunk_x[:slice_len]
        self.buf_y[self._ix:end] = chunk_y[:slice_len]
        self.buf_z[self._ix:end] = chunk_z[:slice_len]
        self._ix = end
        self.update_plot()

    @Slot(int)
    def on_progress(self, count):
        self.status_lbl.setText(f"Collecting… {count}/{self._maxpts}")

    @Slot(str)
    def on_error(self, msg):
        QMessageBox.critical(self, "Serial Error", msg)

    @Slot(int)
    def on_finished(self, count):
        self.status_lbl.setText(f"Done. Collected {count} points.")
        self._autoscale_timer.stop()
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        # Tear down thread safely
        if self.thread:
            self.thread.quit()
            self.thread.wait(2000)
        self.thread = None
        self.worker = None

    def autoscale_view(self):
        # Autoscale y; keep x from 0..current (or maxpts if full)
        xmax = max(self._ix, 100)
        self.plot.setXRange(0, xmax, padding=0)
        self.plot.enableAutoRange(axis='y', enable=True)

    def on_stop(self):
        if self.worker:
            self.worker.stop()
        self.status_lbl.setText("Stopping…")
        self.btn_stop.setEnabled(False)

    def update_plot(self):
        xs = np.arange(self._ix, dtype=int)
        self.curve_x.setData(xs, self.buf_x[:self._ix])
        self.curve_y.setData(xs, self.buf_y[:self._ix])
        self.curve_z.setData(xs, self.buf_z[:self._ix])

def main():
    # HiDPI niceties
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
