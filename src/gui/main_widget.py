from __future__ import annotations
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QCheckBox,
    QLabel,
    QMessageBox,
    QSpinBox,
    QFormLayout,
)
from PySide6.QtCharts import QChartView, QChart, QScatterSeries, QValueAxis
from serial_worker import SerialWorker

# ---------------------------- Configuration ---------------------------------
# e.g. "COM5" on Windows, "/dev/ttyACM0" on Linux
SERIAL_PORT = "COM5"
BAUD_RATE = 115200
NUM_SENSORS = 6


class Widget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Storage for raw 3D points per sensor (for calibration)
        self.raw_points: list[list[tuple[float, float, float]]] = [
            [] for _ in range(NUM_SENSORS)
        ]

        # Calibration parameters per sensor: {"offset": (ox, oy, oz), "scale": (sx, sy, sz)} or None
        self.calibration_params: list[dict | None] = [None] * NUM_SENSORS

        # --------------------- Controls (left column) -----------------------
        controls_layout = QVBoxLayout()

        self.status_label = QLabel("Status: Not collecting")
        controls_layout.addWidget(self.status_label)

        # Start/Stop button
        self.start_button = QPushButton("Start Collecting Data")
        self.start_button.setCheckable(True)
        self.start_button.toggled.connect(self.on_start_collecting)
        controls_layout.addWidget(self.start_button)

        # Buffer size option
        form_layout = QFormLayout()
        self.buffer_spin = QSpinBox()
        self.buffer_spin.setRange(1, 5000)  # min/max points
        self.buffer_spin.setValue(1000)  # default
        form_layout.addRow("Buffer size (points):", self.buffer_spin)
        controls_layout.addLayout(form_layout)

        controls_layout.addWidget(QLabel("Sensors:"))

        self.sensor_checkboxes: list[QCheckBox] = []
        for i in range(NUM_SENSORS):
            cb = QCheckBox(f"Sensor {i}")
            cb.setChecked(True)  # enabled by default
            self.sensor_checkboxes.append(cb)
            controls_layout.addWidget(cb)

        # Buttons
        self.clear_button = QPushButton("Clear Data")
        self.clear_button.clicked.connect(self.on_clear_data)
        controls_layout.addWidget(self.clear_button)

        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.on_calibrate)
        controls_layout.addWidget(self.calibrate_button)

        controls_layout.addStretch(1)

        # --------------------- Charts: Original (center) --------------------
        original_layout = QVBoxLayout()

        (
            self.chart_xy,
            self.chartview_xy,
            self.series_xy,
            self.axis_xy_x,
            self.axis_xy_y,
        ) = self._create_pair_chart("XY plane", "X", "Y")

        (
            self.chart_xz,
            self.chartview_xz,
            self.series_xz,
            self.axis_xz_x,
            self.axis_xz_y,
        ) = self._create_pair_chart("XZ plane", "X", "Z")

        (
            self.chart_yz,
            self.chartview_yz,
            self.series_yz,
            self.axis_yz_x,
            self.axis_yz_y,
        ) = self._create_pair_chart("YZ plane", "Y", "Z")

        original_layout.addWidget(self.chartview_xy)
        original_layout.addWidget(self.chartview_xz)
        original_layout.addWidget(self.chartview_yz)

        # --------------------- Charts: Calibrated (right) ------------------
        calibrated_layout = QVBoxLayout()

        (
            self.chart_xy_cal,
            self.chartview_xy_cal,
            self.series_xy_cal,
            self.axis_xy_x_cal,
            self.axis_xy_y_cal,
        ) = self._create_pair_chart("XY plane (Calibrated)", "X", "Y")

        (
            self.chart_xz_cal,
            self.chartview_xz_cal,
            self.series_xz_cal,
            self.axis_xz_x_cal,
            self.axis_xz_y_cal,
        ) = self._create_pair_chart("XZ plane (Calibrated)", "X", "Z")

        (
            self.chart_yz_cal,
            self.chartview_yz_cal,
            self.series_yz_cal,
            self.axis_yz_x_cal,
            self.axis_yz_y_cal,
        ) = self._create_pair_chart("YZ plane (Calibrated)", "Y", "Z")

        calibrated_layout.addWidget(self.chartview_xy_cal)
        calibrated_layout.addWidget(self.chartview_xz_cal)
        calibrated_layout.addWidget(self.chartview_yz_cal)

        # --------------------- Main Layout (3 columns) ---------------------
        main_layout = QHBoxLayout(self)
        main_layout.addLayout(controls_layout, 1)  # left: controls
        main_layout.addLayout(original_layout, 3)  # center: original
        main_layout.addLayout(calibrated_layout, 3)  # right: calibrated

        # --------------------- Serial Worker (lazy start) ------------------
        self.worker: SerialWorker | None = None

    # ----------------------------------------------------------------------
    # Chart creation helper
    # ----------------------------------------------------------------------
    def _create_pair_chart(self, title: str, x_label: str, y_label: str):
        chart = QChart()
        chart.setTitle(title)
        chart.legend().setVisible(True)

        # One series per sensor
        series_list: list[QScatterSeries] = []
        for idx in range(NUM_SENSORS):
            s = QScatterSeries()
            s.setName(f"S{idx}")
            s.setMarkerSize(5.0)
            chart.addSeries(s)
            series_list.append(s)

        # Axes; initial range is just a placeholder and will be updated dynamically
        axis_x = QValueAxis()
        axis_y = QValueAxis()
        axis_x.setTitleText(x_label)
        axis_y.setTitleText(y_label)
        axis_x.setRange(-10, 10)
        axis_y.setRange(-10, 10)

        chart.addAxis(axis_x, Qt.AlignBottom)
        chart.addAxis(axis_y, Qt.AlignLeft)

        for s in series_list:
            s.attachAxis(axis_x)
            s.attachAxis(axis_y)

        view = QChartView(chart)
        view.setRenderHint(QPainter.Antialiasing)

        # Return axes so we can change ranges later
        return chart, view, series_list, axis_x, axis_y

    def _update_axes_for_plane(
        self, series_list, axis_x: QValueAxis, axis_y: QValueAxis
    ):
        first_point = True
        min_x = 0.0
        max_x = 0.0
        min_y = 0.0
        max_y = 0.0

        for idx, series in enumerate(series_list):
            # Only consider checked sensors
            if not self.sensor_checkboxes[idx].isChecked():
                continue

            points = series.pointsVector()
            for p in points:
                x = p.x()
                y = p.y()
                if first_point:
                    min_x = x
                    max_x = x
                    min_y = y
                    max_y = y
                    first_point = False
                else:
                    if x < min_x:
                        min_x = x
                    if x > max_x:
                        max_x = x
                    if y < min_y:
                        min_y = y
                    if y > max_y:
                        max_y = y

        if first_point:
            # No data for checked sensors; keep a default range
            axis_x.setRange(-10, 10)
            axis_y.setRange(-10, 10)
            return

        dx = max_x - min_x
        dy = max_y - min_y
        if dx == 0.0:
            dx = 1.0
        if dy == 0.0:
            dy = 1.0

        margin_x = dx * 0.1
        margin_y = dy * 0.1

        axis_x.setRange(min_x - margin_x, max_x + margin_x)
        axis_y.setRange(min_y - margin_y, max_y + margin_y)

    def _apply_calibration(
        self, idx: int, x: float, y: float, z: float
    ) -> tuple[float, float, float]:
        """
        Apply saved calibration parameters for sensor idx to (x, y, z).
        If no calibration parameters exist, returns the input unchanged.
        """
        params = self.calibration_params[idx]
        if params is None:
            return x, y, z

        ox, oy, oz = params["offset"]
        sx, sy, sz = params["scale"]

        x_cal = (x - ox) * sx
        y_cal = (y - oy) * sy
        z_cal = (z - oz) * sz
        return x_cal, y_cal, z_cal

    # ----------------------------------------------------------------------
    # Slots - Buttons
    # ----------------------------------------------------------------------
    @Slot(bool)
    def on_start_collecting(self, checked: bool):
        if checked:
            # ON state → start worker
            if self.worker is None:
                self.worker = SerialWorker(SERIAL_PORT, BAUD_RATE, self)
                self.worker.data_received.connect(self.on_data_received)
                self.worker.error.connect(self.on_error)
                self.worker.start()

            self.status_label.setText("Status: Collecting Data")
            self.start_button.setText("Stop Collecting Data")

        else:
            # OFF state → stop worker
            if self.worker is not None:
                self.worker.stop()
                self.worker = None

            self.status_label.setText("Status: Stopped")
            self.start_button.setText("Start Collecting Data")

    @Slot()
    def on_clear_data(self):
        """Clear all data from both original and calibrated charts."""
        for series_group in (
            self.series_xy,
            self.series_xz,
            self.series_yz,
            self.series_xy_cal,
            self.series_xz_cal,
            self.series_yz_cal,
        ):
            for s in series_group:
                s.clear()

        # Reset axes to default ranges
        self.axis_xy_x.setRange(-10, 10)
        self.axis_xy_y.setRange(-10, 10)
        self.axis_xz_x.setRange(-10, 10)
        self.axis_xz_y.setRange(-10, 10)
        self.axis_yz_x.setRange(-10, 10)
        self.axis_yz_y.setRange(-10, 10)

        self.axis_xy_x_cal.setRange(-10, 10)
        self.axis_xy_y_cal.setRange(-10, 10)
        self.axis_xz_x_cal.setRange(-10, 10)
        self.axis_xz_y_cal.setRange(-10, 10)
        self.axis_yz_x_cal.setRange(-10, 10)
        self.axis_yz_y_cal.setRange(-10, 10)

        self.status_label.setText("Status: Data cleared")

    @Slot()
    def on_calibrate(self):
        """
        Calibrate button clicked.

        Uses stored raw_points (original x, y, z) to estimate
        simple hard-iron (offset) and soft-iron (scale) parameters.
        These parameters are saved and then used for:
        * regenerating calibrated charts from all stored data
        * calibrating future incoming data in on_data_received
        """
        self.status_label.setText("Status: Calibrating data")

        min_points_for_calibration = 10

        for idx in range(NUM_SENSORS):
            points = self.raw_points[idx]

            if len(points) < min_points_for_calibration:
                # Not enough data to calibrate this sensor
                self.calibration_params[idx] = None
                self.series_xy_cal[idx].clear()
                self.series_xz_cal[idx].clear()
                self.series_yz_cal[idx].clear()
                continue

            # --- Compute min/max per axis for this sensor ---
            min_x = points[0][0]
            max_x = points[0][0]
            min_y = points[0][1]
            max_y = points[0][1]
            min_z = points[0][2]
            max_z = points[0][2]

            for x, y, z in points[1:]:
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
                if z < min_z:
                    min_z = z
                if z > max_z:
                    max_z = z

            # --- Hard-iron offset: center of the ranges ---
            offset_x = (max_x + min_x) / 2.0
            offset_y = (max_y + min_y) / 2.0
            offset_z = (max_z + min_z) / 2.0

            # --- Soft-iron scale: normalize ranges to a common radius ---
            range_x = (max_x - min_x) / 2.0
            range_y = (max_y - min_y) / 2.0
            range_z = (max_z - min_z) / 2.0

            if range_x == 0.0:
                range_x = 1.0
            if range_y == 0.0:
                range_y = 1.0
            if range_z == 0.0:
                range_z = 1.0

            avg_radius = (range_x + range_y + range_z) / 3.0
            if avg_radius == 0.0:
                avg_radius = 1.0

            scale_x = avg_radius / range_x
            scale_y = avg_radius / range_y
            scale_z = avg_radius / range_z

            # Save parameters
            self.calibration_params[idx] = {
                "offset": (offset_x, offset_y, offset_z),
                "scale": (scale_x, scale_y, scale_z),
            }

            # --- Regenerate calibrated charts from all stored original data ---
            self.series_xy_cal[idx].clear()
            self.series_xz_cal[idx].clear()
            self.series_yz_cal[idx].clear()

            for x, y, z in points:
                x_cal, y_cal, z_cal = self._apply_calibration(idx, x, y, z)

                self.series_xy_cal[idx].append(x_cal, y_cal)
                self.series_xz_cal[idx].append(x_cal, z_cal)
                self.series_yz_cal[idx].append(y_cal, z_cal)

        # Update axes for calibrated charts after recalculation
        self._update_axes_for_plane(
            self.series_xy_cal, self.axis_xy_x_cal, self.axis_xy_y_cal
        )
        self._update_axes_for_plane(
            self.series_xz_cal, self.axis_xz_x_cal, self.axis_xz_y_cal
        )
        self._update_axes_for_plane(
            self.series_yz_cal, self.axis_yz_x_cal, self.axis_yz_y_cal
        )

        self.status_label.setText("Status: Calibration done")
        print("Calibration parameters updated and applied to calibrated plots.")

    # ----------------------------------------------------------------------
    # Slots - Data & Errors
    # ----------------------------------------------------------------------
    @Slot(list)
    def on_data_received(self, arr: list):
        """
        arr: list of dicts like:
            {"idx":0,"x":...,"y":...,"z":...,"temp":...}
        """
        self.status_label.setText("Status: Receiving data")

        max_points = self.buffer_spin.value()

        for item in arr:
            idx = item.get("idx")
            if idx is None or not (0 <= idx < NUM_SENSORS):
                continue

            # Skip if this sensor is disabled
            if not self.sensor_checkboxes[idx].isChecked():
                continue

            x = float(item.get("x", 0.0))
            y = float(item.get("y", 0.0))
            z = float(item.get("z", 0.0))

            # ---- Store raw 3D point for calibration ----
            self.raw_points[idx].append((x, y, z))
            if len(self.raw_points[idx]) > max_points:
                self.raw_points[idx].pop(0)

            # ---- Plot original data (center column) ----
            self.series_xy[idx].append(x, y)
            self.series_xz[idx].append(x, z)
            self.series_yz[idx].append(y, z)

            # Trim old points from original chart series
            for series in (
                self.series_xy[idx],
                self.series_xz[idx],
                self.series_yz[idx],
            ):
                if series.count() > max_points:
                    series.removePoints(0, series.count() - max_points)

            # ---- Plot calibrated data (right column) if calibration exists ----
            if self.calibration_params[idx] is not None:
                x_cal, y_cal, z_cal = self._apply_calibration(idx, x, y, z)

                self.series_xy_cal[idx].append(x_cal, y_cal)
                self.series_xz_cal[idx].append(x_cal, z_cal)
                self.series_yz_cal[idx].append(y_cal, z_cal)

                # Trim old points for calibrated series too
                for series in (
                    self.series_xy_cal[idx],
                    self.series_xz_cal[idx],
                    self.series_yz_cal[idx],
                ):
                    if series.count() > max_points:
                        series.removePoints(0, series.count() - max_points)

        # After updating all points, recompute axes based on checked sensors
        self._update_axes_for_plane(self.series_xy, self.axis_xy_x, self.axis_xy_y)
        self._update_axes_for_plane(self.series_xz, self.axis_xz_x, self.axis_xz_y)
        self._update_axes_for_plane(self.series_yz, self.axis_yz_x, self.axis_yz_y)

        self._update_axes_for_plane(
            self.series_xy_cal, self.axis_xy_x_cal, self.axis_xy_y_cal
        )
        self._update_axes_for_plane(
            self.series_xz_cal, self.axis_xz_x_cal, self.axis_xz_y_cal
        )
        self._update_axes_for_plane(
            self.series_yz_cal, self.axis_yz_x_cal, self.axis_yz_y_cal
        )

    @Slot(str)
    def on_error(self, msg: str):
        self.status_label.setText("Status: Error")
        QMessageBox.warning(self, "Serial Error", msg)

    # ----------------------------------------------------------------------
    # Cleanup
    # ----------------------------------------------------------------------
    def stop_worker(self):
        if hasattr(self, "worker") and self.worker is not None:
            self.worker.stop()
            self.worker = None
            self.status_label.setText("Status: Stopped")
