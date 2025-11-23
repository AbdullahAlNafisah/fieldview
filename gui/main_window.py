from __future__ import annotations
from PySide6.QtWidgets import QMainWindow


class MainWindow(QMainWindow):
    def __init__(self, widget):
        super().__init__()
        self.setWindowTitle("Sensors Viewer (XY, XZ, YZ)")
        self._central_widget = widget
        self.setCentralWidget(widget)

        self.statusBar().showMessage("Status bar ready")

    def closeEvent(self, event):
        # Make sure to stop the worker thread when closing
        if hasattr(self._central_widget, "stop_worker"):
            self._central_widget.stop_worker()
        super().closeEvent(event)
