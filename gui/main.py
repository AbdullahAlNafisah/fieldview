from __future__ import annotations
import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from main_window import MainWindow
from main_widget import Widget

if __name__ == "__main__":
    app = QApplication(sys.argv)

    widget = Widget()
    window = MainWindow(widget)

    window.show()
    QTimer.singleShot(0, window.showMaximized)

    sys.exit(app.exec())
