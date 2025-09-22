from PyQt6.QtWidgets import *
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt, QProcess, QTimer, QCommandLineOption, QCommandLineParser

#Fix the global variable issue in the buttons class, then add the exit window as well.

class mainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        central_widget = QWidget(self)  # The main content widget
        self.setCentralWidget(central_widget)
        self.layout = QGridLayout(central_widget)

        self.setWindowTitle("24-25 UDMRT")
        self.show()
