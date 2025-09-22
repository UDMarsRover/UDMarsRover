import sys
import os
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox, QCheckBox, QLabel
from drive.MotorController import MotorController
from PyQt6.QtCore import Qt

class RoverControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.mc = None
        self.port = "/dev/cu.usbmodem101"
        self.setupMotorController()

    def initUI(self):
        self.setWindowTitle('Rover Control')
        layout = QVBoxLayout()

        self.button = QPushButton('Toggle LED', self)
        self.button.clicked.connect(self.toggle_led)
        layout.addWidget(self.button)

        self.status_button = QPushButton('Get LED Status', self)
        self.status_button.clicked.connect(self.get_led_status)
        layout.addWidget(self.status_button)

        self.led_toggle = QCheckBox('LED Switch', self)
        self.led_toggle.stateChanged.connect(self.set_led)
        layout.addWidget(self.led_toggle)

        self.led_status_bar = QLabel(self)
        self.led_status_bar.setStyleSheet("background-color: grey;")
        layout.addWidget(self.led_status_bar)

        self.setLayout(layout)
        self.show()

    def setupMotorController(self):
        if not os.path.exists(self.port):
            QMessageBox.critical(self, "Error", f"The port {self.port} does not exist.")
        else:
            self.mc = MotorController(port=self.port)
            self.mc.connect()

    def toggle_led(self):
        if self.mc:
            self.mc.toggle_led()
        else:
            QMessageBox.critical(self, "Error", "MotorController is not connected.")

    def set_led(self, state):
        if self.mc:
            self.mc.set_led(state > 0)
        else:
            QMessageBox.critical(self, "Error", "MotorController is not connected.")

    def get_led_status(self):
        if self.mc:
            led_is_on = self.mc.get_led_status()
            self.update_led_status_bar(led_is_on)
        else:
            QMessageBox.critical(self, "Error", "MotorController is not connected.")

    def update_led_status_bar(self, led_is_on):
        if led_is_on:
            self.led_status_bar.setStyleSheet("background-color: red;")
        else:
            self.led_status_bar.setStyleSheet("background-color: grey;")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RoverControlApp()
    sys.exit(app.exec())