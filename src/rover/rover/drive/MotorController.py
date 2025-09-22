from serial import SerialException
from UDMRTMotorSerial import UDMRTMotorSerial
import time
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QSlider, QTableWidget, QTableWidgetItem
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
import sys

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Sample GUI")

        # Create the main layout
        layout = QVBoxLayout()

        # Create a slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(300)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.on_slider_value_changed)
        layout.addWidget(self.slider)

        # Create a table
        self.table = QTableWidget(5, 3)  # 5 rows, 3 columns
        self.table.setHorizontalHeaderLabels(["Column 1", "Column 2", "Column 3"])
        for i in range(5):
            for j in range(3):
                self.table.setItem(i, j, QTableWidgetItem(f"Item {i+1},{j+1}"))
        layout.addWidget(self.table)
        self.mc = MotorController()
        self.mc.data_received.connect(self.update_table)
        # self.mc.start_listening()
        # Set the central widget
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Create a QTimer to limit the refresh rate
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_motor_controller)
        self.timer.start(100)  # 100ms interval for 10Hz
        self.mc.start_listening()

        self.slider_value = 0

    def on_slider_value_changed(self, value):
        self.slider_value = value

    def update_motor_controller(self):
        # parsed_data = self.mc.sniff()
        # try: self.update_table(parsed_data)
        # except: pass
        velocities = [self.slider_value] + [0]*5
        self.mc.serial_conn.send_velocity_set(velocities)

    def update_table(self, data):
        for i, (temperature, voltage, current) in enumerate(data):
            self.table.setItem(i, 0, QTableWidgetItem(f"{temperature:.1f}"))
            self.table.setItem(i, 1, QTableWidgetItem(f"{voltage:.1f}"))
            self.table.setItem(i, 2, QTableWidgetItem(f"{current:.1f}"))

class MotorController(QObject):
    data_received = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.serial_conn = UDMRTMotorSerial(port="/dev/cu.usbmodem2101", baudrate=115200)
        if not self.serial_conn.connect():
            raise SerialException("Could not connect to motor controller")
        self.running = True

    def sniff(self):
        self.serial_conn.spin_once()
        parsed_data = self.serial_conn.spin_once()
        return parsed_data

    def listen(self):
        while self.running:
            parsed_data = self.serial_conn.spin_once()
            self.data_received.emit(parsed_data)
            # for i, (temperature, voltage, current) in enumerate(parsed_data):
            #     print(f"Motor {i+1} - temperature: {temperature:.1f}, voltage: {voltage:.1f}, current: {current:.1f}")
            time.sleep(0.1)

    def get_input(self):
        while self.running:
            command = input("Enter command: ")
            if command == "brake":
                self.serial_conn.send_brake()
            elif command == "idle":
                self.serial_conn.send_idle()
            elif command == "exit":
                self.running = False
            else:
                try:
                    velocities = [float(vel) for vel in command.split()]
                    self.serial_conn.send_velocity_set(velocities)
                except ValueError:
                    print("Invalid command")
            time.sleep(0.1)

    def start_listening(self):
        self.listener_thread = threading.Thread(target=self.listen)
        self.listener_thread.start()

    def loop_input(self):
        self.input_thread = threading.Thread(target=self.get_input)
        self.input_thread.start()

    def stop_listening(self):
        self.running = False
        self.listener_thread.join()

    def kill(self):
        self.serial_conn.send_velocity_set([0]*6)
        self.running = False
        self.listener_thread.join()
        self.input_thread.join()
        self.serial_conn.close()

def main():
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec())
    except KeyboardInterrupt:
        window.mc.kill()
        sys.exit(0)
    # motor_controller = MotorController()
    # motor_controller.start_listening()
    # motor_controller.loop_input()
    # try:
    #     while True:
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     motor_controller.stop_listening()


if __name__ == '__main__':
    main()