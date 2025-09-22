from pro_controller import NintendoProController
from UDMRTMotorSerial import UDMRTMotorSerial
from serial.serialutil import SerialException
class BTDrive:
    def __init__(self, serial_port='/dev/serial/by-id/usb-Adafruit_Feather_M4_CAN_CC17951D534837434E202020FF0F291F-if00'):
        self.serial_conn = UDMRTMotorSerial(port=serial_port, baudrate=115200)
        if not self.serial_conn.connect():
            raise SerialException("Could not connect to motor controller")
        
        self.controller = NintendoProController()
        self.controller.add_analog_callback("LS_x", self.lsx_callback)
        self.controller.add_analog_callback("LS_y", self.lsy_callback)
        
        self.right_velocity = 0.0
        self.left_velocity = 0.0
        self.max_velocity = 300
        self.ls_received = False

        self.controller.run()

    def lsy_callback(self, value):
        # Only calculate velocities if lsc_callback has been called with a new value
        if self.ls_received:
            x = getattr(self, 'lsx_value', 0.0)
            y = value
            # print(f"LS_y value: {y}, LS_x value: {x}")
            left_velocity, right_velocity = self.calculate_velocities(x, y)
            self.left_velocity = left_velocity
            self.right_velocity = right_velocity
            velocities = [self.right_velocity] * 3 + [self.left_velocity] * 3
            # parsed_data = self.serial_conn.spin_once()
            # print(f"Parsed data: {parsed_data}")
            # self.serial_conn.send_velocity_set([0.0, 0.0, 100.0, 100.0, 100.0, 100.0])
            self.serial_conn.send_velocity_set(velocities)
            print(f"Setting velocities: Left: {self.left_velocity}, Right: {self.right_velocity}")
            self.ls_received = False  # Reset flag after processing

    def lsx_callback(self, value):
        # Store the latest x value for use in lsy_callback
        self.ls_received = True
        self.lsx_value = value
        
    def a_callback(self, value):
        parsed_data = self.serial_conn.spin_once()

    def calculate_velocities(self, x, y):
        left_velocity = ((-y) + 0.5 * x) * self.max_velocity
        right_velocity = ((-y) - 0.5 * x) * self.max_velocity
        if left_velocity > self.max_velocity:
            left_velocity = self.max_velocity
        if left_velocity < -self.max_velocity:
            left_velocity = -self.max_velocity
        if right_velocity > self.max_velocity:
            right_velocity = self.max_velocity
        if right_velocity < -self.max_velocity:
            right_velocity = -self.max_velocity
        return left_velocity, right_velocity
        

if __name__ == "__main__":
    try:
        bt_drive = BTDrive()
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"An error occurred: {e}")
