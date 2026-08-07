import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from arm_control.cpp import arm_control

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

        self.create_subscription(
            arm_control,
            '/arm_command',
            self.callback,
            10
        )

    def callback(self, msg):
        cmd = "<j," + ",".join([f"{p:.3f}" for p in msg.position]) + ">\n"
        self.ser.write(cmd.encode())

def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    rclpy.shutdown()

