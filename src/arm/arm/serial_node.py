import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

        self.create_subscription(
            JointState,
            '/joint_targets',
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

