#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial


class SerialSender(Node):

    def __init__(self):
        super().__init__('serial_sender')

        # Adjust for your device
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=1
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/arm_command',
            self.callback,
            10
        )

        self.get_logger().info("Serial Sender Started")

    def callback(self, msg):

        # Convert floats to integers
        int_values = [int(round(x)) for x in msg.data]

        # Create CSV string
        serial_string = ",".join(str(x) for x in int_values)

        # Add newline for easier parsing on MCU
        serial_string += "\n"

        self.ser.write(serial_string.encode())

        self.get_logger().info(
            f"Sent: {serial_string.strip()}"
        )

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SerialSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()