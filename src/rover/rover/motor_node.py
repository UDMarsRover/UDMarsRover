import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rover.drive.UDMRTMotorSerial import UDMRTMotorSerial


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.get_logger().info("MotorNode has been started.")
        self.serial_conn = UDMRTMotorSerial(port="/dev/ttyAMA0", baudrate=115200)
        if not self.serial_conn.connect():
            self.get_logger().error("Could not connect to motor controller")
        
        # Subscribers
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_velocities',
            self.motor_speed_callback,
            10
        )
        
        # Motor speed variable
        self.current_speed = 0.0

    def motor_speed_callback(self, msg):
        self.current_speed = msg.data

        self.get_logger().info(f"Received motor speed: {self.current_speed}")
        self.set_motor_speed(self.current_speed)

    def set_motor_speed(self, speed):
        # Placeholder for motor control logic
        self.get_logger().info(f"Setting motor speed to: {speed}")
        # Add hardware-specific motor control code here

    def sniff(self):
        self.serial_conn.spin_once()
        parsed_data = self.serial_conn.spin_once()
        return parsed_data

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()

    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        motor_node.get_logger().info("Shutting down MotorNode.")
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()