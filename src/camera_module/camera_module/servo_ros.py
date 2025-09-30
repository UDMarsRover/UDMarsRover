import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import smbus2
import time
from collections import deque

# --- I2C Constants ---
# The I2C address of the Pi Pico slave device.
# This must match the address defined in the Pico's sketch.
I2C_SLAVE_ADDRESS = 8

# The Raspberry Pi 3B+, 4, and later models use I2C bus 1.
I2C_BUS_NUMBER = 1

# --- Servo Constants ---
# Min and Max angles of the servo in degrees
MIN_ANGLE = 0
MAX_ANGLE = 180

# --- Moving Average Filter Settings ---
SMOOTHING_WINDOW_SIZE = 5

class I2CServoNode(Node):
    def __init__(self):
        super().__init__('i2c_servo_node')

        # Initialize I2C bus
        try:
            self.bus = smbus2.SMBus(I2C_BUS_NUMBER)
            time.sleep(2)  # Give the bus time to initialize
            self.get_logger().info(f"I2C bus {I2C_BUS_NUMBER} opened successfully.")
        except FileNotFoundError:
            self.get_logger().error("Error: I2C bus not found. Make sure I2C is enabled in raspi-config.")
            raise Exception("I2C bus not found")

        # Initialize the moving average filter
        # self.position_history = deque(maxlen=SMOOTHING_WINDOW_SIZE)
        self.last_angle = 90  # Start at a neutral position
        self.last_sent_angle = 90

        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        # self.camera_center_pub = self.create_publisher(Bool, 'camera_center', 10)
        self.camera_center_sub = self.create_subscription(
            Bool,
            'camera_center',
            self.camera_center_callback,
            10
        )
        self.get_logger().info("I2C Servo Node started. Listening on 'servo_position' topic.")

    def camera_center_callback(self, msg):
        if msg.data:
            self.get_logger().info("Camera center command received. Resetting servo position to 90 degrees.")
            self.last_angle = 90

    def listener_callback(self, msg):
        # The input data is assumed to be a normalized value (e.g., -1 to 1) or a direct angle.
        # This script expects a normalized value and converts it to a 0-180 degree angle.
        # You may need to adjust this logic based on your specific ROS message data.
        normalized_position = msg.data
        if abs(normalized_position) < 0.05:
            return
        # Add the new data to the smoothing window

        # Calculate the average of the values in the window

        # Convert the smoothed normalized value (-1 to 1) to an angle (0 to 180)
        # Assuming the incoming data is a normalized value from -1 to 1,
        # where -1 maps to MIN_ANGLE and 1 maps to MAX_ANGLE.
        angle = self.last_angle + normalized_position * 5
        # Constrain the angle to a valid range
        angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))

        try:
            # The Pico expects two bytes for the angle.
            # Convert the integer angle to two bytes using big-endian byte order.
            if abs(angle - self.last_sent_angle) > 1:
                # If the angle hasn't changed significantly, skip sending to avoid jitter.
                data = int(angle).to_bytes(2, byteorder='big')
            
            # Write the command byte (0x00) and the two-byte angle data to the Pico.
                self.bus.write_i2c_block_data(I2C_SLAVE_ADDRESS, 0x00, list(data))
                self.last_sent_angle = angle
            self.last_angle = angle
        except Exception as e:
            self.get_logger().error(f"Failed to send I2C data: {e}")

    def destroy_node(self):
        # Close the I2C bus when the program ends.
        if hasattr(self, 'bus'):
            self.bus.close()
            self.get_logger().info("I2C bus closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = I2CServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()