import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class RoverTranslator(Node):
    def __init__(self):
        super().__init__('rover_translator')
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.servo_pub = self.create_publisher(Int32, 'servo_angle', 10)
        self.stepper_pub = self.create_publisher(Int32, 'stepper_speed', 10)
        
        self.timer = self.create_timer(0.05, self.publish_timer) # 20Hz Heartbeat
        self.servo_val = 90
        self.stepper_val = 0

    def joy_callback(self, msg):
        # PC handles the heavy mapping and deadzones
        # Axis 4 (Right Stick Up/Down) -> Servo
        self.servo_val = int(((msg.axes[4] + 1.0) / 2.0) * 180)
        
        # Axis 3 (Right Stick Left/Right) -> Stepper
        if abs(msg.axes[3]) > 0.1:
            self.stepper_val = int(msg.axes[3] * 2000)
        else:
            self.stepper_val = 0

    def publish_timer(self):
        s = Int32()
        s.data = self.servo_val
        self.servo_pub.publish(s)

        st = Int32()
        st.data = self.stepper_val
        self.stepper_pub.publish(st)

def main():
    rclpy.init()
    rclpy.spin(RoverTranslator())

if __name__ == '__main__':
    main()