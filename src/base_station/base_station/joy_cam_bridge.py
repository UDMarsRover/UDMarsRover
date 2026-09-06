#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class CameraJoyBridge(Node):
    def __init__(self):
        super().__init__('camera_joy_bridge')
        
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
            
        self.pan_pub = self.create_publisher(Int32, 'camera_pan', 10)
        self.tilt_pub = self.create_publisher(Int32, 'camera_tilt', 10)
        
        self.current_pan = 0.0
        self.current_tilt = 90.0
        
        self.stick_pan = 0.0
        self.stick_tilt = 0.0
        
        self.pan_speed = 5.0   
        self.tilt_speed = 2.0  
        self.deadzone = 0.20 

        self.timer = self.create_timer(0.05, self.motion_loop)

        self.get_logger().info("Joystick Bridge Ready! Controlling via Axes 3 and 4.")

    def joy_callback(self, msg):
        # The image confirms your array length is 8, so checking for 5 is perfectly safe
        if len(msg.axes) >= 5:
            # Directly reading indexes 3 and 4 with no weird math
            self.stick_pan = msg.axes[3]
            self.stick_tilt = msg.axes[4]
            
        if len(msg.buttons) > 0:
            if msg.buttons[0] == 1:
                self.get_logger().info("Center Button Pressed!")
                self.current_pan = 0.0
                self.current_tilt = 90.0
                self.stick_pan = 0.0
                self.stick_tilt = 0.0

    def motion_loop(self):
        moved = False
        
        if abs(self.stick_pan) > self.deadzone:
            self.current_pan += (self.stick_pan * self.pan_speed)
            moved = True
            
        if abs(self.stick_tilt) > self.deadzone:
            self.current_tilt += (self.stick_tilt * self.tilt_speed)
            moved = True
            
        if moved:
            self.current_pan = max(-200.0, min(200.0, self.current_pan))
            self.current_tilt = max(0.0, min(180.0, self.current_tilt))
            
            pan_msg = Int32()
            pan_msg.data = int(self.current_pan)
            self.pan_pub.publish(pan_msg)
            
            tilt_msg = Int32()
            tilt_msg.data = int(self.current_tilt)
            self.tilt_pub.publish(tilt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraJoyBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
