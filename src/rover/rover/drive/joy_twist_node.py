#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTwistNode(Node):
    def __init__(self):
        super().__init__('joy_twist_node')
        
        # Declare parameters for joystick configuration
        self.declare_parameter('linear_axis', 1)  # Left stick Y-axis (forward/back)
        self.declare_parameter('angular_axis', 0)  # Left stick X-axis (left/right)
        self.declare_parameter('deadzone', 0.1)  # Ignore small movements
        self.declare_parameter('linear_scale', 1.0)  # Max linear velocity (m/s)
        self.declare_parameter('angular_scale', 2.0)  # Max angular velocity (rad/s)
        self.declare_parameter('invert_linear', False)  # Invert forward/back
        self.declare_parameter('invert_angular', False)  # Invert left/right
        
        # Get parameters
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.deadzone = self.get_parameter('deadzone').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.invert_linear = self.get_parameter('invert_linear').value
        self.invert_angular = self.get_parameter('invert_angular').value
        
        # Subscriber to joy messages
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        # Publisher for twist commands
        self.publisher_ = self.create_publisher(
            Twist,
            '/drive/in/control',
            10)
        
        self.get_logger().info('Joy twist node started')
        self.get_logger().info(f'Linear axis: {self.linear_axis}, Angular axis: {self.angular_axis}')
        self.get_logger().info(f'Deadzone: {self.deadzone}, Linear scale: {self.linear_scale}, Angular scale: {self.angular_scale}')
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick value"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the remaining range
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg: Joy):
        # Check if axes exist
        if len(msg.axes) <= max(self.linear_axis, self.angular_axis):
            self.get_logger().warn(f'Joy message has {len(msg.axes)} axes, need at least {max(self.linear_axis, self.angular_axis)+1}')
            return
        
        # Extract axis values
        linear_raw = msg.axes[self.linear_axis]
        angular_raw = msg.axes[self.angular_axis]
        
        # Apply inversion
        if self.invert_linear:
            linear_raw = -linear_raw
        if self.invert_angular:
            angular_raw = -angular_raw
        
        # Apply deadzone
        linear_filtered = self.apply_deadzone(linear_raw)
        angular_filtered = self.apply_deadzone(angular_raw)
        
        # Scale to target velocities
        linear_vel = linear_filtered * self.linear_scale
        angular_vel = angular_filtered * self.angular_scale
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTwistNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
