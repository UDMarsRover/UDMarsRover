#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class TwistControlNode(Node):
    def __init__(self):
        super().__init__('twist_control_node')
        
        # Declare parameters for motor configuration
        # Motor indices: which physical motor corresponds to which position
        # Default: [0,1,2] = left side, [3,4,5] = right side
        self.declare_parameter('left_motor_indices', [0, 1, 2])
        self.declare_parameter('right_motor_indices', [3, 4, 5])
        
        # Scaling factors
        self.declare_parameter('max_linear_velocity', 1.0)  # m/s
        self.declare_parameter('max_angular_velocity', 2.0)  # rad/s
        self.declare_parameter('wheel_rpm_per_ms', 300.0)  # RPM output per m/s input
        self.declare_parameter('track_width', 0.5)  # meters between left and right wheels
        
        # Get parameters
        self.left_indices = self.get_parameter('left_motor_indices').value
        self.right_indices = self.get_parameter('right_motor_indices').value
        self.max_linear = self.get_parameter('max_linear_velocity').value
        self.max_angular = self.get_parameter('max_angular_velocity').value
        self.rpm_per_ms = self.get_parameter('wheel_rpm_per_ms').value
        self.track_width = self.get_parameter('track_width').value
        
        # Validate configuration
        if len(self.left_indices) + len(self.right_indices) != 6:
            self.get_logger().error(f'Motor configuration must have 6 motors total. Got {len(self.left_indices)} left + {len(self.right_indices)} right')
            raise ValueError('Invalid motor configuration')
        
        # Subscriber to twist commands
        self.subscription = self.create_subscription(
            Twist,
            '/drive/in/control',
            self.twist_callback,
            10)
        
        # Publisher for motor velocities
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'wheel_velocities_cmd',
            10)
        
        self.get_logger().info(f'Twist control node started')
        self.get_logger().info(f'Left motors: {self.left_indices}, Right motors: {self.right_indices}')
        self.get_logger().info(f'Max linear: {self.max_linear} m/s, Max angular: {self.max_angular} rad/s')
    
    def twist_callback(self, msg: Twist):
        # Extract linear and angular velocities from Twist message
        # For tank drive: linear.x = forward/backward, angular.z = rotation
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Clamp inputs to max values
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        
        # Tank drive kinematics:
        # left_velocity = (linear - angular * track_width / 2)
        # right_velocity = (linear + angular * track_width / 2)
        left_velocity = linear - (angular * self.track_width / 2.0)
        right_velocity = linear + (angular * self.track_width / 2.0)
        
        # Convert from m/s to RPM
        left_rpm = left_velocity * self.rpm_per_ms
        right_rpm = right_velocity * self.rpm_per_ms
        
        # Create velocity array for all 6 motors
        velocities = [0.0] * 6
        
        # Assign velocities based on motor configuration
        for idx in self.left_indices:
            velocities[idx] = left_rpm
        
        for idx in self.right_indices:
            velocities[idx] = right_rpm
        
        # Publish
        output_msg = Float32MultiArray()
        output_msg.data = velocities
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistControlNode()
    
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
