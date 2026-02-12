#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Declare ROS parameters
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('gear_ratio', 1.0/75.0)
        self.declare_parameter('encoder_ticks_per_revolution', 42.0)
        self.declare_parameter('wheel_diameter', 0.2)  # in meters
        self.declare_parameter('robot_width', 0.5)  # in meters

        # Get parameters
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.encoder_ticks_per_revolution = self.get_parameter('encoder_ticks_per_revolution').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.robot_width = self.get_parameter('robot_width').get_parameter_value().double_value

        # Ensure joint names match expected count
        if len(self.joint_names) != 6:
            self.get_logger().error("Expected 6 joint names, but got {}".format(len(self.joint_names)))
            rclpy.shutdown()
            return

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Subscriber for wheel status
        self.wheel_status_subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_status',
            self.wheel_status_callback,
            10
        )

        # Initialize Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info("JointStatePublisher node initialized.")

    def wheel_status_callback(self, msg):
        data = msg.data

        if len(data) < 36:  # Ensure data has enough fields for 6 motors
            self.get_logger().error("Received wheel_status data with insufficient length: {}".format(len(data)))
            return

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = []
        joint_state_msg.velocity = []

        wheel_velocities = []

        for i in range(6):
            base = i * 6
            velocity = data[base + 0]  # Velocity in RPM
            position = data[base + 1]  # Position in encoder ticks

            # Convert velocity from RPM to radians per second
            velocity_rad_s = (velocity / 60.0) * 2.0 * 3.14159265359 * self.gear_ratio

            # Convert position from encoder ticks to radians
            position_rad = (position / self.encoder_ticks_per_revolution) * 2.0 * 3.14159265359 * self.gear_ratio

            joint_state_msg.velocity.append(velocity_rad_s)
            joint_state_msg.position.append(position_rad)

            # Convert velocity to linear velocity (m/s)
            linear_velocity = velocity_rad_s * (self.wheel_diameter / 2.0)
            wheel_velocities.append(linear_velocity)

        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info("Published joint states.")

        # Compute odometry
        self.compute_odometry(wheel_velocities)

    def compute_odometry(self, wheel_velocities):
        # Assuming tank steering: left wheels [0, 1, 2], right wheels [3, 4, 5]
        left_velocity = sum(wheel_velocities[:3]) / 3.0
        right_velocity = sum(wheel_velocities[3:]) / 3.0

        # Compute linear and angular velocity
        linear_velocity = (left_velocity + right_velocity) / 2.0
        angular_velocity = (right_velocity - left_velocity) / self.robot_width

        # Time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Update pose
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_publisher.publish(odom_msg)
        self.get_logger().info("Published odometry.")

        # Publish TF from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()