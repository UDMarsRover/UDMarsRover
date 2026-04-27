import os

from ament_index_python import get_package_share_directory
import launch
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover')
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf.yaml')

    imu_xyz = ["-0.1", "0.0", "0.2"]   # x y z
    imu_rpy = ["0.0", "0.0", "3.1416"]   # roll pitch yaw

    return launch.LaunchDescription([\
        
        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     name='micro_ros_agent',
        #     output='screen',
        #     arguments=['serial', '--dev', '/dev/ttyACM0'],
        # ),
        Node(
            package='rover',
            executable='twist_control',
            name='twist_control_node',
            output='screen'
        ),
        Node(
            package='rover',
            executable='joy_twist',
            name='joy_twist_node',
            output='screen'
        ),
        Node(
            package='rover',
            executable='joint_state_pub',
            name='joint_state_pub_node',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_to_imu",
            arguments=[
                "--x", imu_xyz[0], "--y", imu_xyz[1], "--z", imu_xyz[2],
                "--roll", imu_rpy[0], "--pitch", imu_rpy[1], "--yaw", imu_rpy[2],
                "--frame-id", "base_link",
                "--child-frame-id", "imu_link",
            ],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_yaml],
            remappings=[
                # optional, only if your topics differ
                # ("/odom", "/odom"),
                # ("/imu_raw", "/imu_raw"),
            ],
        ),
    ])
