import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
        ),
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
        )
    ])
