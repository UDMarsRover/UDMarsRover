from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the nav_vis_pub node
        Node(
            package='rover',
            executable='nav_vis_pub',
            name='nav_vis_pub',
            output='screen',
            parameters=[],
        ),
        # Launch the flow node
        Node(
            package='rover',
            executable='flow',
            name='optical_flow',
            output='screen',
            parameters=[],
        ),
    ])