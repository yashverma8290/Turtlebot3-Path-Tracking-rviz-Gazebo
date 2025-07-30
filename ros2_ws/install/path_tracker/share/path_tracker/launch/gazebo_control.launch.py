from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_tracker',
            executable='controller_node',
            output='screen'
        )
    ])
