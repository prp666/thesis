from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_layer',
            executable='action_node',
            name='action_node',
            output='screen',
        ),
    ])
