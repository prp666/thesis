import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('decision_layer')
    default_config = os.path.join(package_share, 'config', 'decision_node.yaml')

    return LaunchDescription([
        Node(
            package='decision_layer',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[default_config],
        ),
    ])
