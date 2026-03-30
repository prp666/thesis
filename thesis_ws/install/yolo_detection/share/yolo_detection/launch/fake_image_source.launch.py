import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('yolo_detection')
    default_config = os.path.join(package_share, 'config', 'fake_image_source.yaml')

    return LaunchDescription([
        Node(
            package='yolo_detection',
            executable='fake_image_source_node',
            name='fake_image_source_node',
            output='screen',
            parameters=[default_config],
        ),
    ])
