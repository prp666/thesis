import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('yolo_detection')
    default_config = os.path.join(package_share, 'config', 'vision_detector.yaml')

    return LaunchDescription([
        Node(
            package='yolo_detection',
            executable='vision_detector',
            name='vision_detector_node',
            output='screen',
            parameters=[default_config],
        ),
    ])
