import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    decision_share = get_package_share_directory('decision_layer')
    yolo_share = get_package_share_directory('yolo_detection')

    decision_config = os.path.join(decision_share, 'config', 'decision_node.yaml')
    fake_image_config = os.path.join(yolo_share, 'config', 'fake_image_source.yaml')
    vision_config = os.path.join(yolo_share, 'config', 'vision_detector.yaml')

    return LaunchDescription([
        Node(
            package='yolo_detection',
            executable='fake_image_source_node',
            name='fake_image_source_node',
            output='screen',
            parameters=[fake_image_config],
        ),
        Node(
            package='yolo_detection',
            executable='vision_detector',
            name='vision_detector_node',
            output='screen',
            parameters=[vision_config],
        ),
        Node(
            package='sound_detection',
            executable='fake_audio_source_node',
            name='fake_audio_source_node',
            output='screen',
            parameters=[{'drone_present': True}],
        ),
        Node(
            package='sound_detection',
            executable='sound_presence_detector_node',
            name='sound_presence_detector_node',
            output='screen',
        ),
        Node(
            package='decision_layer',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[decision_config],
        ),
    ])
