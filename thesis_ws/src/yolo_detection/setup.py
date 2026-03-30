from setuptools import find_packages, setup
from glob import glob

package_name = 'yolo_detection'
picture_files = (
    glob('pictures/*.jpg') +
    glob('pictures/*.jpeg') +
    glob('pictures/*.png') +
    glob('pictures/*.bmp')
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/models',
            glob('models/*.tflite')
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
        (
            'share/' + package_name + '/pictures',
            picture_files
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prp',
    maintainer_email='pengruipu@gmail.com',
    description='ROS2 vision detection node backed by a TFLite YOLO model.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_image_source_node = yolo_detection.fake_image_source_node:main',
            'vision_detector = yolo_detection.vision_detector:main',
        ],
    },
)
