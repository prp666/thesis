from glob import glob

from setuptools import find_packages, setup

package_name = 'action_layer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prp',
    maintainer_email='pengruipu@gmail.com',
    description='ROS2 action layer stub for Docker-only threat response validation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'action_node = action_layer.action_node:main',
        ],
    },
)
