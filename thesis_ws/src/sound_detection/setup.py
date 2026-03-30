from setuptools import find_packages, setup

package_name = 'sound_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prp',
    maintainer_email='pengruipu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "fake_audio_source_node = sound_detection.fake_audio_source_node:main",
            "sound_presence_detector_node = sound_detection.sound_presence_detector_node:main",
        ],
    },
)
