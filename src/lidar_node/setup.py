from setuptools import setup

package_name = 'lidar_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ika',
    maintainer_email='ika@example.com',
    description='RPLidar A1 için ROS 2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_reader_node = lidar_node.lidar_reader_node:main',
            'map_builder_node = lidar_node.map_builder_node:main',
        ],
    },
)

