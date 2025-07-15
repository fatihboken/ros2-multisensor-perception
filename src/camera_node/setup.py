from setuptools import setup

package_name = 'camera_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ika',
    maintainer_email='ika@example.com',
    description='Camera publisher node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_node.camera_node:main',
        ],
    },
)

