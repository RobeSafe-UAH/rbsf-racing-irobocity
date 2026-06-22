import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stack_launcher'
launch_files = glob(os.path.join('launch', '*.launch.py')) + glob(
    os.path.join('launch', '*_launch.py')
)

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
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santi',
    maintainer_email='santi@todo.todo',
    description='Launch files and utilities for the iRobocity racing stack.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_movement = stack_launcher.simple_movement_node:main',
            'simple_odom = stack_launcher.simple_odom_node:main',
            'ackermann_to_twist = stack_launcher.ackermann_to_twist_node:main',
            'keyboard_teleop = stack_launcher.keyboard_teleop_node:main',
        ],
    },
)
