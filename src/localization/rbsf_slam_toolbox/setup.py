from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rbsf_slam_toolbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robesafe',
    maintainer_email='robesafe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_saver = rbsf_slam_toolbox.map_saver_node:main',
            'centerline_publisher = rbsf_slam_toolbox.centerline_publisher_node:main',
        ],
    },
)
