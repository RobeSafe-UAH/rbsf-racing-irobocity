from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rbsf_mvsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/maps', glob(os.path.join('maps', '*.world.xml'))),
        ('share/' + package_name + '/definitions', glob(os.path.join('definitions', '*.xml'))),
        ('share/' + package_name + '/models', glob(os.path.join('models', '*'))),
        ('share/' + package_name + '/rviz', glob(os.path.join('rviz', '*.rviz'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),

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
            'generate_mvsim_world = rbsf_mvsim.world_generator:main',
        ],
    },
)
