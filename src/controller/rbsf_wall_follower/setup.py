import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rbsf_wall_follower"
launch_files = glob(os.path.join("launch", "*.launch.py")) + glob(
    os.path.join("launch", "*_launch.py")
)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            glob(os.path.join("config", "*.yaml")),
        ),
        ("share/" + package_name + "/launch", launch_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rodrigo Gutiérrez-Moreno",
    maintainer_email="rodrigo.gutierrez@uah.es",
    description="Simple PID wall follower using LaserScan and Ackermann commands.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "wall_follower = rbsf_wall_follower.wall_follower_node:main",
        ],
    },
)
