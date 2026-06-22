import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rbsf_wall_following"
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
        ("share/" + package_name + "/launch", launch_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="santi",
    maintainer_email="santi@todo.todo",
    description="Simple PI wall following using LaserScan and Ackermann commands.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "wall_follower = rbsf_wall_following.wall_follower_node:main",
        ],
    },
)
