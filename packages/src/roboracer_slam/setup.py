from glob import glob
import os

from setuptools import find_packages, setup

package_name = "roboracer_slam"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AVAI Team",
    maintainer_email="",
    description="SLAM integration for rrstack using LiDAR, odometry, TF, slam_toolbox, Gazebo, and RViz.",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
