# SPDX-License-Identifier: MIT
# Author: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Part of the rrstack RoboRacer software stack.

from setuptools import find_packages, setup

package_name = 'roboracer_wallfollowing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_follow.launch.py']),
        ('share/' + package_name + '/config', ['config/wall_follow_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarunbhyri9',
    maintainer_email='tarunbhyri9@todo.todo',
    description='PID wall-following controller using two LiDAR beams and odometry',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follow_pid_node = roboracer_wallfollowing.wall_follow_pid_node:main',
        ],
    },
)
