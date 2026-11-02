# SPDX-License-Identifier: MIT
# Author: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Part of the rrstack RoboRacer software stack.

from glob import glob

from setuptools import find_packages, setup

package_name = 'roboracer_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarunbhyri9',
    maintainer_email='tarunbhyri9@todo.todo',
    description='Cone-based Delaunay waypoints with centreline PID / waypoint pure pursuit',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'delaunay_node = roboracer_planning.delaunay_node:main',
            'centerline_pid_node = roboracer_planning.centerline_pid_node:main',
            'waypoint_pursuit_node = roboracer_planning.waypoint_pursuit_node:main',
        ],
    },
)
