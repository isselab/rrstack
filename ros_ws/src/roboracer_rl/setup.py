# SPDX-License-Identifier: MIT
# Author: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Part of the rrstack RoboRacer software stack.

from setuptools import find_packages, setup

package_name = 'roboracer_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarunbhyri9',
    maintainer_email='tarunbhyri9@todo.todo',
    description='Minimal RL controller for the RoboRacer Gazebo simulation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rl_train = roboracer_rl.train:main',
            'rl_policy_node = roboracer_rl.policy_node:main',
            'rl_random_policy = roboracer_rl.random_policy:main',
        ],
    },
)
