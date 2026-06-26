# SPDX-License-Identifier: MIT
# Author of rrstack modifications: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# rrstack RoboRacer Gazebo integration.
# Note: Some Gazebo meshes, model files, world files, and structure are based on
# or inspired by TU Dortmund RoboRacer/F1TENTH reference files, where applicable.
# Third-party assets remain subject to their original license terms.

from setuptools import find_packages, setup

package_name = 'roboracer_gazebo'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
