# SPDX-License-Identifier: MIT
# Author: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum

from glob import glob

from setuptools import find_packages, setup

package_name = 'roboracer_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models',
            glob('roboracer_perception/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarunbhyri9',
    maintainer_email='tarunbhyri9@todo.todo',
    description='YOLO cone detection for the RoboRacer stack',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = roboracer_perception.yolo_node:main',
            'delaunay_node = roboracer_perception.delaunay_node:main',
        ],
    },
)
