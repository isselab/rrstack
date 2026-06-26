# SPDX-License-Identifier: MIT
# Author of rrstack modifications: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# rrstack RoboRacer Gazebo integration.
# Note: Some Gazebo meshes, model files, world files, and structure are based on
# or inspired by TU Dortmund RoboRacer/F1TENTH reference files, where applicable.
# Third-party assets remain subject to their original license terms.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('roboracer_gazebo')

    # ===== GAZEBO_MODEL_PATH — lets Gazebo find model://flw_track =====
    models_path = os.path.join(pkg, 'models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path
    )

    # ===== Launch Arguments =====
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'roboracer_track.world'),
        description='Path to Gazebo world file'
    )
    x_arg   = DeclareLaunchArgument('x',   default_value='0.0', description='Spawn X')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0', description='Spawn Y')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.1', description='Spawn Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw')

   # ===== URDF =====
    urdf_file = os.path.join(pkg, 'models', 'f110_car', 'f110_car.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ===== Gazebo Classic =====
    # Start gzserver
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Start gzclient (without the buggy EOL plugin)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            )
        )
    )

    # ===== Robot State Publisher =====
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ===== Spawn car in Gazebo =====
    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_f110_car',
        output='screen',
        arguments=[
            '-entity', 'f110_car',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ]
    )

    return LaunchDescription([
        set_model_path,
        world_arg, x_arg, y_arg, z_arg, yaw_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_car,
    ])