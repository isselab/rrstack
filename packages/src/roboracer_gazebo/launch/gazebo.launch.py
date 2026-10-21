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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('roboracer_gazebo')

    # ===== GAZEBO_MODEL_PATH — lets Gazebo find model://flw_track =====
    models_path = os.path.join(pkg, 'models')
    share_path = os.path.dirname(pkg)   # .../install/roboracer_gazebo/share
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + os.pathsep + share_path
    )

    # ===== Launch Arguments =====
    # World selection: 'walls', 'cones', or a full path
    walls_world = os.path.join(pkg, 'worlds', 'roboracer_track.world')
    cones_world = os.path.join(pkg, 'worlds', 'flw_cone_track_from_walls.world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='walls',
        description="World: 'walls', 'cones', or full path to a .world file"
    )
    world = LaunchConfiguration('world')
    world_file = PythonExpression([
        "{'walls': '", walls_world, "', 'cones': '", cones_world, "'}"
        ".get('", world, "', '", world, "')"
    ])

    x_arg   = DeclareLaunchArgument('x',   default_value='0.0', description='Spawn X')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0', description='Spawn Y')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.1', description='Spawn Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw')
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
        launch_arguments={'world': world_file}.items()
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
            '-timeout', '120',
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