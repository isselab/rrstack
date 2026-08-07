# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
# SPDX-License-Identifier: MIT

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    side = LaunchConfiguration("side")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    gazebo_pkg = get_package_share_directory("roboracer_gazebo")
    wallfollow_pkg = Path(get_package_share_directory("roboracer_wallfollowing"))

    default_world = os.path.join(gazebo_pkg, "worlds", "roboracer_track.world")
    params_file = wallfollow_pkg / "config" / "wall_follow_params.yaml"

    # ===== GAZEBO_MODEL_PATH — lets Gazebo find model://flw_track =====
    models_path = os.path.join(gazebo_pkg, "models")
    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=models_path,
    )

    # ===== URDF =====
    urdf_file = os.path.join(gazebo_pkg, "models", "f110_car", "f110_car.urdf")
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # ===== Gazebo Classic =====
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch", "gzserver.launch.py",
            )
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch", "gzclient.launch.py",
            )
        )
    )

    # ===== Robot State Publisher =====
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ===== Spawn car in Gazebo =====
    spawn_car = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_f110_car",
        output="screen",
        arguments=[
            "-entity", "f110_car",
            "-topic", "robot_description",
            "-x", x,
            "-y", y,
            "-z", z,
            "-Y", yaw,
            "-timeout", "120",
        ],
    )

    # ===== Wall-follow PID node =====
    wall_follow_node = Node(
        package="roboracer_wallfollowing",
        executable="wall_follow_pid_node",
        name="wall_follow_pid_node",
        output="screen",
        parameters=[
            str(params_file),
            {"side": side},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "side",
            default_value="left",
            description="Which wall to follow: 'left' or 'right'.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Gazebo world file.",
        ),
        DeclareLaunchArgument("x", default_value="0.0", description="Vehicle spawn x position."),
        DeclareLaunchArgument("y", default_value="0.0", description="Vehicle spawn y position."),
        DeclareLaunchArgument("z", default_value="0.1", description="Vehicle spawn z position."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Vehicle spawn yaw angle."),

        set_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_car,

        TimerAction(period=7.0, actions=[wall_follow_node]),
    ])