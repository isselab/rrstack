# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
# SPDX-License-Identifier: MIT

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    gazebo_pkg = Path(get_package_share_directory("roboracer_gazebo"))
    slam_pkg = Path(get_package_share_directory("roboracer_slam"))

    gazebo_launch = gazebo_pkg / "launch" / "gazebo.launch.py"
    default_world = gazebo_pkg / "worlds" / "roboracer_track.world"

    slam_params = slam_pkg / "config" / "mapper_params_online_async.yaml"
    rviz_config = slam_pkg / "rviz" / "roboracer_slam.rviz"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            "world": world,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
        }.items(),
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            str(slam_params),
            {"use_sim_time": use_sim_time},
        ],
        arguments=[
            "--ros-args",
            "--log-level", "info",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d", str(rviz_config),
            "--ros-args",
            "--log-level", "error",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation time.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=str(default_world),
            description="Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="Vehicle spawn x position.",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Vehicle spawn y position.",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.1",
            description="Vehicle spawn z position.",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Vehicle spawn yaw angle.",
        ),

        gazebo,

        TimerAction(period=7.0, actions=[slam_toolbox]),
        TimerAction(period=9.0, actions=[rviz]),
    ])
