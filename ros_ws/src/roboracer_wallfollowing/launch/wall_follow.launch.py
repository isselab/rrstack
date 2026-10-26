# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
# SPDX-License-Identifier: MIT

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    gazebo_pkg = Path(get_package_share_directory("roboracer_gazebo"))
    wallfollow_pkg = Path(get_package_share_directory("roboracer_wallfollowing"))

    params_file = wallfollow_pkg / "config" / "wall_follow_params.yaml"

    # ===== Gazebo: world + car (reuses roboracer_gazebo/gazebo.launch.py) =====
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_pkg / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "world": world,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
        }.items(),
    )

    # ===== Wall-follow PID node =====
    wall_follow_node = Node(
        package="roboracer_wallfollowing",
        executable="wall_follow_pid_node",
        name="wall_follow_pid_node",
        output="screen",
        parameters=[
            str(params_file),
            {"side": side, "use_sim_time": True},
        ],
    )

    # ===== /drive -> /cmd_vel bridge (simulation only) =====
    # The PID publishes /drive (real-car interface); Gazebo only reads /cmd_vel.
    drive_to_cmd_vel = Node(
        package="roboracer_drive_to_cmd_vel",
        executable="drive_to_cmd_vel",
        name="drive_to_cmd_vel",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "side",
            default_value="left",
            description="Which wall to follow: 'left' or 'right'.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="walls",
            description="World: 'walls', 'cones', or full path to a .world file.",
        ),
        DeclareLaunchArgument("x", default_value="0.0", description="Vehicle spawn x position."),
        DeclareLaunchArgument("y", default_value="0.0", description="Vehicle spawn y position."),
        DeclareLaunchArgument("z", default_value="0.1", description="Vehicle spawn z position."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Vehicle spawn yaw angle."),

        gazebo,

        TimerAction(period=7.0, actions=[wall_follow_node, drive_to_cmd_vel]),
    ])