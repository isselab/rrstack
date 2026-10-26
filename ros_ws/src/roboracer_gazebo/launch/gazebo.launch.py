# SPDX-License-Identifier: MIT
# Author of rrstack modifications: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# rrstack RoboRacer Gazebo (Ignition) integration.
# Note: Some Gazebo meshes, model files, world files, and structure are based on
# or inspired by TU Dortmund RoboRacer/F1TENTH reference files, where applicable.
# Third-party assets remain subject to their original license terms.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('roboracer_gazebo')

    # ===== Resource path: lets Ignition find model://flw_track, model://f110_car, ... =====
    models_path = os.path.join(pkg, 'models')
    share_path = os.path.dirname(pkg)   # .../install/roboracer_gazebo/share
    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path + os.pathsep + share_path + os.pathsep +
              os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
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

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch the Ignition Gazebo GUI (set false for headless).'
    )
    gui = LaunchConfiguration('gui')

    x_arg   = DeclareLaunchArgument('x',   default_value='0.0', description='Spawn X')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0', description='Spawn Y')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.1', description='Spawn Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw')

    # World name used in the world-control (reset) service. It must match the
    # <world name="..."> inside the world file. Derived automatically from the
    # 'world' argument for the built-in worlds; override for custom worlds.
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='auto',
        description="Name of the Ignition world (reset service). 'auto' infers "
                    "it from the 'world' argument."
    )
    world_name = LaunchConfiguration('world_name')
    resolved_world_name = PythonExpression([
        "'", world_name, "' if '", world_name, "' != 'auto' else "
        "{'walls': 'roboracer_track', 'cones': 'flw_cone_track_from_walls_lane_origin'}"
        ".get('", world, "', 'roboracer_track')"
    ])

    # ===== URDF (only for robot_state_publisher / TF of static frames) =====
    urdf_file = os.path.join(pkg, 'models', 'f110_car', 'f110_car.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ===== Ignition Gazebo (server + optional GUI) =====
    # -r : start running (unpaused)
    gz_args_prefix = PythonExpression(["'-r ' if '", gui, "' == 'true' else '-r -s '"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [gz_args_prefix, world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ===== Robot State Publisher (TF for the static link tree) =====
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # ===== Spawn the Ignition SDF model into the world =====
    car_sdf = os.path.join(pkg, 'models', 'f110_car', 'model.sdf')
    spawn_car = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_f110_car',
        output='screen',
        arguments=[
            '-world', resolved_world_name,
            '-file', car_sdf,
            '-name', 'f110_car',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ]
    )

    # ===== ROS <-> Ignition bridge =====
    # All AckermannSteering + sensor topics are set explicitly in model.sdf
    # (see the <topic> elements), so they are FLAT Ignition topics:
    #   /cmd_vel, /odom, /tf, /scan, /imu, /joint_states
    #   /camera/image, /camera/depth_image, /camera/camera_info, /camera/points
    # We bridge them and remap the ROS side where a different name is expected
    # (the RealSense-style camera topic names the perception stack uses).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # drive command: ROS -> Ignition
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # odometry: Ignition -> ROS
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # tf: Ignition Pose_V -> ROS TFMessage
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # lidar
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # imu
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # joint states
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # rgbd camera (rgb image, depth image, camera info, points)
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # world control service (used to reset the clock by the RL env).
            # The Ignition side always serves /world/<world_name>/control.
            ['/world/', resolved_world_name, '/control@ros_gz_interfaces/srv/ControlWorld'],
            # NOTE: the /world/<name>/set_pose service uses a flat
            # ignition.msgs.Pose request that the parameter_bridge cannot map
            # onto ros_gz_interfaces/srv/SetEntityPose, so the RL env calls the
            # `ign service` CLI directly to teleport the car (see gym_env.py).
        ],
        remappings=[
            # Map the raw RGBD camera topics to the RealSense-style names the
            # perception stack (yolo_node) subscribes to.
            ('/camera/image', '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
            ('/camera/camera_info', '/camera/color/camera_info'),
            ('/camera/points', '/camera/depth/color/points'),
        ],
    )

    return LaunchDescription([
        set_resource_path,
        world_arg, gui_arg, world_name_arg, x_arg, y_arg, z_arg, yaw_arg,
        gazebo,
        robot_state_publisher,
        spawn_car,
        bridge,
    ])
