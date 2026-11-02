# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
# SPDX-License-Identifier: MIT
#
# REAL CAR: cone pipeline without any simulator parts.
#
#   RealSense --> yolo_node --> delaunay_node --> controller --/drive--> VESC
#
# Differences to cone_drive.launch.py (simulation):
#   - no Gazebo, no steering_lag_compensator, no drive_to_cmd_vel
#     (the VESC reads /drive directly; a real servo has no 1 s steering lag)
#   - camera topics are the RealSense names: <camera_prefix>/color/image_raw,
#     <camera_prefix>/aligned_depth_to_color/image_raw, <camera_prefix>/color/camera_info
#     (the simulator keeps its own /camera/color/... names)
#   - use_sim_time false (system clock); speed capped by max_speed for first runs
#
# Started in stages, like the simulation launch: nothing publishes /drive
# until the camera is up and YOLO is detecting cones.
#
#   1. yolo_node            when the first camera_info arrives (camera driver running)
#   2. planner + controller when the first /cones/blue arrives

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = ('pursuit', 'pid')


def wait_for(topic, msg_type, name):
    """Process that exits as soon as the first message on `topic` arrives."""
    return ExecuteProcess(cmd=['ros2', 'topic', 'echo', '--once', topic, msg_type],
                          name=name, output='log')


def then(waiter, message, actions):
    """Start `actions` when `waiter` exits."""
    return RegisterEventHandler(OnProcessExit(
        target_action=waiter, on_exit=[LogInfo(msg=message)] + actions))


def build(context):
    """All car actions, built once the launch arguments are known (plain strings)."""
    arg = lambda name: LaunchConfiguration(name).perform(context)
    controller = arg('controller')
    if controller not in CONTROLLERS:
        # Without this, a typo matches no controller: everything starts but
        # nothing publishes /drive and the car just stands still.
        raise RuntimeError(
            f"unknown controller '{controller}': use controller:=pursuit or controller:=pid")
    prefix = arg('camera_prefix').rstrip('/')
    max_speed = float(arg('max_speed'))
    clock = {'use_sim_time': arg('use_sim_time').lower() == 'true'}

    planning_pkg = Path(get_package_share_directory('roboracer_planning'))
    params = str(planning_pkg / 'config' / 'planning_params.yaml')
    rviz_cfg = str(planning_pkg / 'rviz' / 'cone_planning.rviz')

    # RealSense topic names (realsense-ros 4.x: /<namespace>/<camera name>/...).
    # Only this launch uses them; the simulator keeps /camera/color/... .
    image_topic = prefix + '/color/image_raw'
    depth_topic = prefix + '/aligned_depth_to_color/image_raw'
    info_topic = prefix + '/color/camera_info'

    yolo = Node(package='roboracer_perception', executable='yolo_node',
                name='yolo_cone_debug_node', output='screen',
                parameters=[clock, {'image_topic': image_topic,
                                    'depth_topic': depth_topic,
                                    'camera_info_topic': info_topic,
                                    'yellow_conf': float(arg('yellow_conf'))}])

    delaunay = Node(package='roboracer_planning', executable='delaunay_node',
                    name='delaunay_node', output='screen', parameters=[params, clock])

    # Exactly one controller; max_speed overrides the yaml speed for first runs.
    if controller == 'pursuit':
        ctrl = Node(package='roboracer_planning', executable='waypoint_pursuit_node',
                    name='waypoint_pursuit_node', output='screen',
                    parameters=[params, clock, {'target_speed': max_speed,
                                                'maximum_speed': max_speed}])
    else:
        ctrl = Node(package='roboracer_planning', executable='centerline_pid_node',
                    name='centerline_pid_node', output='screen',
                    parameters=[params, clock, {'nominal_speed': max_speed}])

    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
                     arguments=['-d', rviz_cfg], parameters=[clock],
                     condition=IfCondition(LaunchConfiguration('rviz')))

    wait_camera = wait_for(info_topic, 'sensor_msgs/msg/CameraInfo', 'wait_for_camera')
    wait_cones = wait_for('/cones/blue', 'geometry_msgs/msg/PoseArray', 'wait_for_cones')

    return [
        LogInfo(msg=f'[car] {controller}, max {max_speed} m/s; waiting for the camera on {info_topic} ...'),
        wait_camera,
        then(wait_camera, '[stage 1] camera is up: starting yolo_node; waiting for cones...',
             [yolo, wait_cones]),
        then(wait_cones, '[stage 2] cones detected: starting planner + controller (/drive)',
             [delaunay, ctrl, rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='pursuit',
                              description="Controller: 'pursuit' (recommended) or 'pid'."),
        DeclareLaunchArgument('camera_prefix', default_value='/camera/realsense2_camera',
                              description='RealSense topic prefix (check: ros2 topic list).'),
        DeclareLaunchArgument('max_speed', default_value='0.8',
                              description='Speed cap [m/s]; sim-tested up to 1.5.'),
        DeclareLaunchArgument('yellow_conf', default_value='0.3',
                              description='yolo_node publish gate for yellow cones (R9).'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='false on the car; true only for testing in Gazebo.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.'),
        OpaqueFunction(function=build),
    ])
