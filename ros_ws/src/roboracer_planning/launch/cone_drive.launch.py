# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
# SPDX-License-Identifier: MIT
#
# Full cone-driving pipeline in Gazebo, started in stages. Each stage starts
# only when the previous one is actually ready (no fixed timers):
#
#   1. Gazebo + car       (gazebo.launch.py spawns the car once /clock runs)
#   2. yolo_node          when the first /camera/color/camera_info arrives
#   3. planner + control  when the first /cones/blue arrives (YOLO is detecting)
#   4. RViz               when the first /waypoints arrives
#
# controller:=pursuit  waypoint_pursuit_node (default, recommended)
# controller:=pid      centerline_pid_node
# Any other value stops the launch with an error before Gazebo starts.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def wait_for(topic, msg_type, name):
    """Process that exits as soon as the first message on `topic` arrives.

    The type is given explicitly so `echo` waits for the topic to appear
    instead of failing because it does not exist yet.
    """
    return ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', topic, msg_type],
        name=name, output='log',
    )


def then(waiter, message, actions):
    """Start `actions` when `waiter` exits."""
    return RegisterEventHandler(OnProcessExit(
        target_action=waiter, on_exit=[LogInfo(msg=message)] + actions))


CONTROLLERS = ('pursuit', 'pid')


def check_controller(context):
    """Stop the launch if `controller` is not a known controller.

    Without this, a typo (e.g. controller:=purepursuit) matches neither
    IfCondition below: Gazebo, YOLO and the planner start, but no node
    publishes /drive and the car just stands still without any error.
    """
    value = LaunchConfiguration('controller').perform(context)
    if value not in CONTROLLERS:
        raise RuntimeError(
            f"unknown controller '{value}': use controller:=pursuit or controller:=pid")
    return []


def generate_launch_description():
    controller = LaunchConfiguration('controller')
    rviz = LaunchConfiguration('rviz')

    gazebo_pkg = Path(get_package_share_directory('roboracer_gazebo'))
    planning_pkg = Path(get_package_share_directory('roboracer_planning'))
    params = str(planning_pkg / 'config' / 'planning_params.yaml')
    rviz_cfg = str(planning_pkg / 'rviz' / 'cone_planning.rviz')

    # Every node on sim time, otherwise stamp ages are meaningless.
    sim_time = {'use_sim_time': True}

    # ----- stage 1: Gazebo + car -----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_pkg / 'launch' / 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    # ----- stage 2: perception -----
    # R9 - Lighting: on the shaded side of a corner, yellow cones score only
    # 0.3-0.5, so yolo_node's default yellow gate (0.5) drops cones that are
    # clearly visible. Use the same gate as blue; R4 (track-width check in
    # delaunay_node) still rejects false pairs.
    yolo = Node(package='roboracer_perception', executable='yolo_node',
                name='yolo_cone_debug_node', output='screen',
                parameters=[sim_time, {'yellow_conf': LaunchConfiguration('yellow_conf')}])

    # ----- stage 3: planning + control -----
    # Exactly one controller starts, chosen by the `controller` argument
    # (validated by check_controller). Both publish /drive.
    delaunay = Node(package='roboracer_planning', executable='delaunay_node',
                    name='delaunay_node', output='screen', parameters=[params, sim_time])

    pid = Node(package='roboracer_planning', executable='centerline_pid_node',
               name='centerline_pid_node', output='screen', parameters=[params, sim_time],
               condition=IfCondition(PythonExpression(["'", controller, "' == 'pid'"])))

    pursuit = Node(package='roboracer_planning', executable='waypoint_pursuit_node',
                   name='waypoint_pursuit_node', output='screen', parameters=[params, sim_time],
                   condition=IfCondition(PythonExpression(["'", controller, "' == 'pursuit'"])))

    # Simulation only: compensate Ignition's ~1.05 s steering lag, then
    # /drive_sim -> /cmd_vel. Controllers publish /drive exactly as on the car.
    lag_comp = Node(package='roboracer_drive_to_cmd_vel', executable='steering_lag_compensator',
                    name='steering_lag_compensator', output='screen',
                    parameters=[sim_time, {'enabled': LaunchConfiguration('lag_comp'),
                                           'boost': LaunchConfiguration('lag_boost')}])
    bridge = Node(package='roboracer_drive_to_cmd_vel', executable='drive_to_cmd_vel',
                  name='drive_to_cmd_vel', output='screen',
                  parameters=[sim_time, {'drive_topic': '/drive_sim'}])

    # ----- stage 4: visualisation -----
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2',
                     arguments=['-d', rviz_cfg], parameters=[sim_time],
                     condition=IfCondition(rviz))

    # ----- the chain -----
    wait_camera = wait_for('/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo',
                           'wait_for_camera')
    wait_cones = wait_for('/cones/blue', 'geometry_msgs/msg/PoseArray', 'wait_for_cones')
    wait_waypoints = wait_for('/waypoints', 'geometry_msgs/msg/PoseArray', 'wait_for_waypoints')

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='pursuit',
                              description="Controller: 'pursuit' (recommended) or 'pid'."),
        DeclareLaunchArgument('world', default_value='cones',
                              description="World: 'cones', 'walls' or a .world path."),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('lag_comp', default_value='auto',
                              description="Steering-lag compensation: 'auto' = on in simulation "
                                          "(use_sim_time), 'false' = passthrough."),
        DeclareLaunchArgument('lag_boost', default_value='3.0',
                              description='Compensation strength: effective lag = 1.05 s / boost.'),
        DeclareLaunchArgument('yellow_conf', default_value='0.3',
                              description='yolo_node publish gate for yellow cones (R9).'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y.'),
        DeclareLaunchArgument('z', default_value='0.1', description='Spawn z.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw.'),

        OpaqueFunction(function=check_controller),   # before anything starts
        gazebo,
        LogInfo(msg='[stage 1] Gazebo starting; waiting for the camera...'),
        wait_camera,
        then(wait_camera, '[stage 2] camera is up: starting yolo_node; waiting for cones...',
             [yolo, wait_cones]),
        then(wait_cones, '[stage 3] cones detected: starting planner + controller',
             [delaunay, pid, pursuit, lag_comp, bridge, wait_waypoints]),
        then(wait_waypoints, '[stage 4] waypoints flowing: starting RViz', [rviz_node]),
    ])
