import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config=os.path.join(get_package_share_directory('roboracer_lidar_centerline'),'config','lidar_centerline.yaml')
    scan=LaunchConfiguration('scan_topic'); cmd=LaunchConfiguration('cmd_vel_topic')
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic',default_value='/scan'),DeclareLaunchArgument('cmd_vel_topic',default_value='/cmd_vel'),
        Node(package='roboracer_lidar_centerline',executable='wall_centerline',name='wall_centerline_node',output='screen',parameters=[config,{'scan_topic':scan}]),
        Node(package='roboracer_lidar_centerline',executable='pure_pursuit',name='pure_pursuit_node',output='screen',parameters=[config,{'command_topic':cmd}]),
    ])
