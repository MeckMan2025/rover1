"""
Lightweight Patrol Launch - No RTAB-Map, No Nav2

Launches the minimal stack for teach-and-repeat patrol:
  - Base rover (motors, encoders, IMU, teleop)
  - Simple waypoint follower (odometry-based)
  - Waypoint recorder
  - Web dashboard

Use this when:
  - CPU is limited (Pi 5 without heavy workloads)
  - Indoor environment with clear paths
  - You don't need SLAM or obstacle avoidance

Usage:
  ros2 launch rover1_bringup patrol_lite.launch.py

For outdoor GPS patrol, also launch GPS nodes:
  ros2 launch rover1_bringup gps.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('rover1_bringup')
    patrol_dir = get_package_share_directory('rover1_patrol')
    dashboard_dir = get_package_share_directory('gnss_web_dashboard')

    # Paths directory
    paths_dir = os.path.expanduser('~/paths')

    # FastRTPS config (disable SHM for stability)
    fastrtps_config = os.path.join(bringup_dir, 'config', 'fastrtps_no_shm.xml')

    return LaunchDescription([
        # Environment setup
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_config),

        # Include base rover launch (motors, encoders, IMU, teleop)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'rover.launch.py')
            )
        ),

        # Waypoint Recorder (for recording paths)
        Node(
            package='rover1_patrol',
            executable='waypoint_recorder.py',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'min_distance': 0.30,
                'paths_directory': paths_dir,
                'tf_timeout': 0.5,
                'odom_topic': '/odom/wheel_odom'
            }]
        ),

        # Simple Waypoint Follower (odometry-based, lightweight)
        Node(
            package='rover1_patrol',
            executable='simple_waypoint_follower.py',
            name='simple_waypoint_follower',
            output='screen',
            parameters=[{
                'paths_directory': paths_dir,
                'waypoint_tolerance': 0.20,
                'max_linear_speed': 0.3,
                'max_angular_speed': 0.8,
                'use_odom_frame': True
            }]
        ),

        # GPS Waypoint Follower (for outdoor mode)
        Node(
            package='rover1_patrol',
            executable='gps_waypoint_follower.py',
            name='gps_waypoint_follower',
            output='screen',
            parameters=[{
                'paths_directory': paths_dir,
                'waypoint_tolerance': 1.0,
                'max_linear_speed': 0.4,
                'max_angular_speed': 0.8
            }]
        ),

        # Web Dashboard
        Node(
            package='gnss_web_dashboard',
            executable='web_dashboard',
            name='rover1_web_dashboard',
            output='screen'
        ),
    ])
