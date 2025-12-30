"""
Nav2 Navigation Launch File for Rover1 Indoor Autonomy

Launches the Nav2 stack configured for indoor navigation using RTAB-Map.
Designed for Phase 6: Indoor Autonomy Demo.

Prerequisites:
  - RTAB-Map must be running (provides /map and localization)
  - rover.launch.py provides base transforms and hardware
  - CycloneDDS: sudo apt install ros-jazzy-rmw-cyclonedds-cpp

Usage:
  ros2 launch rover1_bringup nav2.launch.py

With RTAB-Map (full indoor autonomy):
  Terminal 1: ros2 launch rover1_bringup rover.launch.py
  Terminal 2: ros2 launch rover1_bringup rtabmap.launch.py
  Terminal 3: ros2 launch rover1_bringup nav2.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('rover1_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')

    # Default params file path
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    return LaunchDescription([
        # Use CycloneDDS instead of FastRTPS to avoid shared memory issues on Pi 5
        # FastRTPS SHM transport fails with stale lock files after crashes
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start Nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to Nav2 parameters file'
        ),

        # Nav2 Controller Server (includes local costmap)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 Planner Server (includes global costmap)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 Behavior Server (recoveries)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 BT Navigator - DISABLED (Jazzy/ARM64 issues)
        # Using nav2_simple_commander instead for goal sending
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[params_file, {'use_sim_time': use_sim_time}],
        # ),

        # Nav2 Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 Lifecycle Manager - manages all Nav2 servers
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 15.0,  # Generous timeout for Pi 5
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    # 'bt_navigator',  # Disabled - using simple_commander
                    'waypoint_follower',
                    'velocity_smoother',
                ]
            }],
        ),
    ])
