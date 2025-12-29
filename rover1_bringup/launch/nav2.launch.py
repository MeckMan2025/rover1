"""
Nav2 Navigation Launch File for Rover1 Indoor Autonomy

Launches the Nav2 stack configured for indoor navigation using RTAB-Map.
Designed for Phase 6: Indoor Autonomy Demo.

Prerequisites:
  - RTAB-Map must be running (provides /map and localization)
  - rover.launch.py provides base transforms and hardware

Usage:
  ros2 launch rover1_bringup nav2.launch.py

With RTAB-Map (full indoor autonomy):
  Terminal 1: ros2 launch rover1_bringup rover.launch.py
  Terminal 2: ros2 launch rover1_bringup rtabmap.launch.py
  Terminal 3: ros2 launch rover1_bringup nav2.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('rover1_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')

    # Default params file path
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    return LaunchDescription([
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

        # Nav2 Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')]  # Separate from teleop
        ),

        # Nav2 Planner Server
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

        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

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
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')
            ]
        ),

        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 10.0,  # Increased for Pi 5
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ]
            }],
        ),

        # Local Costmap Node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav')
            ]
        ),

        # Global Costmap Node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # Costmap Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 10.0,  # Increased for Pi 5
                'node_names': [
                    'local_costmap',
                    'global_costmap',
                ]
            }],
        ),

        # Note: twist_mux removed for now - will add back with mission controller
        # For testing, Nav2 outputs directly to cmd_vel via velocity_smoother
    ])
