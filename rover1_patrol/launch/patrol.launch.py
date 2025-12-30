"""
Patrol System Launch File

Launches the Teach & Repeat patrol nodes:
  - waypoint_recorder: Records waypoints during manual driving
  - map_snapshot: Generates map images with waypoint overlays
  - pointcloud_exporter: Exports 3D point clouds for web visualization
  - patrol_manager: Manages patrol execution
  - velocity_scaler: Scales velocities based on patrol speed setting

Prerequisites:
  - rover.launch.py running (base hardware + teleop)
  - rtabmap.launch.py running (SLAM + map)
  - nav2.launch.py running (navigation stack)

Usage:
  ros2 launch rover1_patrol patrol.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Parameters
    paths_dir = os.path.expanduser('~/paths')

    return LaunchDescription([
        DeclareLaunchArgument(
            'paths_directory',
            default_value=paths_dir,
            description='Directory to store patrol paths'
        ),

        # Waypoint Recorder
        Node(
            package='rover1_patrol',
            executable='waypoint_recorder.py',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'min_distance': 0.30,  # 30cm = rover length
                'paths_directory': LaunchConfiguration('paths_directory'),
                'tf_timeout': 0.5
            }]
        ),

        # Map Snapshot
        Node(
            package='rover1_patrol',
            executable='map_snapshot.py',
            name='map_snapshot',
            output='screen',
            parameters=[{
                'paths_directory': LaunchConfiguration('paths_directory')
            }]
        ),

        # Point Cloud Exporter (for 3D web visualization)
        Node(
            package='rover1_patrol',
            executable='pointcloud_exporter.py',
            name='pointcloud_exporter',
            output='screen',
            parameters=[{
                'paths_directory': LaunchConfiguration('paths_directory'),
                'voxel_size': 0.05,  # 5cm voxel grid for downsampling
                'max_points': 200000  # Limit for browser performance
            }]
        ),

        # Patrol Manager
        Node(
            package='rover1_patrol',
            executable='patrol_manager.py',
            name='patrol_manager',
            output='screen',
            parameters=[{
                'paths_directory': LaunchConfiguration('paths_directory'),
                'stuck_timeout': 30.0
            }]
        ),

        # Note: velocity_scaler is launched in nav2.launch.py
        # It always runs and passes through at 100% when patrol not active
    ])
