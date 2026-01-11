#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch person follower node for rover2."""
    
    # Launch arguments
    confidence_threshold = LaunchConfiguration('confidence_threshold', default='0.5')
    linear_speed = LaunchConfiguration('linear_speed', default='0.4')
    angular_speed = LaunchConfiguration('angular_speed', default='0.8')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Person detection confidence threshold (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'linear_speed', 
            default_value='0.4',
            description='Maximum linear speed for following (m/s)'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.8', 
            description='Maximum angular speed for following (rad/s)'
        ),

        # Person Follower Node
        Node(
            package='rover2_vision',
            executable='person_follower',
            name='person_follower',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'model_path': '/home/andrewmeckley/ros2_ws/src/rover2/models/yolov8s.hef',
                'confidence_threshold': confidence_threshold,
                'target_foot_y_ratio': 0.70,
                'too_close_foot_y_ratio': 0.85,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'center_tolerance': 0.12,
                'detection_timeout': 2.0,
                'teleop_override_timeout': 0.5,
                'recovery_scan_timeout': 4.0,
            }]
        )
    ])