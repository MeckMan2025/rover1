"""
Shoe Follower Launch File

Launches the shoe_follower node for person tracking and following.
Uses foot position (bottom of bounding box) to estimate distance.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    model_path = LaunchConfiguration('model_path',
        default='/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef')
    confidence = LaunchConfiguration('confidence_threshold', default='0.5')
    linear_speed = LaunchConfiguration('linear_speed', default='0.4')
    angular_speed = LaunchConfiguration('angular_speed', default='0.8')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef',
            description='Path to YOLOv8 HEF model file'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Minimum confidence for person detection (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.4',
            description='Maximum linear following speed (m/s)'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.8',
            description='Maximum angular following speed (rad/s)'
        ),

        Node(
            package='rover1_vision',
            executable='shoe_follower',
            name='shoe_follower',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'confidence_threshold': confidence,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'target_foot_y_ratio': 0.70,
                'too_close_foot_y_ratio': 0.85,
                'center_tolerance': 0.12,
                'detection_timeout': 2.0,
                'teleop_override_timeout': 0.5,
                'recovery_scan_timeout': 4.0,
            }]
        ),
    ])
