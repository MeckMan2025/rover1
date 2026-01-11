"""
Dog Follower Launch File

Launches the dog_follower node with configurable parameters.
Can be included in rover.launch.py or run standalone for testing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('rover1_vision')

    # Launch arguments
    model_path = LaunchConfiguration('model_path',
        default='/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef')
    confidence = LaunchConfiguration('confidence_threshold', default='0.5')
    linear_speed = LaunchConfiguration('linear_speed', default='0.2')
    angular_speed = LaunchConfiguration('angular_speed', default='0.5')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef',
            description='Path to YOLOv8 HEF model file'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Minimum confidence for dog detection (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.2',
            description='Maximum linear following speed (m/s)'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.5',
            description='Maximum angular following speed (rad/s)'
        ),

        Node(
            package='rover1_vision',
            executable='dog_follower',
            name='dog_follower',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'confidence_threshold': confidence,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'target_box_area_ratio': 0.15,
                'center_tolerance': 0.1,
                'detection_timeout': 1.0,
                'teleop_override_timeout': 0.5,
            }]
        ),
    ])
