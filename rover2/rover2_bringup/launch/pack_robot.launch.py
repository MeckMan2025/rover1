"""
Pack Robot Launch File - Minimal Rover2 Configuration

This is the minimal launch configuration for Rover2 "Pack Robot" mode.
Only includes essential components for person following functionality:
- Motor control (hiwonder_driver + kinematics)  
- Camera (ascamera hp60c)
- Person follower (shoe_follower)
- Battery monitoring
- Web dashboard with teleop control
- Basic robot state publisher

Excluded: SLAM, GPS/RTK, Nav2, patrol system, EKF, GNSS monitoring
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('rover2_bringup')
    desc_share = FindPackageShare('rover1_description')
    
    # Launch Arguments
    enable_person_follower = LaunchConfiguration('enable_person_follower', default='false')
    
    # Process URDF (use rover2 description)
    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'rover.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    # FastRTPS config to disable shared memory transport
    fastrtps_config = PathJoinSubstitution([pkg_share, 'config', 'fastrtps_no_shm.xml'])

    return LaunchDescription([
        # Use FastRTPS with shared memory disabled
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_config),

        DeclareLaunchArgument(
            'enable_person_follower',
            default_value='false',
            description='Whether to start the Person Follower vision node (safety: disabled by default)'
        ),

        # Robot State Publisher (minimal TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Hardware Drivers - Essential Only
        Node(
            package='rover2_hardware',
            executable='hiwonder_driver',
            name='motor_driver',
            output='screen',
            parameters=[{
                'i2c_bus': 1, 
                'i2c_address': 0x34, 
                'invert_fl': True,
                'invert_rr': True
            }]
        ),
        Node(
            package='rover2_hardware',
            executable='mecanum_kinematics',
            name='kinematics',
            output='screen'
        ),
        
        # Battery Monitor (safety critical)
        Node(
            package='rover2_hardware',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[{'i2c_bus': 1, 'i2c_address': 0x34, 'publish_rate': 1.0}],
            output='screen'
        ),
        
        # Camera Integration (Nuwa-HP60C) - Required for person following
        Node(
            package='ascamera',
            executable='ascamera_node',
            name='camera_publisher',
            namespace='ascamera_hp60c',
            output='screen',
            parameters=[{
                'confiPath': '/home/andrewmeckley/ros2_ws/src/ascamera/configurationfiles',
                'fps': 10,
                'depth_width': 640,
                'depth_height': 480,
                'rgb_width': 640,
                'rgb_height': 480,
                'pub_tfTree': False
            }]
        ),

        # Person Follower (Pack Robot core functionality)
        Node(
            condition=IfCondition(enable_person_follower),
            package='rover2_vision',
            executable='person_follower',
            name='person_follower',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'model_path': '/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef',
                'confidence_threshold': 0.5,
                'linear_speed': 0.4,
                'angular_speed': 0.8,
                'target_foot_y_ratio': 0.70,
                'too_close_foot_y_ratio': 0.85,
                'center_tolerance': 0.12,
                'detection_timeout': 2.0,
                'teleop_override_timeout': 0.5,
                'recovery_scan_timeout': 4.0,
            }]
        ),
        
        # Pack Robot Dashboard (includes teleop control + person follower UI)
        Node(
            package='rover2_dashboard',
            executable='pack_robot_dashboard',
            name='pack_robot_dashboard',
            output='screen'
        ),
    ])