from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('rover2_bringup')
    desc_share = FindPackageShare('rover2_description')
    
    # Launch Arguments
    use_joy = LaunchConfiguration('use_joy', default='true')
    use_foxglove = LaunchConfiguration('use_foxglove', default='false')
    enable_person_follower = LaunchConfiguration('enable_person_follower', default='true')

    # Process URDF
    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'rover.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    # FastRTPS config to disable shared memory transport
    fastrtps_config = PathJoinSubstitution([pkg_share, 'config', 'fastrtps_no_shm.xml'])

    return LaunchDescription([
        # Use FastRTPS with shared memory disabled
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_config),

        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Whether to start the joystick/teleop nodes'
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='false',
            description='Whether to start the Foxglove Bridge'
        ),
        DeclareLaunchArgument(
            'enable_person_follower',
            default_value='true',
            description='Whether to start the Person Follower vision node'
        ),

        # Robot State Publisher (TF Tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Hardware Drivers
        Node(
            package='rover2_hardware',
            executable='berry_imu_driver',
            name='imu_driver',
            output='screen',
            parameters=[{'i2c_bus': 1, 'frame_id': 'imu_link'}]
        ),
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
        
        # Battery Monitor
        Node(
            package='rover2_hardware',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[{'i2c_bus': 1, 'i2c_address': 0x34, 'publish_rate': 1.0}],
            output='screen'
        ),
        
        # GNSS Health Monitor
        Node(
            package='gnss_health_monitor',
            executable='gnss_health_monitor_node',
            name='gnss_health_monitor_node',
            output='screen',
            parameters=[{
                'battery_topic': '/battery/voltage'
            }]
        ),
        
        # Rover2 Web Dashboard
        Node(
            package='rover2_dashboard', 
            executable='rover2_web_dashboard',
            name='rover2_web_dashboard',
            output='screen'
        ),

        # Camera Integration (Nuwa60C)
        Node(
            package='ascamera',
            executable='ascamera_node',
            name='camera_publisher',
            namespace='ascamera_hp60c',
            output='screen',
            parameters=[{
                'confiPath': '/home/andrewmeckley/ros2_ws/src/rover1/Camera_Specs/ascam_ros2_ws/src/ascamera/configurationfiles',
                'fps': 10,
                'depth_width': 640,
                'depth_height': 480,
                'rgb_width': 640,
                'rgb_height': 480,
                'pub_tfTree': False
            }]
        ),

        # Person Follower (Main feature for Rover2)
        Node(
            condition=IfCondition(enable_person_follower),
            package='rover2_vision',
            executable='person_follower',
            name='person_follower',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'model_path': '/home/andrewmeckley/ros2_ws/src/rover2/models/yolov8s.hef',
                'confidence_threshold': 0.5,
                'target_foot_y_ratio': 0.70,
                'too_close_foot_y_ratio': 0.85,
                'linear_speed': 0.4,
                'angular_speed': 0.8,
                'center_tolerance': 0.12,
                'detection_timeout': 2.0,
                'teleop_override_timeout': 0.5,
                'recovery_scan_timeout': 4.0
            }]
        ),

        # EKF Sensor Fusion (Local: Odom -> Base_Link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),

        # EKF Sensor Fusion (Global: Map -> Odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf_global.yaml'])],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),

        # GPS Integration (NavSat Transform)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'navsat.yaml'])],
            remappings=[
                ('gps/fix', '/fix'),
                ('imu/data', '/imu/data'),
                ('odometry/filtered', 'odometry/local'),
                ('odometry/gps', 'odometry/gps')
            ]
        ),
        
        # Foxglove Bridge (Optional)
        Node(
            condition=IfCondition(use_foxglove),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'tls': False,
                'use_compression': True,
                'send_buffer_limit': 100000000
            }]
        ),
        
        # Joystick Driver
        Node(
            condition=IfCondition(use_joy),
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'deadzone': 0.1, 'autorepeat_rate': 20.0}]
        ),

        
        # Include GPS Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'gps.launch.py'])
            ])
        )
    ])