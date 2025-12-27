from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('rover1_bringup')
    desc_share = FindPackageShare('rover1_description')
    
    # Launch Arguments
    use_joy = LaunchConfiguration('use_joy', default='true')
    use_foxglove = LaunchConfiguration('use_foxglove', default='true')

    # Process URDF
    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'rover.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Whether to start the joystick/teleop nodes'
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Whether to start the Foxglove Bridge'
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
            package='rover1_hardware',
            executable='berry_imu_driver',
            name='imu_driver',
            output='screen',
            parameters=[{'i2c_bus': 1, 'frame_id': 'imu_link'}]
        ),
        Node(
            package='rover1_hardware',
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
            package='rover1_hardware',
            executable='mecanum_kinematics',
            name='kinematics',
            output='screen'
        ),
        
        # Isolated Battery Monitor (Decoupled per Engineering Strategy)
        Node(
            package='rover1_hardware',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[{'i2c_bus': 1, 'i2c_address': 0x34, 'publish_rate': 1.0}],
            output='screen'
        ),
        
        # GNSS Health Monitor (Aggregates GPS/RTK/NTRIP status)
        Node(
            package='gnss_health_monitor',
            executable='gnss_health_monitor_node',
            name='gnss_health_monitor_node',
            output='screen'
        ),
        
        # GNSS Web Dashboard (Simple web interface for GPS monitoring)
        Node(
            package='gnss_web_dashboard', 
            executable='gnss_web_dashboard',
            name='gnss_web_dashboard',
            output='screen'
        ),

        # Camera Integration (Nuwa-HP60C)
        Node(
            package='ascamera',
            executable='ascamera_node',
            name='ascamera_node',
            namespace='ascamera_hp60c',
            output='screen',
            parameters=[{
                'confiPath': '/home/andrewmeckley/ros2_ws/src/ascamera/configurationfiles',
                'fps': 10,
                'rgb_width': 640,
                'rgb_height': 480,
                'pub_tfTree': False # We handle TF in our own URDF for better control
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
        
        # Foxglove Bridge (Web-based Viz)
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
                'send_buffer_limit': 100000000  # Give it 100MB of breathing room
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

        # Stadia Custom Teleop (Safe-Mecanum)
        Node(
            condition=IfCondition(use_joy),
            package='rover1_hardware',
            executable='stadia_teleop',
            name='stadia_teleop',
            output='screen',
            parameters=[{
                'max_linear_speed': 2.0,
                'max_angular_speed': 4.0,
                'deadman_threshold': 0.0,
                'debug_axes': False  # Set True to log raw axis values for tuning
            }]
        ),
        
        # Include GPS Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'gps.launch.py'])
            ])
        )
    ])
