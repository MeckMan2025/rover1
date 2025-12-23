from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('rover1_bringup')
    desc_share = FindPackageShare('rover1_description')
    
    # Process URDF
    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'rover.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
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
        
        # Include GPS Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'gps.launch.py'])
            ])
        )
    ])
