from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover1_hardware',
            executable='berry_imu_driver',
            name='imu_driver',
            output='screen',
            parameters=[{'i2c_bus': 1}]
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
        
        # EKF Sensor Fusion (Local: Odom -> Base_Link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('rover1_bringup'),
                'config',
                'ekf.yaml'
            ])],
            remappings=[
                ('odometry/filtered', 'odometry/local')
            ]
        ),

        # EKF Sensor Fusion (Global: Map -> Odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('rover1_bringup'),
                'config',
                'ekf_global.yaml'
            ])],
            remappings=[
                ('odometry/filtered', 'odometry/global')
            ]
        ),

        # GPS Integration (NavSat Transform)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('rover1_bringup'),
                'config',
                'navsat.yaml'
            ])],
            remappings=[
                ('gps/fix', '/fix'),
                ('imu/data', '/imu/data'),
                ('odometry/filtered', 'odometry/local'),
                ('odometry/gps', 'odometry/gps')
            ]
        ),
        
        # Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments = ['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments = ['0', '0', '0.2', '0', '0', '0', 'base_link', 'gps_link']
        ),

        # Include GPS Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rover1_bringup'),
                    'launch',
                    'gps.launch.py'
                ])
            ])
        )
    ])
