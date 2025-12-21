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
            parameters=[{'i2c_bus': 1, 'i2c_address': 0x34, 'invert_fl': True}]
        ),
        Node(
            package='rover1_hardware',
            executable='mecanum_kinematics',
            name='kinematics',
            output='screen'
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
