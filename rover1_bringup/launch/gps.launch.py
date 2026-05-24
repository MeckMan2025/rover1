from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ublox_pkg = FindPackageShare('ublox_dgnss')
    bringup_pkg = FindPackageShare('rover1_bringup')
    
    ublox_config = PathJoinSubstitution([bringup_pkg, 'config', 'ublox_gps.yaml'])
    ublox_launch = PathJoinSubstitution([ublox_pkg, 'launch', 'ublox_rover_hpposllh_navsatfix.launch.py'])

    ublox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ublox_launch),
        launch_arguments={
            'device_family': 'F9R',
            'frame_id': 'gps_link',
            'params_file': ublox_config
        }.items()
    )

    nmea_node = Node(
        package='rover1_hardware',
        executable='fix_to_nmea',
        name='fix_to_nmea',
        output='screen'
    )

    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        name='ntrip_client',
        output='screen',
        parameters=[{
            'host': '165.206.203.10',
            'port': 10000,
            'mountpoint': 'RTCM3_IMAX',
            'authenticate': True,
            'username': 'grease454',
            'password': 'nacceb-xekva6-cuTbux',
            'rtcm_message_package': 'rtcm_msgs',
            'rtcm_topic': '/ntrip_client/rtcm'
        }],
        remappings=[('/rtcm', '/ntrip_client/rtcm')]
    )

    return LaunchDescription([ublox_node, nmea_node, ntrip_node])
