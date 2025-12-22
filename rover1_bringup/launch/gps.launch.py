from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Credentials from IARTN-Credentials.md
    ntrip_user = LaunchConfiguration('ntrip_user', default='grease454')
    ntrip_pass = LaunchConfiguration('ntrip_pass', default='nacceb-xekva6-cuTbux')
    ntrip_host = LaunchConfiguration('ntrip_host', default='165.206.203.10')
    ntrip_port = LaunchConfiguration('ntrip_port', default='10000')
    ntrip_mountpoint = LaunchConfiguration('ntrip_mountpoint', default='RTCM3_IMAX')

    return LaunchDescription([
        # NTRIP Client
        # Publishes RTCM corrections to /rtcm
        # NTRIP Client
        # Publishes RTCM corrections to /rtcm
        Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            output='screen',
            parameters=[{
                'host': ntrip_host,
                'port': ntrip_port,
                'mountpoint': ntrip_mountpoint,
                'authenticate': True,
                'username': ntrip_user,
                'password': ntrip_pass,
                'rtcm_topic': '/ntrip_client/rtcm',
                'rtcm_message_package': 'rtcm_msgs'
            }]
        ),

        # U-Blox Driver & Converter (Standard Mode)
        # Includes both the driver (for raw UBX) and the converter (UBX -> NavSatFix).
        # Usage: Host internal topic is /ntrip_client/rtcm, so we match that.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ublox_dgnss'),
                    'launch',
                    'ublox_rover_hpposllh_navsatfix.launch.py'
                ])
            ]),
            launch_arguments={
                'device_family': 'F9R',
                'frame_id': 'gps_link',
                # 'log_level': 'DEBUG' # Uncomment for troubleshooting
            }.items()
        )
    ])
