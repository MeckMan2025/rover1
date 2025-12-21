from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
        Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            output='screen',
            parameters=[{
                'host': ntrip_host,
                'port': ntrip_port,
                'mountpoint': ntrip_mountpoint,
                'username': ntrip_user,
                'password': ntrip_pass,
                'rtcm_topic': '/rtcm' 
            }]
        ),

        # U-Blox Driver
        # Consumes RTCM from /rtcm (via internally remapped topic if needed, usually 'rtcm')
        # ZED-F9R default baud 38400 works via USB, but often 460800 or 115200 is negotiated
        Node(
            package='ublox_dgnss_node',
            executable='ublox_dgnss_node',
            name='ublox_dgnss',
            output='screen',
            parameters=[{
                'device_serial_string': '', # Auto-detect or specify if multiple are present
                'frame_id': 'gps_link',
                # Config to accept RTCM integration
            }],
            # Remap to ensure the driver hears the corrections
            remappings=[
                ('rtcm', '/rtcm')
            ]
        )
    ])
