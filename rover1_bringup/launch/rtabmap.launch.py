"""
RTAB-Map Visual SLAM Launch File for Rover1 Indoor Autonomy

This launch file configures RTAB-Map for RGB-D SLAM using the HP60C camera.
Designed for Phase 6: Indoor Autonomy Demo (GPS-denied environments).

Camera Topics (from ascamera_node):
  - RGB: /ascamera_hp60c/camera_publisher/rgb0/image
  - Depth: /ascamera_hp60c/camera_publisher/depth0/image_raw
  - RGB Info: /ascamera_hp60c/camera_publisher/rgb0/camera_info
  - Depth Info: /ascamera_hp60c/camera_publisher/depth0/camera_info

Output Topics:
  - /map (OccupancyGrid for Nav2)
  - /rtabmap/odom (Visual odometry)
  - /rtabmap/mapData (Full map data for save/load)

Usage:
  ros2 launch rover1_bringup rtabmap.launch.py
  ros2 launch rover1_bringup rtabmap.launch.py localization:=true  # Use saved map
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Launch arguments
    localization = LaunchConfiguration('localization', default='false')
    use_viz = LaunchConfiguration('use_viz', default='false')  # Disabled by default (needs display)
    database_path = LaunchConfiguration('database_path',
                                         default='/home/andrewmeckley/maps/rtabmap.db')

    # Common parameters for RTAB-Map
    rtabmap_parameters = {
        # Frame configuration
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',

        # Database
        'database_path': database_path,

        # Detection and loop closure
        'Rtabmap/DetectionRate': '2.0',  # Hz - balance CPU vs responsiveness
        'Kp/MaxFeatures': '400',  # Feature detection limit
        'Kp/DetectorStrategy': '6',  # ORB features (fast, good for indoor)

        # Memory management for Pi 5
        'Rtabmap/MemoryThr': '300',  # Max nodes in working memory
        'Mem/RehearsalSimilarity': '0.6',

        # Visual odometry
        'Odom/Strategy': '0',  # Frame-to-map (more accurate)
        'Odom/GuessMotion': 'true',
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1',

        # Grid map for Nav2
        'Grid/FromDepth': 'true',
        'Grid/RangeMax': '4.0',  # 4 meter range for indoor
        'Grid/RangeMin': '0.3',  # Ignore too-close objects
        'Grid/CellSize': '0.05',  # 5cm resolution
        'Grid/MaxGroundHeight': '0.1',  # Ground plane threshold
        'Grid/MaxObstacleHeight': '1.5',  # Max obstacle height
        'Grid/NormalsSegmentation': 'true',

        # Optimizer
        'Optimizer/Strategy': '1',  # g2o
        'Optimizer/Iterations': '10',

        # RGBD parameters
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/AngularUpdate': '0.1',  # Radians
        'RGBD/LinearUpdate': '0.1',  # Meters
        'RGBD/OptimizeFromGraphEnd': 'false',
    }

    # Topic remappings for HP60C camera
    remappings = [
        ('rgb/image', '/ascamera_hp60c/camera_publisher/rgb0/image'),
        ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
        ('depth/image', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
        ('depth/camera_info', '/ascamera_hp60c/camera_publisher/depth0/camera_info'),
    ]

    return LaunchDescription([
        # Use CycloneDDS instead of FastRTPS to avoid shared memory issues on Pi 5
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        # Arguments
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Set to true to use existing map (localization mode)'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='/home/andrewmeckley/maps/rtabmap.db',
            description='Path to RTAB-Map database file'
        ),
        DeclareLaunchArgument(
            'use_viz',
            default_value='false',
            description='Enable RTAB-Map visualization (requires display)'
        ),

        # Static transform: Bridge URDF camera frame to driver's frame
        # URDF defines: camera_optical_frame
        # Camera driver publishes with: ascamera_hp60c_ascamera_0
        # This identity transform links them so RTAB-Map can find the TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_bridge',
            arguments=['0', '0', '0', '0', '0', '0',
                       'camera_optical_frame', 'ascamera_hp60c_ascamera_0'],
        ),

        # RTAB-Map SLAM Node (Mapping Mode)
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters, {
                'Mem/IncrementalMemory': 'true',  # Build map
                'Mem/InitWMWithAllNodes': 'false',
            }],
            remappings=remappings,
            arguments=['--delete_db_on_start'],  # Fresh map each time in mapping mode
        ),

        # RTAB-Map Localization Node (Use existing map)
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters, {
                'Mem/IncrementalMemory': 'false',  # Don't add to map
                'Mem/InitWMWithAllNodes': 'true',  # Load full map
            }],
            remappings=remappings,
        ),

        # Visual Odometry Node
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'wait_for_transform_duration': 0.2,
                'approx_sync': True,
                'queue_size': 10,
                # Visual odometry parameters
                'Odom/Strategy': '0',
                'Odom/GuessMotion': 'true',
                'Vis/MinInliers': '15',
                'Vis/InlierDistance': '0.1',
                'Odom/ResetCountdown': '1',
                'Vis/FeatureType': '6',  # ORB
                'Vis/MaxFeatures': '500',
            }],
            remappings=remappings,
        ),

        # RTAB-Map Visualization (optional - requires display, disabled by default)
        Node(
            condition=IfCondition(use_viz),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=remappings,
            arguments=['-d'],  # Start with default config
        ),
    ])
