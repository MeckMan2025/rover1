#!/usr/bin/env python3
"""
GNSS Health Monitor Launch File

Launches the GNSS health monitoring node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for GNSS Health Monitor"""
    
    return LaunchDescription([
        # Launch arguments for configuration
        DeclareLaunchArgument(
            'navsat_topic',
            default_value='/gps/filtered',
            description='Primary NavSat topic'
        ),
        
        DeclareLaunchArgument(
            'navsat_fallback_topic', 
            default_value='/fix',
            description='Fallback NavSat topic'
        ),
        
        DeclareLaunchArgument(
            'navsat_timeout_s',
            default_value='2.0',
            description='NavSat timeout in seconds'
        ),
        
        DeclareLaunchArgument(
            'ubx_nav_sat_topic',
            default_value='/ubx_nav_sat',
            description='u-blox NavSAT topic for satellite info'
        ),
        
        DeclareLaunchArgument(
            'rtcm_topic',
            default_value='/ntrip_client/rtcm',
            description='Primary RTCM corrections topic'
        ),
        
        DeclareLaunchArgument(
            'rtcm_fallback_topic',
            default_value='/rtcm', 
            description='Fallback RTCM corrections topic'
        ),
        
        DeclareLaunchArgument(
            'rtcm_timeout_s',
            default_value='5.0',
            description='RTCM timeout for connection status'
        ),
        
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='5.0',
            description='Health message publish rate'
        ),
        
        DeclareLaunchArgument(
            'rtk_fixed_h_acc_threshold_m',
            default_value='0.05',
            description='Horizontal accuracy threshold for RTK FIXED (meters)'
        ),
        
        DeclareLaunchArgument(
            'rtk_float_h_acc_threshold_m', 
            default_value='0.30',
            description='Horizontal accuracy threshold for RTK FLOAT (meters)'
        ),
        
        DeclareLaunchArgument(
            'sat_used_flag_mask',
            default_value='8',  # 0x08 
            description='Bitmask for satellite used-in-solution flag'
        ),
        
        DeclareLaunchArgument(
            'use_ublox_carrsoln_if_available',
            default_value='true',
            description='Use u-blox carrier solution status if available'
        ),
        
        # Log launch configuration
        LogInfo(
            msg=PythonExpression([
                "'Starting GNSS Health Monitor with primary NavSat topic: ", 
                LaunchConfiguration('navsat_topic'), 
                "'"
            ])
        ),
        
        # GNSS Health Monitor Node
        Node(
            package='gnss_health_monitor',
            executable='gnss_health_monitor_node',
            name='gnss_health_monitor_node',
            output='screen',
            parameters=[{
                'navsat_topic': LaunchConfiguration('navsat_topic'),
                'navsat_fallback_topic': LaunchConfiguration('navsat_fallback_topic'),
                'navsat_timeout_s': LaunchConfiguration('navsat_timeout_s'),
                'ubx_nav_sat_topic': LaunchConfiguration('ubx_nav_sat_topic'),
                'rtcm_topic': LaunchConfiguration('rtcm_topic'),
                'rtcm_fallback_topic': LaunchConfiguration('rtcm_fallback_topic'),
                'rtcm_timeout_s': LaunchConfiguration('rtcm_timeout_s'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'rtk_fixed_h_acc_threshold_m': LaunchConfiguration('rtk_fixed_h_acc_threshold_m'),
                'rtk_float_h_acc_threshold_m': LaunchConfiguration('rtk_float_h_acc_threshold_m'),
                'sat_used_flag_mask': LaunchConfiguration('sat_used_flag_mask'),
                'use_ublox_carrsoln_if_available': LaunchConfiguration('use_ublox_carrsoln_if_available'),
            }]
        ),
    ])