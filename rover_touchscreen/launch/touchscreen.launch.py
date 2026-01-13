#!/usr/bin/env python3
"""
Launch file for rover touchscreen dashboard
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_touchscreen',
            executable='touchscreen_dashboard',
            name='touchscreen_dashboard',
            output='screen',
            parameters=[],
            arguments=[]
        )
    ])