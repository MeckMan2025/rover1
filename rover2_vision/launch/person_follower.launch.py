#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch person follower node for rover2 with YAML config."""

    # Get package share directory for config file
    pkg_share = get_package_share_directory('rover2_vision')
    config_file = os.path.join(pkg_share, 'config', 'person_follower.yaml')

    return LaunchDescription([
        # Person Follower Node - all parameters loaded from YAML
        Node(
            package='rover2_vision',
            executable='person_follower',
            name='person_follower',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[config_file]
        )
    ])