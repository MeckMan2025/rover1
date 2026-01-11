#!/bin/bash
# Pack Robot Launch Script - Minimal rover2 system

# Load environment variables
source scripts/load_env.sh

# Source the ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch the pack robot (minimal system)
echo "Starting Pack Robot launch..."
ros2 launch rover2_bringup pack_robot.launch.py