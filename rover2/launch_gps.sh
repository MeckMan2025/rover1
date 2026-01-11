#!/bin/bash
# Convenience script to automatically load environment and launch GPS

# Load environment variables
source scripts/load_env.sh

# Source the ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch GPS
echo "Starting GPS launch..."
ros2 launch rover1_bringup gps.launch.py