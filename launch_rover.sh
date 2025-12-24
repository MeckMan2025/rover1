#!/bin/bash
# Convenience script to automatically load environment and launch rover

# Load environment variables
source scripts/load_env.sh

# Source the ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch the rover
echo "Starting rover launch..."
ros2 launch rover1_bringup rover.launch.py