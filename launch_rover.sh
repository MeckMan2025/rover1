#!/bin/bash
# Convenience script to automatically load environment and launch rover

# Load environment variables
source scripts/load_env.sh

# Launch the rover
echo "Starting rover launch..."
ros2 launch rover1_bringup rover.launch.py