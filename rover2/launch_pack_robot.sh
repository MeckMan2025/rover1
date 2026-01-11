#!/bin/bash
# Pack Robot Launch Script - Minimal rover2 system for human following

# Load environment variables
source scripts/load_env.sh

# Source the ROS 2 workspace
source /home/andrewmeckley/rover-brains/rover2/install/setup.bash

# Launch the pack robot (minimal system)
echo "🤖 Starting Pack Robot..."
echo "Essential components: Motors + Camera + Web Dashboard + Person Follower"
echo "Web interface: http://rover-ip:8080"
echo ""

ros2 launch rover2_bringup pack_robot.launch.py