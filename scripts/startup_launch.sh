#!/bin/bash
# Rover1 Master Startup Script
# Designed for Systemd Autostart

set -e

# SAFETY OVERRIDE: If this file exists, the service will not run.
# Create it via: touch ~/ros2_ws/src/rover1/STOP_ROVER
if [ -f "$HOME/ros2_ws/src/rover1/STOP_ROVER" ]; then
    echo ">>> SAFETY OVERRIDE DETECTED. Exiting startup script."
    exit 0
fi

echo ">>> Rover1 Starting in 15 seconds... (Press Ctrl+C in SSH to override if testing manually)"
sleep 15

# 1. Wait for Network (Up to 30 seconds)
echo ">>> Waiting for network..."
for i in {1..30}; do
    if ping -c 1 8.8.8.8 &> /dev/null; then
        echo ">>> Network is UP!"
        break
    fi
    sleep 1
done

# 2. Update Code
echo ">>> Checking for updates..."
cd ~/ros2_ws/src/rover1
git pull || echo "Warning: Git pull failed (offline or conflict), continuing anyway..."

# 3. Environment & Workspace (Proper order for custom message visibility)
source /opt/ros/jazzy/setup.bash
source scripts/load_env.sh
source ~/ros2_ws/install/setup.bash

# 4. Launch Essential Stack (Teleop + Camera + Odometry Patrol)
# NOTE: RTAB-Map and Nav2 disabled - too heavy for Pi 5, causes instability
# Use patrol_lite which includes simple_waypoint_follower (odometry-based)
echo ">>> Starting Rover1 Essential Stack..."

# Launch base rover in background
echo ">>> [1/2] Launching rover base system..."
ros2 launch rover1_bringup rover.launch.py &
ROVER_PID=$!
sleep 20

# Launch lightweight patrol (waypoint recorder + simple follower + dashboard)
echo ">>> [2/2] Launching lightweight patrol system..."
ros2 launch rover1_bringup patrol_lite.launch.py &
sleep 10

echo ">>> Essential stack launched (teleop + camera + odometry patrol)."
echo ">>> RTAB-Map and Nav2 are DISABLED for stability."
echo ">>> Dashboard available at http://rover1.local:8080"

# Wait for rover (main process) - keeps systemd happy
wait $ROVER_PID
