#!/bin/bash
# Rover1 Master Startup Script
# Designed for Systemd Autostart
#
# =============================================================================
# LAUNCH HIERARCHY (do not launch multiple files that include each other!)
#
#   patrol_lite.launch.py  <- USE THIS (includes everything below)
#       └── rover.launch.py (base system: motors, camera, teleop, dog_follower)
#
#   rtabmap.launch.py <- For SLAM (includes rover.launch.py)
#   nav2.launch.py    <- For Nav2 (includes rover.launch.py) - UNSTABLE on Pi5
#
# NEVER launch rover.launch.py AND patrol_lite.launch.py together!
# =============================================================================

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
# Use patrol_lite which includes rover.launch.py + adds patrol nodes
echo ">>> Starting Rover1 Essential Stack..."

# Launch patrol_lite (includes rover.launch.py + patrol nodes)
# NOTE: Do NOT launch rover.launch.py separately - patrol_lite includes it
echo ">>> Launching patrol_lite (base rover + patrol system)..."
ros2 launch rover1_bringup patrol_lite.launch.py &
ROVER_PID=$!
sleep 30

echo ">>> Essential stack launched (teleop + camera + odometry patrol + dog follower)."
echo ">>> RTAB-Map and Nav2 are DISABLED for stability."
echo ">>> Dashboard available at http://rover1.local:8080"

# Sanity check for duplicate nodes
echo ">>> Running duplicate node check..."
sleep 5  # Let nodes fully register
DUPES=$(ros2 node list 2>/dev/null | sort | uniq -c | grep -v "^ *1 " || true)
if [ -n "$DUPES" ]; then
    echo "==========================================="
    echo "WARNING: DUPLICATE NODES DETECTED!"
    echo "==========================================="
    echo "$DUPES"
    echo ""
    echo "This may cause service calls to hang."
    echo "Check launch file includes for conflicts."
    echo "==========================================="
else
    echo ">>> Node check passed - no duplicates."
fi

# Wait for rover (main process) - keeps systemd happy
wait $ROVER_PID
