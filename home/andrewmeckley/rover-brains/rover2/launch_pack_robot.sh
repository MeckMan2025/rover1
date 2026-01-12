#!/bin/bash
# Brain-Selector Launch Script for Pack Robot (Rover2)
# Based on rover2_bringup package for the pack robot configuration

set -e

# SAFETY OVERRIDE: If this file exists, the service will not run.
if [ -f "$HOME/ros2_ws/src/rover1/STOP_ROVER" ]; then
    echo ">>> SAFETY OVERRIDE DETECTED. Exiting pack robot launch."
    exit 0
fi

echo ">>> Brain-Selector: Starting Pack Robot (Rover2)..."

# 1. Fast Network Check
echo ">>> Quick network check..."
NETWORK_AVAILABLE=false
for i in {1..5}; do
    if ping -c 1 -W 1 8.8.8.8 &> /dev/null; then
        echo ">>> Network is UP!"
        NETWORK_AVAILABLE=true
        break
    fi
    sleep 1
done

# 2. Quick Git Update
echo ">>> Checking for code updates..."
cd ~/ros2_ws/src/rover1
if [ "$NETWORK_AVAILABLE" = true ]; then
    timeout 10 git pull 2>/dev/null || echo ">>> Git update skipped (offline/timeout)"
else
    echo ">>> Network unavailable - using cached code"
fi

# 3. Environment Setup
cd ~/ros2_ws/src/rover1
source /opt/ros/jazzy/setup.bash
source scripts/load_env.sh
source ~/ros2_ws/install/setup.bash

# 4. Launch Pack Robot Stack (rover2.launch.py)
echo ">>> Launching Pack Robot Stack (rover2)..."

# Launch rover2 configuration
ros2 launch rover2_bringup rover2.launch.py &
ROVER_PID=$!

sleep 15

echo ">>> Brain-Selector: Pack Robot stack launched successfully!"
echo ">>> Pack robot active: motors + camera + person follower"
echo ">>> Dashboard available at http://rover1.local:8080"

# Quick system verification
echo ">>> Quick system check..."
sleep 5
DUPES=$(ros2 node list 2>/dev/null | sort | uniq -c | grep -v "^ *1 " || true)
if [ -n "$DUPES" ]; then
    echo "WARNING: Duplicate nodes detected - check launch file conflicts"
    echo "$DUPES"
else
    echo ">>> System check passed - no conflicts detected"
fi

# Launch PyQt Camera Viewer on touchscreen
echo ">>> Launching camera viewer on touchscreen..."
export DISPLAY=:0
export QT_QPA_PLATFORM=wayland
cd ~/ros2_ws/src/rover1
python3 scripts/rover_camera_viewer.py --rover2 &
CAMERA_PID=$!
echo ">>> Camera viewer started (PID: $CAMERA_PID)"
echo ">>> Touch screen to exit camera view"

# Keep process running for brain-selector
wait $ROVER_PID || wait $CAMERA_PID