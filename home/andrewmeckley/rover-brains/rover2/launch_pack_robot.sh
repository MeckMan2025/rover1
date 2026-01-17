#!/bin/bash
# Brain-Selector Launch Script for Pack Robot (Rover2)
# Based on rover2_bringup package for the pack robot configuration

set -e

# SAFETY OVERRIDE: If this file exists, the service will not run.
if [ -f "$HOME/ros2_ws/src/rover1/STOP_ROVER" ]; then
    echo ">>> SAFETY OVERRIDE DETECTED. Exiting pack robot launch."
    exit 0
fi

# Check if rover2 is already running (prevent duplicate launches)
if pgrep -f "ros2 launch rover2_bringup rover2.launch.py" > /dev/null; then
    echo ">>> Pack Robot already running - skipping duplicate launch"
    # Wait to keep brain-selector happy
    sleep infinity
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

# Launch rover2 configuration with person follower enabled
ros2 launch rover2_bringup rover2.launch.py enable_person_follower:=true &
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

# Launch Touchscreen Dashboard (battery voltage + GPS sats + camera)
echo ">>> Launching touchscreen dashboard..."
export DISPLAY=:0
export QT_QPA_PLATFORM=wayland
# Note: Using direct path because rover_touchscreen installs to bin/ instead of lib/
# which makes ros2 run unable to find it. TODO: fix package build structure.
$HOME/ros2_ws/install/rover_touchscreen/bin/touchscreen_dashboard &
DASHBOARD_PID=$!
echo ">>> Touchscreen dashboard started (PID: $DASHBOARD_PID)"

# Monitor and restart dashboard if it crashes (runs in background)
(
    while true; do
        sleep 5
        if ! kill -0 $DASHBOARD_PID 2>/dev/null; then
            echo ">>> Dashboard crashed, restarting..."
            WAYLAND_DISPLAY=wayland-0 QT_QPA_PLATFORM=wayland \
                $HOME/ros2_ws/install/rover_touchscreen/bin/touchscreen_dashboard &
            DASHBOARD_PID=$!
            echo ">>> Dashboard restarted (PID: $DASHBOARD_PID)"
        fi
    done
) &

# Keep main process alive while rover runs
wait $ROVER_PID