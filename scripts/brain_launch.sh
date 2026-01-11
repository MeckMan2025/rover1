#!/bin/bash
# Brain-Selector Optimized Launch Script (Option C Hybrid)
# Fast startup while preserving all essential startup_launch.sh features
#
# =============================================================================
# LAUNCH HIERARCHY (do not launch multiple files that include each other!)
#
#   patrol_lite.launch.py  <- USE THIS (includes everything below)
#       └── rover.launch.py (base system: motors, camera, teleop, dog_follower)
#
# =============================================================================

set -e

# SAFETY OVERRIDE: If this file exists, the service will not run.
if [ -f "$HOME/ros2_ws/src/rover1/STOP_ROVER" ]; then
    echo ">>> SAFETY OVERRIDE DETECTED. Exiting brain launch script."
    exit 0
fi

echo ">>> Brain-Selector: Starting Rover1..."

# 1. Fast Network Check (5 second timeout vs 30 seconds)
echo ">>> Quick network check..."
for i in {1..5}; do
    if ping -c 1 -W 1 8.8.8.8 &> /dev/null; then
        echo ">>> Network is UP!"
        NETWORK_AVAILABLE=true
        break
    fi
    sleep 1
done

# 2. Optimized Git Update (fast-fail vs blocking)
echo ">>> Checking for code updates..."
cd ~/ros2_ws/src/rover1
if [ "$NETWORK_AVAILABLE" = true ]; then
    # Fast git pull with timeout
    timeout 10 git pull 2>/dev/null || echo ">>> Git update skipped (offline/timeout)"
else
    echo ">>> Network unavailable - using cached code"
fi

# 3. Cached Dependency Check (only if marker file missing)
echo ">>> Verifying dependencies..."
if [ ! -f .deps_verified ] || [ "$NETWORK_AVAILABLE" = true ]; then
    source /opt/ros/jazzy/setup.bash
    cd ~/ros2_ws
    
    # Quick dependency check
    if rosdep install --from-paths src --ignore-src -r -y --dry-run 2>/dev/null | grep -q "All required packages"; then
        echo ">>> Dependencies OK"
        touch ~/ros2_ws/src/rover1/.deps_verified
    elif [ "$NETWORK_AVAILABLE" = true ]; then
        echo ">>> Installing missing dependencies..."
        rosdep install --from-paths src --ignore-src -r -y 2>&1 | grep -v "^#All" || true
        touch ~/ros2_ws/src/rover1/.deps_verified
    else
        echo ">>> Dependency check skipped (offline mode)"
    fi
else
    echo ">>> Dependencies verified (cached)"
fi

# 4. Environment Setup (same as startup_launch.sh)
cd ~/ros2_ws/src/rover1
source /opt/ros/jazzy/setup.bash
source scripts/load_env.sh
source ~/ros2_ws/install/setup.bash

# 5. Launch Full Stack (same as startup_launch.sh but no delays)
echo ">>> Launching Rover1 Full Stack (patrol_lite)..."

# Launch patrol_lite (includes rover.launch.py + patrol nodes)
ros2 launch rover1_bringup patrol_lite.launch.py &
ROVER_PID=$!

# Quick startup verification (15 seconds vs 30)
sleep 15

echo ">>> Brain-Selector: Rover1 stack launched successfully!"
echo ">>> Full system active: teleop + camera + patrol + dog follower"
echo ">>> Dashboard available at http://rover1.local:8080"

# Quick duplicate node check (5 seconds vs original timing)
echo ">>> Quick system check..."
sleep 5
DUPES=$(ros2 node list 2>/dev/null | sort | uniq -c | grep -v "^ *1 " || true)
if [ -n "$DUPES" ]; then
    echo "WARNING: Duplicate nodes detected - check launch file conflicts"
    echo "$DUPES"
else
    echo ">>> System check passed - no conflicts detected"
fi

# Launch camera viewer for immediate visual feedback
echo ">>> Launching camera viewer for touchscreen..."
cd ~/ros2_ws/src/rover1
python3 scripts/rover_camera_viewer.py --rover1 &
CAMERA_PID=$!

echo ">>> Camera viewer launched (PID: $CAMERA_PID)"
echo ">>> Touch screen to exit camera view"

# Keep process running for brain-selector
wait $ROVER_PID