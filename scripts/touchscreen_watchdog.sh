#!/bin/bash
# Touchscreen Dashboard Watchdog
# Monitors the dashboard and restarts it if frozen or crashed
#
# The PyQt5 GUI can freeze under Wayland/cage while the process is still
# running. This watchdog detects unresponsive states and restarts.

DASHBOARD_CMD="$HOME/ros2_ws/install/rover_touchscreen/bin/touchscreen_dashboard"
CHECK_INTERVAL=30  # seconds between health checks
MAX_FAILURES=2     # restart after this many consecutive failures

export WAYLAND_DISPLAY=wayland-0
export DISPLAY=:0
export QT_QPA_PLATFORM=wayland

failure_count=0

start_dashboard() {
    echo "[Watchdog] Starting touchscreen dashboard..."
    $DASHBOARD_CMD &
    DASHBOARD_PID=$!
    echo "[Watchdog] Dashboard started with PID: $DASHBOARD_PID"
    sleep 5  # Give it time to initialize
}

check_dashboard() {
    # Check if process exists
    if ! pgrep -f "touchscreen_dashboard" > /dev/null; then
        echo "[Watchdog] Dashboard process not found"
        return 1
    fi

    # Check if ROS node is responsive (can list its parameters)
    if ! timeout 5 ros2 node info /rover_touchscreen > /dev/null 2>&1; then
        echo "[Watchdog] Dashboard ROS node not responsive"
        return 1
    fi

    return 0
}

kill_dashboard() {
    echo "[Watchdog] Killing frozen dashboard..."
    pkill -9 -f touchscreen_dashboard 2>/dev/null
    sleep 2
}

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Initial start
start_dashboard

# Main watchdog loop
while true; do
    sleep $CHECK_INTERVAL

    if check_dashboard; then
        failure_count=0
    else
        ((failure_count++))
        echo "[Watchdog] Health check failed ($failure_count/$MAX_FAILURES)"

        if [ $failure_count -ge $MAX_FAILURES ]; then
            kill_dashboard
            start_dashboard
            failure_count=0
        fi
    fi
done
