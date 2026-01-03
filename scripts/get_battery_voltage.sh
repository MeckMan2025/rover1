#!/bin/bash
# get_battery_voltage.sh
# Reads battery voltage from ROS topic for tmux status bar display
#
# Output format: "🔋 12.6V" or "🔋 --" if unavailable
#
# Usage: Called by tmux status-right every few seconds
# Dependencies: ros2 CLI (from sourced ROS environment)

# Source ROS environment if not already sourced
if ! command -v ros2 &> /dev/null; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    fi
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash
    fi
fi

# Try ROS topic (1 second timeout to keep tmux responsive)
# Note: --qos-reliability best_effort required to match sensor_data QoS publisher
voltage=$(timeout 1 ros2 topic echo /battery_voltage std_msgs/msg/Float32 --qos-reliability best_effort --once 2>/dev/null | grep "data:" | awk '{print $2}')

if [ -n "$voltage" ] && [ "$voltage" != "0" ]; then
    # Format with one decimal place
    printf "🔋 %.1fV" "$voltage"
else
    # Fallback: show placeholder
    echo "🔋 --"
fi
