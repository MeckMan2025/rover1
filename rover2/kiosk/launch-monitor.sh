#!/bin/bash
# launch-monitor.sh
# Kiosk launcher for Rover1 7" touchscreen display
#
# Runs btop and rover_monitor.sh in a split tmux layout:
#   - Top pane: btop (system resources)
#   - Bottom pane: rover_monitor.sh (ROS status, GPS, battery, SSID)
#
# Install location: ~/.config/rover-kiosk/launch-monitor.sh
# Called by: cage (Wayland kiosk compositor)

# Wait for system to settle
sleep 2

# Set terminal font size for 7" display readability
export FOOT_FONT="monospace:size=10"

# Kill any existing rover-monitor session
tmux kill-session -t rover-monitor 2>/dev/null || true

# Launch split tmux session:
# 1. Create session with btop in top pane
# 2. Split vertically (bottom pane gets 35% height)
# 3. Run rover_monitor.sh in bottom pane
# 4. Select top pane as default view
exec foot tmux new-session -s rover-monitor \; \
    send-keys 'btop' Enter \; \
    split-window -v -p 35 \; \
    send-keys '~/ros2_ws/src/rover1/scripts/rover_monitor.sh' Enter \; \
    select-pane -t 0
