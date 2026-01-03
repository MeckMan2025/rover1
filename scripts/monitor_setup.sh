#!/bin/bash
# Monitor Mode Setup for Rover 7" Touchscreen
# Displays btop system monitor with battery status bar
# Features:
#   - tmux status bar showing battery voltage, IP, and time
#   - Touch-scrollable via mouse mode
#   - SSH users can attach: tmux attach -t rover-monitor
#
# To switch back to dashboard kiosk: ./scripts/kiosk_setup.sh

set -e

USER_NAME="andrewmeckley"
USER_HOME="/home/$USER_NAME"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BATTERY_SCRIPT="$SCRIPT_DIR/get_battery_voltage.sh"

echo "=== Rover Monitor Mode Setup ==="

# Ensure dependencies are installed
echo ""
echo "1. Installing dependencies..."
PACKAGES_NEEDED=""
command -v foot &> /dev/null || PACKAGES_NEEDED="$PACKAGES_NEEDED foot"
command -v btop &> /dev/null || PACKAGES_NEEDED="$PACKAGES_NEEDED btop"
command -v tmux &> /dev/null || PACKAGES_NEEDED="$PACKAGES_NEEDED tmux"

if [ -n "$PACKAGES_NEEDED" ]; then
    echo "   Installing:$PACKAGES_NEEDED"
    sudo apt-get update && sudo apt-get install -y $PACKAGES_NEEDED
else
    echo "   All dependencies already installed"
fi

echo ""
echo "2. Creating tmux configuration..."

cat > "$USER_HOME/.tmux.conf" << 'TMUX_CONF'
# Rover tmux configuration
# Shows battery voltage, IP, and time in status bar

# Status bar at bottom
set -g status on
set -g status-position bottom
set -g status-interval 5

# Status bar content
set -g status-left "[rover1] "
set -g status-left-length 20

# Right side: battery, IP, time
# Note: Battery script path is expanded when this file is created
set -g status-right "#(~/ros2_ws/src/rover1/scripts/get_battery_voltage.sh) | #(hostname -I | awk '{print $1}') | %H:%M"
set -g status-right-length 60

# Colors (Catppuccin-inspired for visibility on 7" screen)
set -g status-style 'bg=#1e1e2e fg=#cdd6f4'
set -g status-left-style 'bg=#89b4fa fg=#1e1e2e bold'
set -g status-right-style 'bg=#1e1e2e fg=#a6adc8'

# Window indicators
set -g window-status-format ' #I '
set -g window-status-current-format ' #I '
set -g window-status-current-style 'bg=#45475a fg=#cdd6f4 bold'

# Enable mouse (for touchscreen scrolling in btop)
set -g mouse on

# Disable escape delay (makes btop more responsive)
set -sg escape-time 0
TMUX_CONF

echo "   Created: $USER_HOME/.tmux.conf"

echo ""
echo "3. Creating btop monitor wrapper script..."
mkdir -p "$USER_HOME/.config/rover-kiosk"

cat > "$USER_HOME/.config/rover-kiosk/launch-monitor.sh" << 'MONITOR_SCRIPT'
#!/bin/bash
# Rover Monitor Launcher
# Runs btop inside tmux with battery status bar on 7" touchscreen
#
# SSH users can attach: tmux attach -t rover-monitor

# Source ROS environment for battery topic access
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "[Monitor] Starting system monitor with battery display..."

# Kill any existing rover-monitor session
tmux kill-session -t rover-monitor 2>/dev/null || true

# Start tmux session with btop, then attach
# Using foot as the terminal emulator (Wayland-native, good for btop's UI)
exec foot tmux new-session -s rover-monitor 'btop'
MONITOR_SCRIPT

chmod +x "$USER_HOME/.config/rover-kiosk/launch-monitor.sh"

# Ensure battery script is executable
chmod +x "$BATTERY_SCRIPT" 2>/dev/null || true

echo ""
echo "4. Updating .bash_profile to launch monitor mode on tty1..."

KIOSK_MARKER="# >>> ROVER KIOSK MODE >>>"
KIOSK_END_MARKER="# <<< ROVER KIOSK MODE <<<"

# Remove old kiosk/monitor block if exists
if [ -f "$USER_HOME/.bash_profile" ]; then
    sed -i "/$KIOSK_MARKER/,/$KIOSK_END_MARKER/d" "$USER_HOME/.bash_profile"
fi

# Add monitor launch block
cat >> "$USER_HOME/.bash_profile" << 'PROFILE_BLOCK'

# >>> ROVER KIOSK MODE >>>
# Launch system monitor on tty1 only (not SSH sessions)
# Using cage Wayland compositor for clean fullscreen display
if [ "$(tty)" = "/dev/tty1" ] && [ -z "$SSH_CONNECTION" ]; then
    echo "Waiting for rover1.service..."
    systemctl is-active --wait rover1.service 2>/dev/null || true
    sleep 5  # Extra time for services to stabilize

    echo "Launching system monitor..."
    exec cage ~/.config/rover-kiosk/launch-monitor.sh
fi
# <<< ROVER KIOSK MODE <<<
PROFILE_BLOCK

echo ""
echo "=== Monitor Mode Setup Complete ==="
echo ""
echo "On next boot, tty1 will show btop with battery status bar."
echo ""
echo "Status bar displays:"
echo "  - Battery voltage (from ROS /battery_voltage topic)"
echo "  - IP address"
echo "  - Current time"
echo ""
echo "Touchscreen scrolling enabled (tmux mouse mode)."
echo ""
echo "SSH users can attach to the same session:"
echo "  tmux attach -t rover-monitor"
echo ""
echo "To switch back to dashboard kiosk:"
echo "  ./scripts/kiosk_setup.sh"
echo ""
echo "Reboot to apply: sudo reboot"
