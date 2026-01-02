#!/bin/bash
# Monitor Mode Setup for Rover 7" Touchscreen
# Displays btop system monitor instead of web dashboard kiosk
# Low CPU usage, shows system stats during demos
#
# To switch back to dashboard kiosk: ./scripts/kiosk_setup.sh

set -e

USER_NAME="andrewmeckley"
USER_HOME="/home/$USER_NAME"

echo "=== Rover Monitor Mode Setup ==="

# Ensure foot terminal emulator is installed (needed for btop's advanced rendering)
if ! command -v foot &> /dev/null; then
    echo "Installing foot terminal emulator..."
    sudo apt-get update && sudo apt-get install -y foot
fi

# Ensure btop is installed
if ! command -v btop &> /dev/null; then
    echo "Installing btop..."
    sudo apt-get install -y btop
fi

echo ""
echo "1. Creating btop monitor wrapper script..."
mkdir -p "$USER_HOME/.config/rover-kiosk"

cat > "$USER_HOME/.config/rover-kiosk/launch-monitor.sh" << 'MONITOR_SCRIPT'
#!/bin/bash
# Rover Monitor Launcher
# Shows btop on the 7" display via foot terminal emulator

echo "[Monitor] Starting system monitor..."

# Launch btop inside foot terminal (foot is Wayland-native, needed for btop's UI)
exec foot btop
MONITOR_SCRIPT

chmod +x "$USER_HOME/.config/rover-kiosk/launch-monitor.sh"

echo ""
echo "2. Updating .bash_profile to launch monitor mode on tty1..."

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
echo "On next boot, tty1 will show btop system monitor."
echo "SSH sessions are unaffected."
echo ""
echo "To switch back to dashboard kiosk:"
echo "  ./scripts/kiosk_setup.sh"
echo ""
echo "Reboot to apply: sudo reboot"
