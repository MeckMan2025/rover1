#!/bin/bash
# setup_wifi_roam.sh
# Installs the WiFi roaming service for Rover1
#
# This service automatically switches to higher-priority WiFi networks
# when they become available. Solves the problem where the rover stays
# connected to a phone hotspot even after returning home.
#
# Usage: sudo ./scripts/setup_wifi_roam.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# --- Check for root ---
if [[ $EUID -ne 0 ]]; then
    echo "ERROR: This script must be run with sudo"
    echo "Usage: sudo $0"
    exit 1
fi

echo "=== WiFi Roaming Service Setup ==="
echo ""

# --- Install the roaming script ---
echo "Installing wifi-roam.sh..."
cp "$SCRIPT_DIR/wifi-roam.sh" /usr/local/bin/wifi-roam.sh
chmod +x /usr/local/bin/wifi-roam.sh
echo "  Installed: /usr/local/bin/wifi-roam.sh"

# --- Install systemd units ---
echo ""
echo "Installing systemd units..."

cp "$REPO_ROOT/systemd/wifi-roam.service" /etc/systemd/system/
cp "$REPO_ROOT/systemd/wifi-roam.timer" /etc/systemd/system/
echo "  Installed: /etc/systemd/system/wifi-roam.service"
echo "  Installed: /etc/systemd/system/wifi-roam.timer"

# --- Reload systemd ---
echo ""
echo "Reloading systemd..."
systemctl daemon-reload

# --- Enable and start timer ---
echo ""
echo "Enabling wifi-roam.timer..."
systemctl enable wifi-roam.timer
systemctl start wifi-roam.timer

# --- Verify ---
echo ""
echo "=== Installation Complete ==="
echo ""
echo "Service Status:"
systemctl status wifi-roam.timer --no-pager || true
echo ""
echo "Timer Schedule:"
systemctl list-timers wifi-roam.timer --no-pager || true
echo ""
echo "Behavior:"
echo "  - Checks every 60 seconds for higher-priority WiFi"
echo "  - Automatically switches when a preferred network appears"
echo "  - Logs to journald: journalctl -t wifi-roam -f"
echo ""
echo "Manual test:"
echo "  sudo systemctl start wifi-roam.service"
echo "  journalctl -t wifi-roam --since '1 min ago'"
echo ""
echo "To disable:"
echo "  sudo systemctl stop wifi-roam.timer"
echo "  sudo systemctl disable wifi-roam.timer"
echo ""
