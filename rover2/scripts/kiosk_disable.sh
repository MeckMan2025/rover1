#!/bin/bash
# Disable Kiosk Mode
# Removes kiosk launch from shell profile and disables autologin

set -e

USER_NAME="andrewmeckley"
USER_HOME="/home/$USER_NAME"

echo "=== Disabling Rover Kiosk Mode ==="

echo ""
echo "1. Removing kiosk block from .bash_profile..."
KIOSK_MARKER="# >>> ROVER KIOSK MODE >>>"
KIOSK_END_MARKER="# <<< ROVER KIOSK MODE <<<"

if [ -f "$USER_HOME/.bash_profile" ]; then
    sed -i "/$KIOSK_MARKER/,/$KIOSK_END_MARKER/d" "$USER_HOME/.bash_profile"
    echo "   Removed kiosk block from .bash_profile"
else
    echo "   No .bash_profile found"
fi

echo ""
echo "2. Disabling autologin..."
sudo rm -f /etc/systemd/system/getty@tty1.service.d/autologin.conf
sudo rmdir /etc/systemd/system/getty@tty1.service.d 2>/dev/null || true

echo ""
echo "3. Cleaning up old systemd service (if exists)..."
sudo systemctl disable kiosk.service 2>/dev/null || true
sudo systemctl stop kiosk.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/kiosk.service

sudo systemctl daemon-reload

echo ""
echo "=== Kiosk Mode Disabled ==="
echo ""
echo "On next reboot, tty1 will show a normal login prompt."
echo "To re-enable: ./scripts/kiosk_setup.sh"
