#!/bin/bash
# Disable Kiosk Mode
# Use this to temporarily disable kiosk for debugging

set -e

echo "=== Disabling Rover Kiosk Mode ==="

echo "Stopping kiosk service..."
sudo systemctl stop kiosk.service 2>/dev/null || true

echo "Disabling kiosk service..."
sudo systemctl disable kiosk.service 2>/dev/null || true

echo "Removing autologin..."
sudo rm -f /etc/systemd/system/getty@tty1.service.d/autologin.conf

sudo systemctl daemon-reload

echo ""
echo "Kiosk mode disabled."
echo "To re-enable: ./kiosk_setup.sh"
