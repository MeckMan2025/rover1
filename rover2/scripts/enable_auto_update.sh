#!/bin/bash
# Script to enable auto-updater on the rover
# Run this script on the Raspberry Pi to activate automatic updates

set -e

echo "=== Rover1 Auto-Updater Setup ==="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Copy systemd files to system directory
echo "Installing systemd service and timer..."
sudo cp "$SCRIPT_DIR/rover1-updater.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/rover1-updater.timer" /etc/systemd/system/

# Reload systemd to recognize new files
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable and start the timer
echo "Enabling auto-updater timer..."
sudo systemctl enable rover1-updater.timer
sudo systemctl start rover1-updater.timer

# Check status
echo "Checking auto-updater status..."
sudo systemctl status rover1-updater.timer --no-pager

echo ""
echo "✅ Auto-updater enabled successfully!"
echo "🔄 Rover will check for updates every 5 minutes"
echo "📋 Check status with: sudo systemctl status rover1-updater.timer"
echo "🛑 Stop with: sudo systemctl stop rover1-updater.timer"
echo "❌ Disable with: sudo systemctl disable rover1-updater.timer"