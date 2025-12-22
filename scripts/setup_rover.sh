#!/bin/bash
# Complete rover setup script
# Run this on a fresh Raspberry Pi to set up everything

set -e

echo "=== Complete Rover1 Setup ==="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "1. Setting up udev rules for GPS..."
sudo cp "$PROJECT_DIR/99-ublox-gnss.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "2. Creating .env file from template..."
if [ ! -f "$PROJECT_DIR/.env" ]; then
    cp "$PROJECT_DIR/.env.example" "$PROJECT_DIR/.env"
    echo "⚠️  Please edit .env with your actual credentials!"
else
    echo "✅ .env file already exists"
fi

echo "3. Enabling auto-updater..."
bash "$SCRIPT_DIR/enable_auto_update.sh"

echo ""
echo "🎉 Rover setup complete!"
echo ""
echo "Next steps:"
echo "1. Edit .env with your NTRIP credentials"
echo "2. Build ROS workspace: colcon build"
echo "3. Source workspace: source install/setup.bash"
echo "4. Launch rover: ./launch_rover.sh"