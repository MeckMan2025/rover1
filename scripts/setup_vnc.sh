#!/bin/bash
# WayVNC Setup Script for Remote Desktop Access
# Sets up VNC server to auto-start on boot for remote rover access

set -e

USER_HOME="/home/andrewmeckley"
CONFIG_DIR="$USER_HOME/.config/wayvnc"

echo "=== WayVNC Setup ==="
echo "Setting up VNC server for remote rover access"
echo

# Check if running as correct user
if [ "$USER" != "andrewmeckley" ]; then
    echo "Warning: This script should be run as andrewmeckley user"
    echo "Current user: $USER"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install wayvnc if not present
if ! command -v wayvnc &> /dev/null; then
    echo "Installing wayvnc..."
    sudo apt update
    sudo apt install -y wayvnc
else
    echo "wayvnc already installed"
fi

# Create config directory
echo "Creating wayvnc config directory..."
mkdir -p "$CONFIG_DIR"

# Create wayvnc config file
echo "Creating wayvnc configuration..."
cat > "$CONFIG_DIR/config" << EOF
address=0.0.0.0
port=5900
enable_auth=false
xkb_layout=us
xkb_variant=
output=
max_rate=30
EOF

# Copy systemd service file
echo "Installing systemd service..."
sudo cp "$(dirname "$0")/../systemd/wayvnc.service" /etc/systemd/system/

# Enable and start the service
echo "Enabling wayvnc service..."
sudo systemctl daemon-reload
sudo systemctl enable wayvnc.service

# Start the service if Wayland is available
if [ -n "$WAYLAND_DISPLAY" ] || [ -S "/run/user/1000/wayland-1" ]; then
    echo "Starting wayvnc service..."
    sudo systemctl start wayvnc.service
    sleep 2
    
    # Check service status
    if systemctl is-active --quiet wayvnc.service; then
        echo "✅ WayVNC service started successfully"
    else
        echo "❌ WayVNC service failed to start"
        systemctl status wayvnc.service --no-pager
    fi
else
    echo "⚠️  Wayland not available - service will start on next boot"
fi

# Set proper permissions
chown -R andrewmeckley:andrewmeckley "$CONFIG_DIR"

echo
echo "=== Setup Complete ==="
echo
echo "WayVNC Configuration:"
echo "  - VNC Port: 5900"
echo "  - Authentication: Disabled (LAN only)"
echo "  - Max framerate: 30 FPS"
echo "  - Config: $CONFIG_DIR/config"
echo "  - Service: wayvnc.service (auto-start on boot)"
echo
echo "Connect via VNC client:"
echo "  - Local network: <rover-ip>:5900"
echo "  - Tailscale: 100.121.159.107:5900"
echo
echo "Service management:"
echo "  - Check status: sudo systemctl status wayvnc"
echo "  - View logs: journalctl -u wayvnc -f"
echo "  - Restart: sudo systemctl restart wayvnc"
echo "  - Disable: sudo systemctl disable wayvnc"
echo

echo "VNC server will auto-start on reboot!"
echo "You can now connect remotely with RealVNC Viewer or any VNC client."