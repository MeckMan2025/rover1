#!/bin/bash
# setup_ethernet_tether.sh
# This script configures the Pi to have a static IP (10.42.0.1) on the Ethernet port
# while still allowing it to receive a DHCP address from a router.
# It also prevents the 2-minute "wait for network" hang on boot.

set -e

INTERFACE="eth0"
STATIC_IP="10.42.0.1/24"

echo "=== Rover1 Ethernet Tether Setup ==="

# Create the netplan configuration
# We use a high priority filename (99-tether.yaml) to ensure it overrides defaults
cat <<EOF | sudo tee /etc/netplan/99-tether.yaml
network:
  version: 2
  ethernets:
    $INTERFACE:
      dhcp4: true
      optional: true
      addresses:
        - $STATIC_IP
EOF

echo "✅ Created /etc/netplan/99-tether.yaml"

# Apply the profile
echo "Applying network changes..."
sudo netplan apply

echo ""
echo "🎉 Setup Complete!"
echo "You can now always SSH into the rover via Ethernet at:"
echo "ssh andrewmeckley@10.42.0.1"
echo ""
echo "Note: On your Mac, you should set your Ethernet adapter to:"
echo "  IP: 10.42.0.2"
echo "  Subnet Mask: 255.255.255.0"
echo "  Router: 10.42.0.1"
