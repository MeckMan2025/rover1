#!/bin/bash
# setup_network_failover.sh
# Configures WiFi failover chain: Home Network -> Phone Hotspot -> Ethernet Tether
# Uses NetworkManager for WiFi priority and Netplan for Ethernet static fallback.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${SCRIPT_DIR}/../.env"

echo "=== Rover1 Network Failover Setup ==="
echo ""

# Load credentials from .env
if [ -f "$ENV_FILE" ]; then
    source "$ENV_FILE"
    echo "Loaded credentials from .env"
else
    echo "ERROR: .env file not found at $ENV_FILE"
    echo "Please copy .env.example to .env and fill in your credentials."
    exit 1
fi

# Validate required variables
if [ -z "$WIFI_HOME_SSID" ] || [ -z "$WIFI_HOME_PASS" ]; then
    echo "ERROR: WIFI_HOME_SSID and WIFI_HOME_PASS must be set in .env"
    exit 1
fi

if [ -z "$WIFI_HOTSPOT_SSID" ] || [ -z "$WIFI_HOTSPOT_PASS" ]; then
    echo "ERROR: WIFI_HOTSPOT_SSID and WIFI_HOTSPOT_PASS must be set in .env"
    exit 1
fi

echo ""
echo "WiFi Networks to configure:"
echo "  1. Home:    $WIFI_HOME_SSID (Priority: 100 - Highest)"
echo "  2. Hotspot: $WIFI_HOTSPOT_SSID (Priority: 50 - Fallback)"
echo "  3. Ethernet: 10.42.0.1 (Static - Always Available)"
echo ""

# --- WiFi Configuration via NetworkManager ---

echo "Configuring WiFi networks..."

# Remove existing connections with same names (if any) to avoid duplicates
sudo nmcli connection delete "$WIFI_HOME_SSID" 2>/dev/null || true
sudo nmcli connection delete "$WIFI_HOTSPOT_SSID" 2>/dev/null || true

# Add Home WiFi (highest priority = 100)
echo "Adding home network: $WIFI_HOME_SSID"
sudo nmcli connection add \
    type wifi \
    con-name "$WIFI_HOME_SSID" \
    ssid "$WIFI_HOME_SSID" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$WIFI_HOME_PASS" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100

# Add Hotspot WiFi (lower priority = 50)
echo "Adding hotspot network: $WIFI_HOTSPOT_SSID"
sudo nmcli connection add \
    type wifi \
    con-name "$WIFI_HOTSPOT_SSID" \
    ssid "$WIFI_HOTSPOT_SSID" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$WIFI_HOTSPOT_PASS" \
    connection.autoconnect yes \
    connection.autoconnect-priority 50

echo "WiFi connections configured."

# --- Ethernet Static Fallback via Netplan ---

echo ""
echo "Configuring Ethernet static fallback..."

INTERFACE="eth0"
STATIC_IP="10.42.0.1/24"

cat <<EOF | sudo tee /etc/netplan/99-rover-network.yaml
# Rover1 Network Configuration
# - Ethernet: Static IP for tethered laptop connection + DHCP for router
# - WiFi: Managed by NetworkManager with priority failover
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    $INTERFACE:
      dhcp4: true
      optional: true
      addresses:
        - $STATIC_IP
EOF

echo "Created /etc/netplan/99-rover-network.yaml"

# Apply netplan
echo "Applying network configuration..."
sudo netplan apply

# --- Verification ---

echo ""
echo "=== Configuration Complete ==="
echo ""
echo "Network Priority Chain:"
echo "  1. $WIFI_HOME_SSID (auto-connect priority: 100)"
echo "  2. $WIFI_HOTSPOT_SSID (auto-connect priority: 50)"
echo "  3. Ethernet tether: ssh andrewmeckley@10.42.0.1"
echo ""
echo "Current connections:"
nmcli connection show | grep -E "(wifi|ethernet)" || true
echo ""
echo "Behavior:"
echo "  - When home WiFi is in range: Connects automatically"
echo "  - When only hotspot is in range: Falls back to hotspot"
echo "  - When no WiFi available: Ethernet tether always works"
echo ""
echo "To manually switch networks:"
echo "  nmcli connection up '$WIFI_HOME_SSID'"
echo "  nmcli connection up '$WIFI_HOTSPOT_SSID'"
echo ""
echo "To check current connection:"
echo "  nmcli device status"
echo ""
