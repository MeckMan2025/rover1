#!/bin/bash
# wifi-roam.sh
# Automatic WiFi roaming script for Rover1
#
# NetworkManager doesn't automatically switch to a higher-priority network
# once connected to a lower-priority one. This script runs periodically
# to check if a preferred network has become available and switches to it.
#
# Usage: Run via systemd timer (wifi-roam.timer) every 60 seconds
#
# Behavior:
#   1. Get current WiFi connection and its priority
#   2. Scan for available networks
#   3. If a higher-priority known network is available, switch to it
#
# Exit codes:
#   0 - No action needed or successful switch
#   1 - Error during execution

set -euo pipefail

# Logging helper
log() {
    logger -t "wifi-roam" "$1"
    echo "[wifi-roam] $1"
}

# Get the priority of a connection by name
get_priority() {
    local conn_name="$1"
    nmcli -t -f connection.autoconnect-priority connection show "$conn_name" 2>/dev/null | cut -d: -f2 || echo "0"
}

# Get current WiFi connection name
get_current_wifi() {
    nmcli -t -f NAME,TYPE,DEVICE connection show --active 2>/dev/null | \
        grep ":wifi:" | grep -v "^--" | head -1 | cut -d: -f1 || echo ""
}

# Get list of available WiFi networks (SSIDs)
get_available_networks() {
    # Trigger a scan first (non-blocking)
    nmcli device wifi rescan 2>/dev/null || true
    sleep 2

    # List available networks
    nmcli -t -f SSID device wifi list 2>/dev/null | sort -u | grep -v "^$" || echo ""
}

# Get list of known (configured) WiFi connections with priorities
get_known_connections() {
    # Returns: connection_name:priority for each WiFi connection
    nmcli -t -f NAME,TYPE connection show 2>/dev/null | \
        grep ":wifi$" | cut -d: -f1 | while read -r conn; do
            priority=$(get_priority "$conn")
            echo "$conn:$priority"
        done | sort -t: -k2 -rn  # Sort by priority descending
}

# Main roaming logic
main() {
    # Get current connection
    current=$(get_current_wifi)

    if [ -z "$current" ]; then
        log "No WiFi connected - letting NetworkManager handle initial connection"
        exit 0
    fi

    current_priority=$(get_priority "$current")
    log "Current: '$current' (priority: $current_priority)"

    # Get available networks
    available=$(get_available_networks)

    if [ -z "$available" ]; then
        log "No networks found in scan"
        exit 0
    fi

    # Check each known connection (sorted by priority, highest first)
    while IFS=: read -r conn_name priority; do
        # Skip if this is our current connection
        if [ "$conn_name" = "$current" ]; then
            continue
        fi

        # Skip if priority is not higher than current
        if [ "$priority" -le "$current_priority" ]; then
            continue
        fi

        # Check if this network is available
        if echo "$available" | grep -qF "$conn_name"; then
            log "Found higher-priority network: '$conn_name' (priority: $priority)"
            log "Switching from '$current' to '$conn_name'..."

            # Attempt to connect
            if nmcli connection up "$conn_name" 2>&1; then
                log "Successfully switched to '$conn_name'"
                exit 0
            else
                log "Failed to connect to '$conn_name' - staying on '$current'"
                exit 1
            fi
        fi
    done <<< "$(get_known_connections)"

    log "Already on best available network: '$current'"
    exit 0
}

main "$@"
