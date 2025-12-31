#!/bin/bash
#
# Stadia Controller Auto-Connect Daemon
# Continuously scans for and connects to the Stadia controller
#
# Problem: The controller may pair with PC first when powered on normally.
# Solution: When user puts controller in pairing mode (Y + Stadia), this daemon
#           detects it and immediately connects.
#
# Author: Rover1 Project
# Date: 2025-12-31
#

set -u

# Configuration
CONTROLLER_MAC="D1:71:42:54:CB:0F"
SCAN_INTERVAL=2          # Seconds between scan cycles
SCAN_DURATION=3          # Seconds to scan for devices
CONNECT_COOLDOWN=10      # Seconds to wait after successful connect before next attempt
SERVICE_NAME="stadia-autoconnect"

# State
RUNNING=true
LAST_CONNECT_TIME=0

# Logging function - logs to journald
log() {
    local level="$1"
    local message="$2"
    logger -t "$SERVICE_NAME" -p "user.$level" "$message"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [$level] $message"
}

log_info()  { log "info" "$1"; }
log_warn()  { log "warning" "$1"; }
log_error() { log "err" "$1"; }

# Signal handlers for graceful shutdown
cleanup() {
    log_info "Received shutdown signal, stopping..."
    RUNNING=false
    # Stop any ongoing scan
    bluetoothctl scan off 2>/dev/null || true
    log_info "Daemon stopped"
    exit 0
}

trap cleanup SIGTERM SIGINT SIGHUP

# Check if controller is already connected
is_connected() {
    bluetoothctl info "$CONTROLLER_MAC" 2>/dev/null | grep -q "Connected: yes"
}

# Check if controller is trusted (can auto-reconnect)
is_trusted() {
    bluetoothctl info "$CONTROLLER_MAC" 2>/dev/null | grep -q "Trusted: yes"
}

# Check if controller is paired
is_paired() {
    bluetoothctl info "$CONTROLLER_MAC" 2>/dev/null | grep -q "Paired: yes"
}

# Scan for devices and check if our controller is advertising
# Returns 0 if controller is detected, 1 otherwise
scan_for_controller() {
    local scan_output

    # Run a timed scan - bluetoothctl will output discovered devices
    # We use timeout to limit the scan duration
    scan_output=$(timeout "$SCAN_DURATION" bluetoothctl --timeout "$SCAN_DURATION" scan on 2>&1) || true

    # Check if our controller MAC appeared in the scan output
    if echo "$scan_output" | grep -qi "$CONTROLLER_MAC"; then
        return 0
    fi

    # Also check if controller is now showing as available (might have been cached)
    if bluetoothctl devices 2>/dev/null | grep -qi "$CONTROLLER_MAC"; then
        return 0
    fi

    return 1
}

# Attempt to connect to the controller
attempt_connect() {
    local current_time
    current_time=$(date +%s)

    # Enforce cooldown period after successful connection
    local time_since_connect=$((current_time - LAST_CONNECT_TIME))
    if [[ $time_since_connect -lt $CONNECT_COOLDOWN ]]; then
        return 0
    fi

    log_info "Attempting to connect to controller $CONTROLLER_MAC..."

    # Ensure controller is trusted for auto-reconnect
    if ! is_trusted; then
        log_info "Trusting controller..."
        bluetoothctl trust "$CONTROLLER_MAC" 2>/dev/null || true
    fi

    # Attempt connection with timeout
    if timeout 10 bluetoothctl connect "$CONTROLLER_MAC" 2>&1 | grep -q "Connection successful"; then
        log_info "Successfully connected to Stadia controller!"
        LAST_CONNECT_TIME=$(date +%s)
        return 0
    else
        # Double-check connection status (sometimes the message varies)
        sleep 1
        if is_connected; then
            log_info "Successfully connected to Stadia controller!"
            LAST_CONNECT_TIME=$(date +%s)
            return 0
        else
            log_warn "Connection attempt failed"
            return 1
        fi
    fi
}

# Wait for bluetooth service to be ready
wait_for_bluetooth() {
    log_info "Waiting for bluetooth service..."
    local attempts=0
    local max_attempts=30

    while [[ $attempts -lt $max_attempts ]]; do
        if systemctl is-active --quiet bluetooth; then
            log_info "Bluetooth service is active"
            # Give it a moment to fully initialize
            sleep 2
            return 0
        fi
        ((attempts++))
        sleep 1
    done

    log_error "Bluetooth service did not become active after ${max_attempts}s"
    return 1
}

# Main daemon loop
main() {
    log_info "=== Stadia Controller Auto-Connect Daemon Starting ==="
    log_info "Controller MAC: $CONTROLLER_MAC"
    log_info "Scan interval: ${SCAN_INTERVAL}s, Scan duration: ${SCAN_DURATION}s"

    # Wait for bluetooth to be ready
    if ! wait_for_bluetooth; then
        log_error "Cannot start without bluetooth service"
        exit 1
    fi

    # Check if controller is paired
    if ! is_paired; then
        log_warn "Controller is not paired! Please pair first using bluetoothctl"
        log_warn "Continuing anyway - will attempt connection when controller is in pairing mode"
    fi

    log_info "Entering main loop - scanning for controller..."

    while $RUNNING; do
        # Check if already connected
        if is_connected; then
            # Already connected - just wait and check again
            sleep "$SCAN_INTERVAL"
            continue
        fi

        # Not connected - scan for the controller
        if scan_for_controller; then
            log_info "Controller detected - attempting to connect..."
            attempt_connect
        fi

        # Wait before next scan cycle
        sleep "$SCAN_INTERVAL"
    done
}

# Run the daemon
main "$@"
