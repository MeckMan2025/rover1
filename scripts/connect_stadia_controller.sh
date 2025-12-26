#!/bin/bash
#
# Stadia Controller Auto-Connect Script
# Automatically connects to paired Stadia controller on boot
#
# Author: Rover1 Project
# Date: 2025-12-26
#

# Configuration
CONTROLLER_MAC="D1:71:42:54:CB:0F"
CONTROLLER_NAME="StadiaWG58-cb0f"
MAX_RETRIES=10
RETRY_DELAY=3
LOG_FILE="/var/log/stadia-controller.log"

# Logging function
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Check if bluetooth service is running
check_bluetooth_service() {
    if systemctl is-active --quiet bluetooth; then
        log "Bluetooth service is running"
        return 0
    else
        log "ERROR: Bluetooth service is not running"
        return 1
    fi
}

# Check if controller is already connected
is_controller_connected() {
    bluetoothctl info "$CONTROLLER_MAC" 2>/dev/null | grep -q "Connected: yes"
}

# Attempt to connect to controller
connect_controller() {
    local attempt=1
    
    log "Starting Stadia controller connection attempts..."
    
    while [ $attempt -le $MAX_RETRIES ]; do
        log "Attempt $attempt/$MAX_RETRIES: Connecting to $CONTROLLER_NAME ($CONTROLLER_MAC)"
        
        # Check if already connected
        if is_controller_connected; then
            log "SUCCESS: Controller is already connected!"
            return 0
        fi
        
        # Try to connect
        if echo "connect $CONTROLLER_MAC" | timeout 10 bluetoothctl; then
            sleep 2
            if is_controller_connected; then
                log "SUCCESS: Controller connected successfully on attempt $attempt"
                return 0
            fi
        fi
        
        log "Attempt $attempt failed, waiting ${RETRY_DELAY}s before retry..."
        sleep $RETRY_DELAY
        ((attempt++))
    done
    
    log "ERROR: Failed to connect controller after $MAX_RETRIES attempts"
    return 1
}

# Main execution
main() {
    log "=== Stadia Controller Auto-Connect Service Started ==="
    
    # Check if bluetooth service is running
    if ! check_bluetooth_service; then
        log "Waiting for bluetooth service..."
        sleep 5
        if ! check_bluetooth_service; then
            log "FATAL: Bluetooth service unavailable, exiting"
            exit 1
        fi
    fi
    
    # Wait a bit for bluetooth to fully initialize
    log "Waiting for bluetooth to fully initialize..."
    sleep 3
    
    # Attempt connection
    if connect_controller; then
        log "=== Stadia Controller Auto-Connect Service Completed Successfully ==="
        exit 0
    else
        log "=== Stadia Controller Auto-Connect Service Failed ==="
        exit 1
    fi
}

# Run main function
main "$@"