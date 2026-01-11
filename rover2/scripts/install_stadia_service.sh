#!/bin/bash
#
# Stadia Controller Auto-Connect Service Installer
# Installs and enables the systemd service for automatic controller connection
#
# Author: Rover1 Project
# Date: 2025-12-26
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SERVICE_NAME="stadia-controller"
CONTROLLER_MAC="D1:71:42:54:CB:0F"

# Logging functions
info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        error "This script should not be run as root. Run as your regular user."
        exit 1
    fi
}

# Check if bluetooth is available
check_bluetooth() {
    if ! command -v bluetoothctl &> /dev/null; then
        error "bluetoothctl not found. Please install bluetooth utilities:"
        echo "  sudo apt update && sudo apt install bluetooth bluez"
        exit 1
    fi
    
    if ! systemctl is-enabled bluetooth &> /dev/null; then
        warning "Bluetooth service is not enabled. Enabling it..."
        sudo systemctl enable bluetooth
    fi
}

# Verify controller is paired
check_controller_paired() {
    info "Checking if Stadia controller is paired..."
    
    if bluetoothctl info "$CONTROLLER_MAC" &> /dev/null; then
        success "Controller $CONTROLLER_MAC is paired"
        
        # Check if trusted
        if bluetoothctl info "$CONTROLLER_MAC" | grep -q "Trusted: yes"; then
            success "Controller is trusted"
        else
            warning "Controller is not trusted. You may want to run:"
            echo "  bluetoothctl trust $CONTROLLER_MAC"
        fi
    else
        error "Controller $CONTROLLER_MAC is not paired!"
        echo ""
        echo "Please pair your controller first:"
        echo "  1. Put controller in pairing mode (Stadia + Y buttons)"
        echo "  2. Run: bluetoothctl"
        echo "  3. In bluetoothctl: scan on"
        echo "  4. Wait for device to appear, then: pair $CONTROLLER_MAC"
        echo "  5. Run: trust $CONTROLLER_MAC"
        echo "  6. Exit bluetoothctl and re-run this installer"
        exit 1
    fi
}

# Install the service
install_service() {
    info "Installing Stadia controller auto-connect service..."
    
    # Copy script to final location (already in correct location)
    if [[ ! -f "$SCRIPT_DIR/connect_stadia_controller.sh" ]]; then
        error "connect_stadia_controller.sh not found in $SCRIPT_DIR"
        exit 1
    fi
    
    # Make sure script is executable
    chmod +x "$SCRIPT_DIR/connect_stadia_controller.sh"
    success "Connection script is ready"
    
    # Install systemd service
    if [[ ! -f "$PROJECT_ROOT/systemd/$SERVICE_NAME.service" ]]; then
        error "Service file not found: $PROJECT_ROOT/systemd/$SERVICE_NAME.service"
        exit 1
    fi
    
    sudo cp "$PROJECT_ROOT/systemd/$SERVICE_NAME.service" "/etc/systemd/system/"
    sudo chown root:root "/etc/systemd/system/$SERVICE_NAME.service"
    sudo chmod 644 "/etc/systemd/system/$SERVICE_NAME.service"
    success "Service file installed to /etc/systemd/system/"
    
    # Reload systemd and enable service
    sudo systemctl daemon-reload
    sudo systemctl enable "$SERVICE_NAME"
    success "Service enabled for auto-start on boot"
}

# Test the service
test_service() {
    info "Testing the service..."
    
    # Start the service
    if sudo systemctl start "$SERVICE_NAME"; then
        success "Service started successfully"
        
        # Wait a moment and check status
        sleep 5
        if systemctl is-active --quiet "$SERVICE_NAME"; then
            success "Service is running"
            
            # Check if controller connected
            if bluetoothctl info "$CONTROLLER_MAC" | grep -q "Connected: yes"; then
                success "Controller is connected!"
            else
                warning "Service is running but controller not connected. Check logs:"
                echo "  sudo journalctl -u $SERVICE_NAME -f"
            fi
        else
            warning "Service exited. Check status:"
            echo "  sudo systemctl status $SERVICE_NAME"
        fi
    else
        error "Failed to start service. Check logs:"
        echo "  sudo journalctl -u $SERVICE_NAME"
    fi
}

# Show usage information
show_info() {
    echo ""
    success "Installation complete!"
    echo ""
    echo "Useful commands:"
    echo "  Start service:      sudo systemctl start $SERVICE_NAME"
    echo "  Stop service:       sudo systemctl stop $SERVICE_NAME"
    echo "  Service status:     sudo systemctl status $SERVICE_NAME"
    echo "  View logs:          sudo journalctl -u $SERVICE_NAME"
    echo "  View live logs:     sudo journalctl -u $SERVICE_NAME -f"
    echo "  Disable service:    sudo systemctl disable $SERVICE_NAME"
    echo ""
    echo "Log file: /var/log/stadia-controller.log"
    echo ""
    echo "The service will automatically start on boot and attempt to connect your"
    echo "Stadia controller. Reboot to test the full auto-connect functionality."
}

# Main installation process
main() {
    echo ""
    info "Stadia Controller Auto-Connect Service Installer"
    echo "================================================"
    echo ""
    
    check_root
    check_bluetooth
    check_controller_paired
    install_service
    test_service
    show_info
}

# Run installer
main "$@"