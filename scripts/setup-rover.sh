#!/bin/bash
# Complete Rover Deployment Script
# Configures brain-selector architecture for autonomous rover deployment
#
# Usage:
#   sudo ./scripts/setup-rover.sh [username] 
#   sudo ./scripts/setup-rover.sh --uninstall [username]
#
# Default username: andrewmeckley

set -e

# Configuration
DEFAULT_USER="andrewmeckley"
BACKUP_DIR="/home/$DEFAULT_USER/rover-setup-backups/$(date +%Y%m%d_%H%M%S)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

usage() {
    cat << EOF
Rover Setup Script - Configures brain-selector architecture

USAGE:
    sudo ./scripts/setup-rover.sh [username]
    sudo ./scripts/setup-rover.sh --uninstall [username]

OPTIONS:
    username      User to configure (default: andrewmeckley)
    --uninstall   Remove all rover configurations and restore backups

EXAMPLE:
    sudo ./scripts/setup-rover.sh                    # Use default user
    sudo ./scripts/setup-rover.sh robotuser         # Custom user
    sudo ./scripts/setup-rover.sh --uninstall       # Remove configuration

This script configures:
- Auto-login on tty1 for physical touchscreen
- Brain-selector launch on physical display
- VNC server for remote access
- Disables rover1.service auto-start
- Installs required dependencies
EOF
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

backup_file() {
    local file="$1"
    if [[ -f "$file" ]]; then
        mkdir -p "$BACKUP_DIR"
        cp "$file" "$BACKUP_DIR/$(basename "$file").backup"
        log_info "Backed up $file to $BACKUP_DIR"
    fi
}

install_dependencies() {
    log_info "Installing required dependencies..."
    
    # Update package list
    apt update
    
    # Install PyQt5, YAML, and Cage (Wayland kiosk compositor)
    apt install -y python3-pyqt5 python3-yaml cage
    
    # Install VNC server if not present
    if ! command -v vncserver-virtual &> /dev/null; then
        apt install -y realvnc-vnc-server
    fi
    
    log_success "Dependencies installed"
}

configure_tty_autologin() {
    local username="$1"
    log_info "Configuring tty1 auto-login for user: $username"
    
    local override_dir="/etc/systemd/system/getty@tty1.service.d"
    local override_file="$override_dir/override.conf"
    
    # Backup existing configuration
    backup_file "$override_file"
    
    # Create override directory
    mkdir -p "$override_dir"
    
    # Create auto-login configuration
    cat > "$override_file" << EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $username --noclear %I \$TERM
EOF
    
    log_success "Configured tty1 auto-login for $username"
}

configure_bash_profile() {
    local username="$1"
    local user_home="/home/$username"
    local bash_profile="$user_home/.bash_profile"
    
    log_info "Configuring bash profile for brain-selector launch"
    
    # Backup existing bash profile
    backup_file "$bash_profile"
    
    # Create or update bash profile
    if [[ ! -f "$bash_profile" ]]; then
        touch "$bash_profile"
        chown "$username:$username" "$bash_profile"
    fi
    
    # Remove existing brain-selector configuration
    sed -i '/# Brain Selector/,/^fi$/d' "$bash_profile"
    
    # Add brain-selector launch configuration
    cat >> "$bash_profile" << 'EOF'

# Brain Selector - Launch on tty1 (physical touchscreen only, not SSH)
if [ "$(tty)" = "/dev/tty1" ] && [ -z "$SSH_CONNECTION" ]; then
    exec ~/.config/brain-selector/launch-selector-tty.sh
fi
EOF
    
    chown "$username:$username" "$bash_profile"
    log_success "Configured bash profile brain-selector launch"
}

configure_cage_launcher() {
    local username="$1"
    local user_home="/home/$username"
    local config_dir="$user_home/.config/brain-selector"
    local launcher_script="$config_dir/launch-selector-tty.sh"
    
    log_info "Creating Cage (Wayland) launcher script"
    
    # Create config directory
    mkdir -p "$config_dir"
    
    # Backup existing launcher
    backup_file "$launcher_script"
    
    # Create launcher script
    cat > "$launcher_script" << EOF
#!/bin/bash
export XDG_RUNTIME_DIR=/run/user/\$(id -u)
export WLR_LIBINPUT_NO_DEVICES=1
echo "\$(date): Starting Brain Selector on physical display" >> /tmp/brain-selector.log
exec cage -s -- python3 /home/$username/rover-brains/brain-selector/selector.py 2>> /tmp/brain-selector.log
EOF
    
    chmod +x "$launcher_script"
    chown -R "$username:$username" "$config_dir"
    log_success "Created Cage launcher script"
}

configure_vnc_autostart() {
    local username="$1"
    local user_home="/home/$username"
    local autostart_dir="$user_home/.config/autostart"
    local desktop_file="$autostart_dir/brain-selector.desktop"
    
    log_info "Configuring VNC autostart for brain-selector"
    
    # Create autostart directory
    mkdir -p "$autostart_dir"
    
    # Backup existing desktop file
    backup_file "$desktop_file"
    
    # Create desktop entry
    cat > "$desktop_file" << EOF
[Desktop Entry]
Type=Application
Name=Brain Selector
Exec=/home/$username/rover-brains/brain-selector/selector.py
Terminal=false
EOF
    
    chown -R "$username:$username" "$autostart_dir"
    log_success "Configured VNC autostart"
}

configure_vnc_service() {
    local username="$1"
    local service_file="/etc/systemd/system/vncserver-brain.service"
    
    log_info "Configuring VNC systemd service"
    
    # Backup existing service
    backup_file "$service_file"
    
    # Create VNC service
    cat > "$service_file" << EOF
[Unit]
Description=RealVNC Server for Brain Selector
After=multi-user.target network.target

[Service]
Type=forking
User=$username
ExecStart=/usr/bin/vncserver-virtual :1 -geometry 1024x600 -depth 24
ExecStop=/usr/bin/vncserver-virtual -kill :1
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF
    
    # Reload systemd and enable service
    systemctl daemon-reload
    systemctl enable vncserver-brain.service
    
    log_success "Configured VNC systemd service"
}

disable_rover_autostart() {
    log_info "Disabling rover1.service auto-start"
    
    if systemctl is-enabled rover1.service &> /dev/null; then
        systemctl disable rover1.service
        log_success "Disabled rover1.service"
    else
        log_info "rover1.service already disabled"
    fi
    
    if systemctl is-active rover1.service &> /dev/null; then
        systemctl stop rover1.service
        log_success "Stopped rover1.service"
    fi
}

uninstall_configuration() {
    local username="$1"
    local user_home="/home/$username"
    
    log_warning "Uninstalling rover configuration..."
    
    # Stop and disable services
    if systemctl is-active vncserver-brain.service &> /dev/null; then
        systemctl stop vncserver-brain.service
    fi
    if systemctl is-enabled vncserver-brain.service &> /dev/null; then
        systemctl disable vncserver-brain.service
    fi
    
    # Remove service file
    rm -f /etc/systemd/system/vncserver-brain.service
    
    # Remove tty1 auto-login override
    rm -rf /etc/systemd/system/getty@tty1.service.d
    
    # Restore bash profile from backup
    local latest_backup
    if [[ -d "/home/$username/rover-setup-backups" ]]; then
        latest_backup=$(find "/home/$username/rover-setup-backups" -name ".bash_profile.backup" | sort | tail -1)
        if [[ -f "$latest_backup" ]]; then
            cp "$latest_backup" "$user_home/.bash_profile"
            chown "$username:$username" "$user_home/.bash_profile"
            log_success "Restored bash profile from backup"
        fi
    fi
    
    # Remove brain-selector configuration
    rm -rf "$user_home/.config/brain-selector"
    rm -f "$user_home/.config/autostart/brain-selector.desktop"
    
    # Re-enable rover1.service if it exists
    if [[ -f "/etc/systemd/system/rover1.service" ]]; then
        systemctl enable rover1.service
        log_success "Re-enabled rover1.service"
    fi
    
    systemctl daemon-reload
    log_success "Rover configuration uninstalled"
}

print_summary() {
    local username="$1"
    
    cat << EOF

${GREEN}=== Rover Setup Complete ===${NC}

Configuration Summary:
${BLUE}User:${NC} $username
${BLUE}Backup Location:${NC} $BACKUP_DIR

${BLUE}Physical Display (tty1):${NC}
- Auto-login configured for $username
- Brain-selector launches automatically via Cage (Wayland)

${BLUE}VNC Access:${NC}
- VNC server runs on :1 (port 5901)
- Brain-selector auto-starts in VNC session
- Connect via: rover1.local:5901 (password: rover123)

${BLUE}Boot Sequence:${NC}
- rover1.service disabled (no auto-start)
- User controls rover startup via brain-selector GUI

${BLUE}Next Steps:${NC}
1. Reboot to test configuration: ${YELLOW}sudo reboot${NC}
2. Verify physical display shows brain-selector
3. Test VNC access from remote computer
4. Ensure rover brain switching works correctly

${BLUE}Emergency Recovery:${NC}
- SSH access preserved: ssh $username@rover1.local
- Uninstall: sudo $0 --uninstall $username
- Manual recovery: systemctl enable rover1.service

${BLUE}Logs:${NC}
- Brain selector: /tmp/brain-selector.log
- VNC service: journalctl -u vncserver-brain.service
EOF
}

main() {
    local uninstall=false
    local username="$DEFAULT_USER"
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --uninstall)
                uninstall=true
                shift
                ;;
            --help|-h)
                usage
                exit 0
                ;;
            -*)
                log_error "Unknown option: $1"
                usage
                exit 1
                ;;
            *)
                username="$1"
                shift
                ;;
        esac
    done
    
    # Validate user exists
    if ! id "$username" &>/dev/null; then
        log_error "User '$username' does not exist"
        exit 1
    fi
    
    check_root
    
    if [[ "$uninstall" == true ]]; then
        uninstall_configuration "$username"
        log_success "Rover configuration removed for user: $username"
        return
    fi
    
    log_info "Setting up rover for user: $username"
    log_info "Backup directory: $BACKUP_DIR"
    
    # Main setup sequence
    install_dependencies
    configure_tty_autologin "$username"
    configure_bash_profile "$username"
    configure_cage_launcher "$username"
    configure_vnc_autostart "$username"
    configure_vnc_service "$username"
    disable_rover_autostart
    
    print_summary "$username"
}

# Run main function with all arguments
main "$@"