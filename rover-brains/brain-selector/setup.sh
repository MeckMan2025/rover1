#!/bin/bash
# Brain Selector Setup Script
# Installs and configures the brain selector for Raspberry Pi 5 with touchscreen

set -e

USER_HOME="/home/andrewmeckley"
BRAIN_DIR="$USER_HOME/rover-brains"
SELECTOR_DIR="$BRAIN_DIR/brain-selector"
CONFIG_DIR="$USER_HOME/.config/brain-selector"

echo "=== Brain Selector Setup ==="
echo "Installing brain selector for touchscreen boot menu"
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

# Install dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y python3-pyqt5 python3-yaml cage

# Create config directory
echo "Creating config directory..."
mkdir -p "$CONFIG_DIR"

# Copy launch script to config directory
echo "Setting up launch wrapper..."
cp "$SELECTOR_DIR/launch-selector.sh" "$CONFIG_DIR/"
chmod +x "$CONFIG_DIR/launch-selector.sh"
chmod +x "$SELECTOR_DIR/selector.py"

# Backup existing bash_profile
if [ -f "$USER_HOME/.bash_profile" ]; then
    echo "Backing up existing .bash_profile..."
    cp "$USER_HOME/.bash_profile" "$USER_HOME/.bash_profile.backup.$(date +%Y%m%d_%H%M%S)"
fi

# Check if brain selector entry already exists
if grep -q "brain-selector" "$USER_HOME/.bash_profile" 2>/dev/null; then
    echo "Brain selector entry already exists in .bash_profile"
else
    echo "Adding brain selector to .bash_profile..."
    cat >> "$USER_HOME/.bash_profile" << 'EOF'

# Brain Selector - Launch on tty1 (non-SSH sessions only)
if [ "$(tty)" = "/dev/tty1" ] && [ -z "$SSH_CONNECTION" ]; then
    exec ~/.config/brain-selector/launch-selector.sh
fi
EOF
fi

# Create brain directories if they don't exist
echo "Setting up brain directories..."
mkdir -p "$BRAIN_DIR/rover1"
mkdir -p "$BRAIN_DIR/rover2"

# Set proper permissions
echo "Setting permissions..."
chown -R andrewmeckley:andrewmeckley "$BRAIN_DIR"
chown -R andrewmeckley:andrewmeckley "$CONFIG_DIR"
chmod +x "$CONFIG_DIR/launch-selector.sh"
chmod +x "$SELECTOR_DIR/selector.py"

echo
echo "=== Setup Complete ==="
echo
echo "Brain Selector has been installed successfully!"
echo
echo "Configuration:"
echo "  - Brain selector: $SELECTOR_DIR/selector.py"
echo "  - Launch wrapper: $CONFIG_DIR/launch-selector.sh"
echo "  - Brain directory: $BRAIN_DIR"
echo "  - Logs: /tmp/brain-selector.log"
echo
echo "Next steps:"
echo "  1. Add brain.yaml files to brain subdirectories"
echo "  2. Reboot to test (selector will auto-start on tty1)"
echo "  3. SSH access remains available during operation"
echo
echo "Testing:"
echo "  - Test selector: python3 $SELECTOR_DIR/selector.py"
echo "  - Test with Cage: cage $SELECTOR_DIR/selector.py"
echo
echo "Emergency recovery:"
echo "  - SSH in and run: sudo systemctl isolate multi-user.target"
echo "  - Or edit .bash_profile to disable auto-launch"
echo

# === UNINSTALL INSTRUCTIONS ===
cat << 'UNINSTALL_EOF'

=== UNINSTALL INSTRUCTIONS ===
To remove the brain selector:

1. Remove from .bash_profile:
   sed -i '/# Brain Selector/,/fi/d' ~/.bash_profile

2. Remove config directory:
   rm -rf ~/.config/brain-selector

3. Optionally remove brain directories:
   rm -rf ~/rover-brains/brain-selector

4. Restore backup if needed:
   cp ~/.bash_profile.backup.XXXXXX_XXXXXX ~/.bash_profile

UNINSTALL_EOF

echo "Setup script completed successfully!"
echo "Reboot when ready to test the brain selector."