#!/bin/bash
# Kiosk Mode Setup for Rover 7" Touchscreen
# Sets up auto-boot into fullscreen web dashboard
#
# Hardware: Raspberry Pi 5, Ubuntu 24.04, Hosyond 7" IPS (1024x600)
# Touch: QDTECH MPI7002
#
# NOTE: Uses shell profile approach (not systemd) because Wayland compositors
# like cage need VT/seat access that only comes from a login session.

set -e

echo "=== Rover Kiosk Mode Setup ==="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
USER_NAME="andrewmeckley"
USER_HOME="/home/$USER_NAME"

# Check if running as root (we need sudo for some operations)
if [ "$EUID" -eq 0 ]; then
    echo "Please run without sudo. Script will prompt for sudo when needed."
    exit 1
fi

echo ""
echo "1. Installing kiosk dependencies..."
sudo apt-get update
sudo apt-get install -y \
    cage \
    wlr-randr \
    chromium-browser \
    libinput-tools

echo ""
echo "2. Creating Chromium kiosk wrapper script..."
mkdir -p "$USER_HOME/.config/rover-kiosk"

cat > "$USER_HOME/.config/rover-kiosk/launch-kiosk.sh" << 'KIOSK_SCRIPT'
#!/bin/bash
# Rover Kiosk Launcher
# Waits for dashboard, then launches fullscreen Chromium

DASHBOARD_URL="http://localhost:8080"
MAX_WAIT=60
WAIT_COUNT=0

echo "[Kiosk] Waiting for dashboard at $DASHBOARD_URL..."

# Wait for the web dashboard to be available
while ! curl -s --head "$DASHBOARD_URL" > /dev/null 2>&1; do
    sleep 2
    WAIT_COUNT=$((WAIT_COUNT + 2))
    if [ $WAIT_COUNT -ge $MAX_WAIT ]; then
        echo "[Kiosk] Warning: Dashboard not responding after ${MAX_WAIT}s, launching anyway..."
        break
    fi
    echo "[Kiosk] Waiting... (${WAIT_COUNT}s)"
done

echo "[Kiosk] Launching Chromium in kiosk mode..."

# Launch Chromium with kiosk flags
exec chromium-browser \
    --kiosk \
    --noerrdialogs \
    --disable-infobars \
    --disable-translate \
    --no-first-run \
    --disable-features=TranslateUI \
    --check-for-update-interval=31536000 \
    --disable-session-crashed-bubble \
    --disable-restore-session-state \
    --disable-background-networking \
    --disable-sync \
    --disable-default-apps \
    --disable-extensions \
    --disable-component-update \
    --autoplay-policy=no-user-gesture-required \
    --start-fullscreen \
    --window-size=1024,600 \
    --window-position=0,0 \
    "$DASHBOARD_URL"
KIOSK_SCRIPT

chmod +x "$USER_HOME/.config/rover-kiosk/launch-kiosk.sh"

echo ""
echo "3. Setting up autologin on tty1..."
# Create autologin override for getty@tty1
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
sudo tee /etc/systemd/system/getty@tty1.service.d/autologin.conf > /dev/null << EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $USER_NAME --noclear %I \$TERM
EOF

echo ""
echo "4. Disabling old systemd kiosk service (if exists)..."
# Clean up old systemd approach (doesn't work - cage needs login session)
sudo systemctl disable kiosk.service 2>/dev/null || true
sudo systemctl stop kiosk.service 2>/dev/null || true

echo ""
echo "5. Configuring shell profile to launch kiosk on tty1..."
# Add kiosk launch to .bash_profile (runs on login, not SSH)
KIOSK_MARKER="# >>> ROVER KIOSK MODE >>>"
KIOSK_END_MARKER="# <<< ROVER KIOSK MODE <<<"
BASHRC_MARKER="# Source .bashrc for interactive shell settings"

# Ensure .bash_profile sources .bashrc (for colors, PATH, aliases in SSH sessions)
if [ ! -f "$USER_HOME/.bash_profile" ] || ! grep -q "source.*\.bashrc" "$USER_HOME/.bash_profile"; then
    echo "   Adding .bashrc sourcing to .bash_profile..."
    cat > "$USER_HOME/.bash_profile.tmp" << 'BASHRC_SOURCE'
# Source .bashrc for interactive shell settings (colors, PATH, aliases)
# This ensures SSH sessions get the same environment as local terminals
if [ -f ~/.bashrc ]; then
    source ~/.bashrc
fi

BASHRC_SOURCE
    # Prepend to existing .bash_profile or create new one
    if [ -f "$USER_HOME/.bash_profile" ]; then
        cat "$USER_HOME/.bash_profile" >> "$USER_HOME/.bash_profile.tmp"
    fi
    mv "$USER_HOME/.bash_profile.tmp" "$USER_HOME/.bash_profile"
fi

# Remove old kiosk block if exists
if [ -f "$USER_HOME/.bash_profile" ]; then
    sed -i "/$KIOSK_MARKER/,/$KIOSK_END_MARKER/d" "$USER_HOME/.bash_profile"
fi

# Add kiosk launch block
cat >> "$USER_HOME/.bash_profile" << 'PROFILE_BLOCK'

# >>> ROVER KIOSK MODE >>>
# Launch kiosk on tty1 only (not SSH sessions)
# Cage needs to run from login session for VT/seat access
if [ "$(tty)" = "/dev/tty1" ] && [ -z "$SSH_CONNECTION" ]; then
    echo "Waiting for rover1.service..."
    systemctl is-active --wait rover1.service 2>/dev/null || true
    sleep 3  # Extra time for web dashboard to initialize

    echo "Launching kiosk mode..."
    exec cage ~/.config/rover-kiosk/launch-kiosk.sh
fi
# <<< ROVER KIOSK MODE <<<
PROFILE_BLOCK

echo ""
echo "=== Kiosk Setup Complete ==="
echo ""
echo "The kiosk will start automatically on next boot (tty1 only)."
echo "SSH sessions are unaffected - you can still SSH in normally."
echo ""
echo "How it works:"
echo "  1. Autologin on tty1 -> runs .bash_profile"
echo "  2. .bash_profile waits for rover1.service"
echo "  3. cage launches Chromium in kiosk mode"
echo ""
echo "To access terminal while kiosk is running:"
echo "  Press Ctrl+Alt+F2 to switch to tty2"
echo "  Press Ctrl+Alt+F1 to switch back to kiosk"
echo ""
echo "To disable kiosk:"
echo "  ./scripts/kiosk_disable.sh"
echo ""
echo "Reboot to test: sudo reboot"
