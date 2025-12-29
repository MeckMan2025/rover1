#!/bin/bash
# Kiosk Mode Setup for Rover 7" Touchscreen
# Sets up auto-boot into fullscreen web dashboard
#
# Hardware: Raspberry Pi 5, Ubuntu 24.04, Hosyond 7" IPS (1024x600)
# Touch: QDTECH MPI7002

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
echo "3. Installing kiosk systemd service..."
sudo cp "$SCRIPT_DIR/kiosk.service" /etc/systemd/system/kiosk.service
sudo systemctl daemon-reload

echo ""
echo "4. Setting up autologin on tty1..."
# Create autologin override for getty@tty1
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
sudo tee /etc/systemd/system/getty@tty1.service.d/autologin.conf > /dev/null << EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $USER_NAME --noclear %I \$TERM
EOF

echo ""
echo "5. Configuring display environment..."
# Create environment file for Wayland/Cage
cat > "$USER_HOME/.config/rover-kiosk/environment" << 'ENV_FILE'
# Kiosk display environment
export WLR_LIBINPUT_NO_DEVICES=1
export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-1
export WLR_BACKENDS=drm
export WLR_DRM_DEVICES=/dev/dri/card1
ENV_FILE

echo ""
echo "6. Enabling kiosk service..."
sudo systemctl enable kiosk.service

echo ""
echo "=== Kiosk Setup Complete ==="
echo ""
echo "The kiosk will start automatically on next boot."
echo ""
echo "Commands:"
echo "  Start now:     sudo systemctl start kiosk"
echo "  Stop:          sudo systemctl stop kiosk"
echo "  Status:        sudo systemctl status kiosk"
echo "  Disable:       sudo systemctl disable kiosk"
echo "  View logs:     journalctl -u kiosk -f"
echo ""
echo "Reboot to test: sudo reboot"
