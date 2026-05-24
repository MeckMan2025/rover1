#!/bin/bash
# Install the rover touchscreen kiosk + auto-starting FastAPI server.
#
# What this does
# --------------
# 1. Stops any manually-launched uvicorn (port 8080 collision).
# 2. Installs the user systemd unit at ~/.config/systemd/user/rover-uvicorn.service.
# 3. Enables and starts the unit (lingering enabled so it survives logout).
# 4. Installs the kiosk launcher script at ~/.config/rover-kiosk/launch-kiosk.sh.
# 5. Appends a tty1 autostart hook to ~/.bash_profile (idempotent).
#
# Run from the repo root on the rover:
#     scripts/install_kiosk.sh
#
# Rollback
# --------
#   systemctl --user disable --now rover-uvicorn.service
#   rm -rf ~/.config/rover-kiosk ~/.config/systemd/user/rover-uvicorn.service
#   cp ~/.bash_profile.pre-simplify ~/.bash_profile     # restore pre-quarantine state
#   sudo loginctl disable-linger "$USER"

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

say() { printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }

say "1. Stop any manual uvicorn"
if pgrep -f "uvicorn.*app.main" >/dev/null 2>&1; then
    pkill -f "uvicorn.*app.main" || true
    sleep 1
    if pgrep -f "uvicorn.*app.main" >/dev/null 2>&1; then
        echo "ERROR: could not stop existing uvicorn. Kill it manually and re-run."
        exit 1
    fi
    echo "stopped"
else
    echo "none running"
fi

say "2. Install systemd user unit"
mkdir -p "$HOME/.config/systemd/user"
cp "$REPO_ROOT/systemd/rover-uvicorn.service" "$HOME/.config/systemd/user/rover-uvicorn.service"
systemctl --user daemon-reload

say "3. Enable + start unit, enable linger"
systemctl --user enable --now rover-uvicorn.service
if ! loginctl show-user "$USER" 2>/dev/null | grep -qi "^Linger=yes"; then
    sudo loginctl enable-linger "$USER"
fi

say "4. Install kiosk launcher"
mkdir -p "$HOME/.config/rover-kiosk"
install -m 755 "$REPO_ROOT/scripts/launch-kiosk.sh" "$HOME/.config/rover-kiosk/launch-kiosk.sh"

say "5. Patch ~/.bash_profile (idempotent)"
BP="$HOME/.bash_profile"
MARKER_START="# >>> rover-kiosk autostart >>>"
MARKER_END="# <<< rover-kiosk autostart <<<"

# Remove any prior block bounded by the markers so re-runs replace cleanly.
if grep -q "$MARKER_START" "$BP" 2>/dev/null; then
    sed -i "/$MARKER_START/,/$MARKER_END/d" "$BP"
    echo "removed previous block"
fi

cat >> "$BP" <<EOF

$MARKER_START
# Auto-launch the rover kiosk on tty1 (installed $(date +%Y-%m-%d) by scripts/install_kiosk.sh).
if [ "\$(tty)" = "/dev/tty1" ] && [ -z "\$SSH_CONNECTION" ]; then
    exec \$HOME/.config/rover-kiosk/launch-kiosk.sh
fi
$MARKER_END
EOF

if bash -n "$BP"; then
    echo "~/.bash_profile syntax OK"
else
    echo "ERROR: ~/.bash_profile failed syntax check; restoring from .pre-simplify backup"
    cp "$HOME/.bash_profile.pre-simplify" "$BP"
    exit 1
fi

say "Done. Quick checks:"
echo "  systemctl --user is-active rover-uvicorn.service  -> $(systemctl --user is-active rover-uvicorn.service)"
echo "  curl localhost:8080/telemetry                     -> $(curl -sf --max-time 2 http://localhost:8080/telemetry || echo FAILED)"
echo
echo "Reboot to verify the touchscreen comes up to the dashboard cleanly."
