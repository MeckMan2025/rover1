#!/bin/bash
# Rover touchscreen kiosk launcher.
#
# Invoked from ~/.bash_profile when user andrewmeckley autologins on tty1.
# Wraps Chromium in app mode inside the `cage` Wayland kiosk compositor,
# pointed at the local FastAPI dashboard (http://localhost:8080).
#
# Replaces the legacy brain-selector launcher at
# ~/.config/brain-selector/launch-selector-tty.sh.

set -e

# Wait for the FastAPI server to be reachable. The user systemd unit
# (rover-uvicorn.service) starts it after default.target, but on a cold boot
# the camera init can take a couple of seconds before /telemetry returns.
echo "kiosk: waiting for FastAPI on localhost:8080..."
for _ in $(seq 1 60); do
    if curl -sf --max-time 1 http://localhost:8080/telemetry >/dev/null 2>&1; then
        echo "kiosk: server ready"
        break
    fi
    sleep 0.5
done

# cage = single-window Wayland compositor for kiosks. Chromium runs inside it
# in app mode (no URL bar, no tabs) and fullscreen via --kiosk.
#
# Pointed at /qr instead of / so the kiosk is a passive viewer (no WebSocket).
# The phone is the sole drive client — scan the QR to connect from your phone.
exec cage -- chromium-browser \
    --kiosk \
    --noerrdialogs \
    --disable-infobars \
    --no-first-run \
    --disable-pinch \
    --overscroll-history-navigation=0 \
    --disable-features=TranslateUI \
    --app=http://localhost:8080/qr
