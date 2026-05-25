#!/bin/bash
# wifi-failover.sh — 4-tier network priority chain for the rover.
#
# Tier 1: Home WiFi   (STA, e.g. "Lake Wifi")
# Tier 2: Phone hotspot (STA, e.g. "AJM17ProMax")
# Tier 3: SIM7600G-H LTE (wwan0, independent)
# Tier 4: Rover-as-AP  (e.g. "rover1-direct") — fallback so the dashboard
#         is reachable in the field even with no WiFi infrastructure.
#
# Tier 3 lives on wwan0 and is not managed by this script — ModemManager
# brings it up at boot and the kernel route metrics (wlan0=600, wwan0=700)
# keep LTE as the always-on backup. See LTE_FAILOVER.md.
#
# This script only touches wlan0: pick the highest-priority STA that's
# in range; if none, drop into AP mode so a phone can connect directly.
#
# Config: /etc/wifi-failover.conf (written by setup_wifi_failover.sh)
# Triggered every 60 s by wifi-failover.timer.

set -euo pipefail

CONF=/etc/wifi-failover.conf
if [ ! -r "$CONF" ]; then
    echo "[wifi-failover] missing config $CONF — run setup_wifi_failover.sh" >&2
    exit 1
fi
# shellcheck source=/dev/null
source "$CONF"

: "${WIFI_HOME_SSID:?WIFI_HOME_SSID not set in $CONF}"
: "${WIFI_HOTSPOT_SSID:?WIFI_HOTSPOT_SSID not set in $CONF}"
: "${WIFI_AP_CONN_NAME:?WIFI_AP_CONN_NAME not set in $CONF}"

# Higher priority first.
PRIORITY_SSIDS=("$WIFI_HOME_SSID" "$WIFI_HOTSPOT_SSID")

# How long to keep the AP up before forcing a re-scan to look for known
# STAs we'd prefer. The AP briefly drops during the scan, so don't make
# this aggressive — 5 min is usually enough to pick up a returning network
# without annoying anyone driving over the local AP.
AP_RESCAN_INTERVAL_SEC=${AP_RESCAN_INTERVAL_SEC:-300}
STATE_DIR=/run/wifi-failover
mkdir -p "$STATE_DIR"
LAST_AP_SCAN_FILE="$STATE_DIR/last_ap_scan"

log() {
    logger -t wifi-failover "$1"
    echo "[wifi-failover] $1"
}

# Active NM connection profile name on wlan0 (or empty).
current_conn() {
    nmcli -t -f NAME,DEVICE,TYPE connection show --active 2>/dev/null \
        | awk -F: -v dev=wlan0 '$2==dev && $3=="802-11-wireless" {print $1; exit}'
}

is_ap_active() {
    [ "$(current_conn)" = "$WIFI_AP_CONN_NAME" ]
}

# NM profile name that owns a given SSID, or empty.
conn_for_ssid() {
    local target="$1" name ssid
    while IFS=: read -r name _; do
        ssid=$(nmcli -t -g 802-11-wireless.ssid connection show "$name" 2>/dev/null || true)
        if [ "$ssid" = "$target" ]; then
            echo "$name"
            return 0
        fi
    done < <(nmcli -t -f NAME,TYPE connection show 2>/dev/null \
                | awk -F: '$2=="802-11-wireless" {print $0}')
    return 0
}

scan_ssids() {
    nmcli device wifi rescan 2>/dev/null || true
    sleep 2
    nmcli -t -f SSID device wifi list ifname wlan0 2>/dev/null \
        | sort -u | grep -v '^$' || true
}

# AP mode interferes with scanning on some drivers. Briefly drop the AP,
# scan, then the caller decides whether to bring it back.
drop_ap_for_scan() {
    log "AP rescan window: dropping AP for fresh scan"
    nmcli connection down "$WIFI_AP_CONN_NAME" >/dev/null 2>&1 || true
    sleep 3
    date +%s > "$LAST_AP_SCAN_FILE"
    scan_ssids
}

main() {
    local now available best_sta best_conn cur
    now=$(date +%s)
    cur=$(current_conn)
    log "current wlan0 connection: ${cur:-none}"

    if is_ap_active; then
        local last_scan=0
        [ -f "$LAST_AP_SCAN_FILE" ] && last_scan=$(cat "$LAST_AP_SCAN_FILE")
        local since=$(( now - last_scan ))
        if (( since < AP_RESCAN_INTERVAL_SEC )); then
            log "AP up; next rescan in $(( AP_RESCAN_INTERVAL_SEC - since ))s"
            exit 0
        fi
        available=$(drop_ap_for_scan)
    else
        available=$(scan_ssids)
    fi

    log "in-range SSIDs: $(echo "$available" | tr '\n' ',' | sed 's/,$//')"

    best_sta=""
    for ssid in "${PRIORITY_SSIDS[@]}"; do
        if echo "$available" | grep -qFx "$ssid"; then
            best_sta="$ssid"
            break
        fi
    done

    if [ -n "$best_sta" ]; then
        best_conn=$(conn_for_ssid "$best_sta")
        if [ -z "$best_conn" ]; then
            log "SSID '$best_sta' in range but no NM profile; skipping"
            best_sta=""
        fi
    fi

    if [ -n "$best_sta" ]; then
        if [ "$cur" = "$best_conn" ]; then
            log "already on best STA: '$best_sta'"
        else
            log "switching to STA: '$best_sta' (profile '$best_conn')"
            nmcli connection up "$best_conn" 2>&1 | sed 's/^/  /' || \
                log "STA up failed — leaving wlan0 as-is"
        fi
        rm -f "$LAST_AP_SCAN_FILE"
    else
        if [ "$cur" = "$WIFI_AP_CONN_NAME" ]; then
            log "no STA available; staying on AP '$WIFI_AP_CONN_NAME'"
        else
            log "no STA available; bringing up AP '$WIFI_AP_CONN_NAME'"
            nmcli connection up "$WIFI_AP_CONN_NAME" 2>&1 | sed 's/^/  /' || \
                log "AP up failed — wlan0 is offline"
            date +%s > "$LAST_AP_SCAN_FILE"
        fi
    fi
}

main "$@"
