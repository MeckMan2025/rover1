# LTE Failover for Rover1

## Overview
The rover has a USB cellular modem that provides an automatic backup uplink when WiFi drops. Combined with Tailscale (see `TAILSCALE_SETUP.md`), this means the rover stays reachable from your phone or laptop over LTE without needing a hotspot or AP-mode build.

## Hardware
- **Modem**: SIMCOM SIM7600G-H (USB ID `1e0e:9001`), global 4G LTE + 3G + 2G fallback.
- **SIM**: EIOTCLUB data-only IoT SIM. Roams on AT&T in the US (operator ID `310410`).
- **APN**: `wbdata` (pre-configured in the NM profile and the modem's initial bearer).
- **Antenna**: External LTE antenna on the MAIN port is **required** — without it the modem reports 0% signal even with good local coverage. Keep the dongle outside the rover chassis or on an antenna lead routed outside the chassis.

## Software stack
- `ModemManager` (system service) — talks to the modem over QMI on `/dev/cdc-wdm1`.
- `NetworkManager` profile `EIOTCLUB-LTE` (GSM type, autoconnect=yes) — brings up `wwan0` once the modem registers.
- `usb_modeswitch` + `usb-modeswitch-data` packages — installed; not needed for this SimCom module but kept for compatibility with other dongles.

## How the failover works
Both connections run simultaneously. The kernel picks the default route by metric (lower = preferred):

| Interface | Default-route metric | Role |
|-----------|---------------------|------|
| `wlan0`   | 600                 | Primary |
| `wwan0`   | 700                 | Backup |

- While WiFi is up, outbound traffic uses WiFi. LTE sits idle (~KB/min of Tailscale keepalives).
- When WiFi drops, the WiFi default route is withdrawn and the LTE default route is the only one left — outbound traffic transparently switches. Expect a 5–15s hiccup while Tailscale re-establishes its path.
- When WiFi comes back, its lower-metric route is re-added and traffic shifts back automatically. No manual action needed.

## Reaching the rover from your phone
The LTE side is carrier-NAT'd, so you cannot reach the rover's `10.176.103.x` IP directly. Use **Tailscale**:

1. Install Tailscale on your phone, log in to the same tailnet.
2. Hit `http://100.121.159.107:8080` (the rover's stable Tailscale IP).
3. Works whether the rover is on home WiFi, a hotspot, or LTE.

## Bring-up checklist
On a fresh boot, everything should come up on its own:

1. Modem enumerates on USB (check with `lsusb | grep 1e0e`).
2. ModemManager probes it (`mmcli -L` shows `SIMCOM_SIM7600G-H`).
3. NetworkManager auto-activates `EIOTCLUB-LTE` (`nmcli connection show --active` lists it on `cdc-wdm1`).
4. `wwan0` gets a carrier-NAT IP (`ip -br addr show wwan0`).

If LTE never comes up, the most common cause is `ModemManager` being inactive. Verify:
```bash
systemctl is-enabled ModemManager  # should be "enabled"
systemctl is-active ModemManager   # should be "active"
```
Both should be enabled and active. Service was enabled on 2026-05-25.

## Diagnostic one-liners
```bash
# Full modem health (signal, registration, operator)
mmcli -m 0 | grep -E 'state:|signal quality|registration|operator'

# Confirm both default routes are present and ordered correctly
ip route | grep default

# Watch live signal during antenna placement
watch -n 2 "mmcli -m 0 | grep 'signal quality'"
```

## Known gotchas
- **0% signal = antenna issue 9 times out of 10.** Move the dongle outside the chassis or attach an external LTE antenna to MAIN. The chassis is a partial Faraday cage.
- **`lock: sim-pin2`** in `mmcli` output is **not** a problem. PIN2 governs fixed dialing and call-cost limits, not data registration.
- The `connection.autoconnect-priority` is 0 for LTE. If you ever want LTE to *preempt* WiFi (e.g., for testing), bump its priority above the WiFi profile or temporarily lower its route metric.
- Cell data is **not free** — while WiFi is up, almost nothing flows over LTE, but if WiFi drops for hours, watch your data usage. Tailscale keepalives are tiny; live video streaming over LTE is not.
