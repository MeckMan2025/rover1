# Network failover

The rover picks an uplink from a 4-tier priority chain and falls back when higher tiers go away.

| Tier | Source | Interface | Notes |
|------|--------|-----------|-------|
| 1 | Home WiFi (`Lake Wifi`) | wlan0 STA | autoconnect priority 100 |
| 2 | Phone hotspot (`AJM17ProMax`) | wlan0 STA | autoconnect priority 50 |
| 3 | SIM7600G-H LTE | wwan0 | always-on backup, route metric 700 (vs wlan0 600) |
| 4 | Rover-as-AP (`rover1-direct`) | wlan0 AP | fallback when no STA in range; **no internet** |

Tier 3 (LTE) lives on a separate interface and is always-on whenever the modem is enumerated — see `LTE_FAILOVER.md`. Tiers 1, 2, and 4 all share the `wlan0` radio and are managed by `scripts/wifi-failover.sh`, which runs every 60 s.

## How the script decides

1. Scan for available SSIDs.
2. If the home SSID is in range → ensure that profile is active; drop the AP if it was up.
3. Else if the hotspot SSID is in range → activate it; drop the AP.
4. Else → activate the AP profile so a phone can connect locally to drive the rover.

When the AP is active, scanning is unreliable on the Pi 5 driver. The script forces a brief AP drop every `AP_RESCAN_INTERVAL_SEC` (default 300 s) to give known STAs a chance to re-appear. Anyone connected to the AP sees a ~5 s blip every 5 min during that window.

## Install / re-install

```bash
# On the rover. Idempotent — safe to re-run after editing .env.
cd ~/ros2_ws/src/rover1
sudo ./scripts/setup_wifi_failover.sh
```

The installer:

- Reads `WIFI_HOME_*`, `WIFI_HOTSPOT_*`, `WIFI_AP_*` from `.env`.
- Creates / refreshes NetworkManager profiles for all three.
- Writes `/etc/wifi-failover.conf` (runtime SSIDs).
- Installs `wifi-failover.sh` to `/usr/local/bin` and enables `wifi-failover.timer`.
- Cleans up the legacy `wifi-roam.{service,timer}` it replaces.

## Verifying

```bash
# Watch the script live
journalctl -t wifi-failover -f

# Current wlan0 state
nmcli -t -f NAME,DEVICE,TYPE connection show --active | grep wlan0

# Force a check
sudo systemctl start wifi-failover.service && journalctl -t wifi-failover --since '1 min ago'

# Confirm AP profile parameters
nmcli connection show rover1-ap | grep -E 'ssid|mode|method|address'
```

## Using the AP fallback

When you see `rover1-direct` (or whatever you set `WIFI_AP_SSID` to) in the WiFi list on your phone:

1. Join it with the password from `WIFI_AP_PASS`.
2. Open `http://192.168.42.1:8080` in the browser.
3. The dashboard works as usual. You won't have internet through the rover — it's a peer connection only.

The rover will leave AP mode automatically once a known STA SSID is in range again (max 5 min, or the next scan tick after you bring up your hotspot).

## Caveats

- **Single-mode radio.** AP and STA share `wlan0`. The rover can't be both an AP and a client at the same time — picking AP means no upstream WiFi (LTE still works via wwan0). Concurrent AP+STA via a virtual `ap0` interface is possible on Pi 5 but deferred.
- **AP doesn't bridge LTE.** If you join the AP, you're in a closed network with just the rover. Local-only is intentional — bridging carrier-NAT'd LTE to a hotspot tends to confuse Tailscale and isn't worth the complexity.
- **Wi-Fi scan during AP is fragile.** That's why we force a periodic AP drop. If you change `AP_RESCAN_INTERVAL_SEC` in `/etc/wifi-failover.conf`, redeploy doesn't rewrite it — the installer's default only lands on a fresh install.
- **Remote access into AP mode = no.** AP-mode is by definition a "no internet" scenario, so Tailscale is also down. You have to be physically near the rover to use this tier.

## Reverting to the pre-AP setup

If we ever want to roll back:

```bash
sudo systemctl disable --now wifi-failover.timer
sudo rm /etc/systemd/system/wifi-failover.{service,timer}
sudo rm /usr/local/bin/wifi-failover.sh /etc/wifi-failover.conf
sudo nmcli connection delete rover1-ap
sudo systemctl daemon-reload
```

The two STA profiles (`Lake Wifi`, `AJM17ProMax`) stay configured, so the rover keeps roaming between them via NetworkManager's own logic.
