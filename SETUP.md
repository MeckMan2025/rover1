# Rover1 Setup Guide

## Quick Start

1. **Clone this repository:**
   ```bash
   git clone <your-repo-url>
   cd rover1
   ```

2. **Set up environment variables:**
   ```bash
   cp .env.example .env
   # Edit .env with your actual credentials
   nano .env
   ```

3. **Configure required credentials in .env:**
   - `NTRIP_*`: GPS RTK correction service credentials
   - `ROVER_*`: SSH login credentials for the rover
   - `WIFI_*`: Network credentials for automatic connection

4. **Run setup script on rover:**
   ```bash
   ./scripts/setup_rover.sh
   ```

## Security Notes

- ⚠️ **Never commit the `.env` file** - it contains sensitive credentials
- ✅ Use `.env.example` as a template for required variables  
- 🔒 Change all default passwords before deployment
- 📝 The `.gitignore` file ensures `.env` stays local

## Network Configuration

4-tier failover chain (see [docs/network.md](docs/network.md) for the full story):

1. Home WiFi (`Lake Wifi`)
2. Phone hotspot (`AJM17ProMax`)
3. SIM7600G-H LTE (always-on backup on wwan0, see [LTE_FAILOVER.md](LTE_FAILOVER.md))
4. Rover-as-AP (`rover1-direct`) — broadcast when nothing else is in range

Set the WiFi credentials (including `WIFI_AP_SSID` / `WIFI_AP_PASS`) in `.env`, then on the rover:

```bash
sudo ./scripts/setup_wifi_failover.sh
```

The installer creates / refreshes all NetworkManager profiles and enables the `wifi-failover.timer`. It also retires the legacy `wifi-roam.*` units it supersedes.

## Remote Access

For remote access via Tailscale, see [TAILSCALE_SETUP.md](TAILSCALE_SETUP.md).