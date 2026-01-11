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

The rover supports automatic WiFi failover:
1. Home network (highest priority)
2. Phone hotspot (fallback)  
3. Ethernet tether (static fallback)

Configure network credentials in `.env` and run:
```bash
./scripts/setup_network_failover.sh
```

## Remote Access

For remote access via Tailscale, see [TAILSCALE_SETUP.md](TAILSCALE_SETUP.md).