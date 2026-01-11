# Tailscale Setup for Rover Remote Access

## Overview
Tailscale creates a secure mesh network that allows you to access your rover from anywhere, even when you're not on the same WiFi network.

## Initial Setup

### On Your Mac (Control Machine)
1. **Start Tailscale service:**
   ```bash
   brew services start tailscale
   ```

2. **Connect to your Tailscale network:**
   ```bash
   tailscale up
   ```
   - This will open a browser window for authentication
   - Sign in with your Tailscale account

### On Your Rover
1. **Install Tailscale on the rover:**
   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh
   ```

2. **Start Tailscale on the rover:**
   ```bash
   sudo tailscale up
   ```
   - Follow the authentication link provided

## Connecting to Your Rover

Once both devices are on the Tailscale network:

1. **Check connected devices:**
   ```bash
   tailscale status
   ```

2. **Find your rover's Tailscale IP:**
   - Look for your rover in the status output
   - Note the 100.x.x.x IP address

3. **Connect via SSH:**
   ```bash
   ssh user@100.x.x.x
   ```
   Replace `user` with your rover's username and `100.x.x.x` with the actual Tailscale IP

4. **Access ROS2 services remotely:**
   ```bash
   export ROS_DOMAIN_ID=0
   export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
   ros2 node list
   ```

## Useful Commands

- **View network status:** `tailscale status`
- **Disconnect:** `tailscale down`
- **Reconnect:** `tailscale up`
- **View logs:** `tailscale debug`

## Troubleshooting

- If connection fails, ensure both devices are authenticated and showing as "online" in `tailscale status`
- Check firewall settings on the rover
- Verify ROS2 environment variables are set correctly for remote access