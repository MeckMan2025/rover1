# Stadia Controller Auto-Connect Service

This service automatically connects your paired Stadia Bluetooth controller when your Raspberry Pi rover boots up.

## Features

- 🔄 Automatic connection attempts with configurable retries
- 📝 Detailed logging for troubleshooting
- 🛡️ Security-hardened systemd service
- ⚡ Fast startup after bluetooth service is ready
- 🔧 Easy installation and management

## Prerequisites

- Ubuntu 24.04 (or compatible Linux distribution)
- Bluetooth utilities installed (`bluetooth` and `bluez` packages)
- Stadia controller already paired, bonded, and trusted

## Controller Information

- **MAC Address**: `D1:71:42:54:CB:0F`
- **Name**: `StadiaWG58-cb0f`
- **Model**: Stadia Wireless Controller

## Installation

### 1. Ensure Controller is Paired

If your controller isn't already paired:

```bash
# Put controller in pairing mode (hold Stadia + Y buttons)
bluetoothctl
> scan on
> pair D1:71:42:54:CB:0F
> trust D1:71:42:54:CB:0F
> exit
```

### 2. Install the Service

From your rover1 project directory:

```bash
cd ~/ros2_ws/src/rover1
git pull  # Get latest updates
./scripts/install_stadia_service.sh
```

The installer will:
- ✅ Verify bluetooth is available
- ✅ Check that your controller is paired and trusted
- ✅ Install the connection script and systemd service
- ✅ Enable auto-start on boot
- ✅ Test the service

### 3. Reboot to Test

```bash
sudo reboot
```

After reboot, your Stadia controller should automatically connect within 10-15 seconds.

## Service Management

### Basic Commands

```bash
# Check service status
sudo systemctl status stadia-controller

# Start service manually
sudo systemctl start stadia-controller

# Stop service
sudo systemctl stop stadia-controller

# Enable auto-start on boot (done during installation)
sudo systemctl enable stadia-controller

# Disable auto-start
sudo systemctl disable stadia-controller
```

### Viewing Logs

```bash
# View recent logs
sudo journalctl -u stadia-controller

# Follow logs in real-time
sudo journalctl -u stadia-controller -f

# View detailed log file
tail -f /var/log/stadia-controller.log
```

## Configuration

### Customizing Settings

Edit the connection script to modify behavior:

```bash
nano ~/ros2_ws/src/rover1/scripts/connect_stadia_controller.sh
```

Key settings:
- `MAX_RETRIES=10` - Number of connection attempts
- `RETRY_DELAY=3` - Seconds between attempts
- `CONTROLLER_MAC` - Your controller's MAC address

After making changes, restart the service:

```bash
sudo systemctl restart stadia-controller
```

### Different Controller

To use a different controller:

1. Find your controller's MAC address:
   ```bash
   bluetoothctl devices
   ```

2. Update the MAC address in the script:
   ```bash
   nano ~/ros2_ws/src/rover1/scripts/connect_stadia_controller.sh
   # Change CONTROLLER_MAC="D1:71:42:54:CB:0F" to your MAC
   ```

3. Restart the service:
   ```bash
   sudo systemctl restart stadia-controller
   ```

## Troubleshooting

### Controller Not Connecting

1. **Check if controller is paired:**
   ```bash
   bluetoothctl info D1:71:42:54:CB:0F
   ```

2. **Check bluetooth service:**
   ```bash
   sudo systemctl status bluetooth
   ```

3. **View service logs:**
   ```bash
   sudo journalctl -u stadia-controller -f
   ```

4. **Test manual connection:**
   ```bash
   bluetoothctl connect D1:71:42:54:CB:0F
   ```

### Service Won't Start

1. **Check service status:**
   ```bash
   sudo systemctl status stadia-controller
   ```

2. **Verify files exist:**
   ```bash
   ls -la ~/ros2_ws/src/rover1/scripts/connect_stadia_controller.sh
   ls -la /etc/systemd/system/stadia-controller.service
   ```

3. **Reload systemd:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl restart stadia-controller
   ```

### Permission Issues

If you see permission errors:

```bash
# Ensure script is executable
chmod +x ~/ros2_ws/src/rover1/scripts/connect_stadia_controller.sh

# Check log file permissions
sudo touch /var/log/stadia-controller.log
sudo chown andrewmeckley:andrewmeckley /var/log/stadia-controller.log
```

## Uninstallation

To remove the service:

```bash
# Stop and disable service
sudo systemctl stop stadia-controller
sudo systemctl disable stadia-controller

# Remove service file
sudo rm /etc/systemd/system/stadia-controller.service

# Reload systemd
sudo systemctl daemon-reload

# Optional: Remove log file
sudo rm /var/log/stadia-controller.log
```

## Files Created

- `scripts/connect_stadia_controller.sh` - Main connection script
- `systemd/stadia-controller.service` - Systemd service unit file
- `scripts/install_stadia_service.sh` - Installation script
- `/etc/systemd/system/stadia-controller.service` - Installed service file
- `/var/log/stadia-controller.log` - Service log file

## Support

Check the project repository for updates and report issues:
- Repository: `rover1` project
- Logs: `/var/log/stadia-controller.log`
- System logs: `sudo journalctl -u stadia-controller`