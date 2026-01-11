# Rover Setup Guide

This guide explains how to configure the brain-selector architecture for rover deployment. You can use either the automated script or manual configuration.

## Quick Setup (Recommended)

Use the automated setup script:

```bash
cd ~/ros2_ws/src/rover1
sudo ./scripts/setup-rover.sh
```

For custom username:
```bash
sudo ./scripts/setup-rover.sh your_username
```

To uninstall:
```bash
sudo ./scripts/setup-rover.sh --uninstall
```

## Manual Setup

If you prefer manual configuration, follow these steps:

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-yaml cage realvnc-vnc-server
```

### 2. Configure TTY1 Auto-Login

Create the systemd override directory and configuration:

```bash
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
sudo tee /etc/systemd/system/getty@tty1.service.d/override.conf << EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin andrewmeckley --noclear %I \$TERM
EOF
```

### 3. Configure Bash Profile

Add brain-selector launch to `~/.bash_profile`:

```bash
cat >> ~/.bash_profile << 'EOF'

# Brain Selector - Launch on tty1 (physical touchscreen only, not SSH)
if [ "$(tty)" = "/dev/tty1" ] && [ -z "$SSH_CONNECTION" ]; then
    exec ~/.config/brain-selector/launch-selector-tty.sh
fi
EOF
```

### 4. Create Cage Launcher Script

Create the Wayland compositor launcher:

```bash
mkdir -p ~/.config/brain-selector
cat > ~/.config/brain-selector/launch-selector-tty.sh << 'EOF'
#!/bin/bash
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WLR_LIBINPUT_NO_DEVICES=1
echo "$(date): Starting Brain Selector on physical display" >> /tmp/brain-selector.log
exec cage -s -- python3 /home/andrewmeckley/rover-brains/brain-selector/selector.py 2>> /tmp/brain-selector.log
EOF
chmod +x ~/.config/brain-selector/launch-selector-tty.sh
```

### 5. Configure VNC Autostart

Create autostart entry for VNC sessions:

```bash
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/brain-selector.desktop << 'EOF'
[Desktop Entry]
Type=Application
Name=Brain Selector
Exec=/home/andrewmeckley/rover-brains/brain-selector/selector.py
Terminal=false
EOF
```

### 6. Configure VNC Service

Create systemd service for VNC server:

```bash
sudo tee /etc/systemd/system/vncserver-brain.service << 'EOF'
[Unit]
Description=RealVNC Server for Brain Selector
After=multi-user.target network.target

[Service]
Type=forking
User=andrewmeckley
ExecStart=/usr/bin/vncserver-virtual :1 -geometry 1024x600 -depth 24
ExecStop=/usr/bin/vncserver-virtual -kill :1
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable vncserver-brain.service
```

### 7. Disable Auto-Start Service

Disable the automatic rover startup:

```bash
sudo systemctl disable rover1.service
sudo systemctl stop rover1.service
```

## Architecture Overview

### Boot Sequence

```
Power On → Auto-login (tty1) → Brain Selector GUI
                                      ↓
                              User Selects Brain
                                      ↓
                         Launch Selected Rover Configuration
```

### Display Configurations

| Display Type | Method | GUI Framework | Use Case |
|--------------|--------|---------------|----------|
| **Physical Display** | Cage + Wayland | PyQt5 | Primary operator interface |
| **VNC Remote** | VNC Server | PyQt5 | Remote monitoring/control |
| **SSH Terminal** | Command Line | N/A | Emergency access |

### Network Access Methods

| Method | Address | Purpose |
|--------|---------|---------|
| **WiFi** | `rover1.local` | Primary network |
| **Ethernet Tether** | `10.42.0.1` | Emergency fallback |
| **Tailscale VPN** | Mesh IP | Remote access |

## File Locations

### Configuration Files
- **TTY Auto-login**: `/etc/systemd/system/getty@tty1.service.d/override.conf`
- **Bash Profile**: `~/.bash_profile`
- **Cage Launcher**: `~/.config/brain-selector/launch-selector-tty.sh`
- **VNC Autostart**: `~/.config/autostart/brain-selector.desktop`
- **VNC Service**: `/etc/systemd/system/vncserver-brain.service`

### Brain Configurations
- **Rover1 Brain**: `~/rover-brains/rover1/brain.yaml`
- **Pack Robot Brain**: `~/rover-brains/rover2/brain.yaml`
- **Launch Scripts**: `scripts/brain_launch.sh`, `home/andrewmeckley/rover-brains/rover2/launch_pack_robot.sh`

### Log Files
- **Brain Selector**: `/tmp/brain-selector.log`
- **VNC Service**: `journalctl -u vncserver-brain.service`
- **System Boot**: `journalctl -b`

## Testing the Setup

### 1. Physical Display Test

Reboot and verify:
```bash
sudo reboot
```

Expected: Physical display shows brain-selector GUI after auto-login

### 2. VNC Access Test

From remote computer:
```bash
# Install VNC viewer if needed
sudo apt install realvnc-vnc-viewer  # Linux
# Download from realvnc.com for Windows/Mac

# Connect
vncviewer rover1.local:5901
# Password: rover123
```

Expected: VNC session shows brain-selector GUI

### 3. Brain Selection Test

1. Tap "Rover 1" button
2. Verify patrol_lite.launch.py starts
3. Check web dashboard: `http://rover1.local:8080`

### 4. SSH Fallback Test

```bash
ssh andrewmeckley@rover1.local
# or
ssh andrewmeckley@10.42.0.1  # Ethernet tether
```

Expected: Normal SSH session without brain-selector interference

## Troubleshooting

### Brain Selector Won't Start

Check logs:
```bash
cat /tmp/brain-selector.log
journalctl -u getty@tty1.service
```

Common issues:
- Missing dependencies: `sudo apt install python3-pyqt5 cage`
- Permission issues: Check file ownership in `~/.config/`
- Display issues: Verify Wayland/Cage configuration

### VNC Connection Failed

Check VNC service:
```bash
sudo systemctl status vncserver-brain.service
journalctl -u vncserver-brain.service
```

Manual VNC start:
```bash
vncserver-virtual :1 -geometry 1024x600 -depth 24
```

### Auto-login Not Working

Check systemd override:
```bash
sudo systemctl status getty@tty1.service
cat /etc/systemd/system/getty@tty1.service.d/override.conf
```

Verify user configuration:
```bash
# Check if user exists
id andrewmeckley
# Check bash profile
cat ~/.bash_profile
```

### Rover Won't Start

Check if old service is still enabled:
```bash
sudo systemctl status rover1.service
sudo systemctl disable rover1.service
```

Check brain configuration:
```bash
cat ~/rover-brains/rover1/brain.yaml
ls -la ~/ros2_ws/src/rover1/scripts/brain_launch.sh
```

## Emergency Recovery

### Restore Auto-Start

If brain-selector fails, restore original behavior:

```bash
sudo systemctl enable rover1.service
sudo systemctl start rover1.service
```

### Disable Brain Selector

Remove tty1 override:
```bash
sudo rm -rf /etc/systemd/system/getty@tty1.service.d
sudo systemctl daemon-reload
```

### Full Uninstall

Use the automated uninstall:
```bash
sudo ./scripts/setup-rover.sh --uninstall
```

Or manually:
```bash
sudo systemctl disable vncserver-brain.service
sudo rm /etc/systemd/system/vncserver-brain.service
sudo rm -rf /etc/systemd/system/getty@tty1.service.d
rm -rf ~/.config/brain-selector
rm ~/.config/autostart/brain-selector.desktop
# Restore .bash_profile from backup
sudo systemctl enable rover1.service
sudo systemctl daemon-reload
```

## Security Considerations

### Network Safety
- SSH access preserved on all network interfaces
- Emergency ethernet tether always available (10.42.0.1)
- Tailscale VPN provides additional access method

### Physical Safety
- STOP_ROVER file respected by all launch scripts
- Emergency stop accessible via SSH
- Physical display provides direct control

### Service Safety
- VNC server runs as user (not root)
- Brain selector runs as user (not root)
- No additional network services exposed

## Maintenance

### Update Brain Configurations

After modifying brain scripts or launch files:

1. Update the setup script: `scripts/setup-rover.sh`
2. Test on development rover first
3. Deploy to production rovers using the setup script
4. Document changes in this guide

### Backup Configuration

The setup script automatically creates backups in:
```bash
~/rover-setup-backups/YYYYMMDD_HHMMSS/
```

Manual backup:
```bash
mkdir -p ~/rover-config-backup
cp ~/.bash_profile ~/rover-config-backup/
cp -r ~/.config/brain-selector ~/rover-config-backup/
sudo cp -r /etc/systemd/system/getty@tty1.service.d ~/rover-config-backup/
```

### Monitor System Health

Regular checks:
```bash
# Check services
sudo systemctl status vncserver-brain.service
sudo systemctl status getty@tty1.service

# Check logs
tail -f /tmp/brain-selector.log
journalctl -f -u vncserver-brain.service

# Check brain selector process
ps aux | grep brain-selector
```