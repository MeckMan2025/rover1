# Rover Brain Launcher Setup Instructions

## Overview
The Rover Brain Launcher is a touchscreen boot selector that allows choosing which "brain" (codebase) to run on your Raspberry Pi rover at startup.

## Installation

### 1. Copy Files to Raspberry Pi
Copy these files to your Raspberry Pi:
- `brain_launcher.py` → `/home/andrewmeckley/brain_launcher.py`
- `rover-brain-launcher.service` → `/etc/systemd/system/rover-brain-launcher.service`

```bash
# On Raspberry Pi
sudo cp brain_launcher.py /home/andrewmeckley/
sudo cp rover-brain-launcher.service /etc/systemd/system/
sudo chmod +x /home/andrewmeckley/brain_launcher.py
```

### 2. Install Dependencies
```bash
sudo apt update
sudo apt install python3-tk python3-yaml
```

### 3. Create Brains Directory
```bash
mkdir -p /home/andrewmeckley/rover-brains
```

### 4. Enable the Service
```bash
sudo systemctl daemon-reload
sudo systemctl enable rover-brain-launcher.service
```

## Adding New Brains

### 1. Create Brain Directory
```bash
mkdir /home/andrewmeckley/rover-brains/my-brain-name
```

### 2. Add Your Brain Code
Copy your brain's code into the directory or clone a git repository:
```bash
cd /home/andrewmeckley/rover-brains/my-brain-name
# Copy files or clone repo here
```

### 3. Create brain.yaml Manifest
Copy and customize the template:
```bash
cp brain.yaml.template /home/andrewmeckley/rover-brains/my-brain-name/brain.yaml
```

Edit the manifest with your brain's details:
```yaml
name: "My Awesome Brain"
description: "Controls rover navigation and obstacle avoidance"
start_command: "python3 main.py"
```

### 4. Test Your Brain
Test the start command works from the brain directory:
```bash
cd /home/andrewmeckley/rover-brains/my-brain-name
python3 main.py  # or whatever your start_command is
```

## Example: Setting up rover1 Brain

```bash
# Create rover1 brain directory
mkdir /home/andrewmeckley/rover-brains/rover1

# Copy or clone rover1 code (example)
cd /home/andrewmeckley/rover-brains/rover1
# ... copy your rover1 files here ...

# Create brain.yaml
cat > brain.yaml << EOF
name: "Rover1"
description: "System monitoring and basic rover operations"
start_command: "btop"
EOF
```

## Usage

### Starting the Launcher
The launcher starts automatically at boot via systemd. To manually start:
```bash
sudo systemctl start rover-brain-launcher.service
```

### Using the Touchscreen Interface
1. Touch interface shows all available brains
2. Swipe to scroll through the list
3. Tap a brain to select and launch it
4. The selected brain takes over the screen
5. Reboot to return to the launcher

### Stopping the Service
To disable auto-start:
```bash
sudo systemctl disable rover-brain-launcher.service
sudo systemctl stop rover-brain-launcher.service
```

## Troubleshooting

### Check Service Status
```bash
sudo systemctl status rover-brain-launcher.service
```

### View Logs
```bash
# Service logs
journalctl -u rover-brain-launcher.service -f

# Application logs
tail -f /tmp/brain_launcher.log
```

### Common Issues

**Launcher doesn't start at boot:**
- Ensure X server is running: `ps aux | grep X`
- Check systemd service is enabled: `sudo systemctl is-enabled rover-brain-launcher.service`

**Touch not working:**
- Verify touchscreen is detected: `xinput list`
- Check display is set correctly: `echo $DISPLAY`

**Brain doesn't start:**
- Check brain.yaml syntax is valid YAML
- Test start_command manually from brain directory
- Check file permissions

**No brains found:**
- Ensure `/home/andrewmeckley/rover-brains/` exists
- Each brain directory must contain a `brain.yaml` file
- Check brain.yaml file permissions

### Development Mode
To run launcher in windowed mode for testing:
1. Edit `brain_launcher.py`
2. Comment out the fullscreen line: `# self.root.attributes('-fullscreen', True)`
3. Run manually: `python3 /home/andrewmeckley/brain_launcher.py`
4. Press Escape or click "Exit (Dev Mode)" to quit

## File Structure
```
/home/andrewmeckley/
├── brain_launcher.py
└── rover-brains/
    ├── rover1/
    │   ├── brain.yaml
    │   └── [rover1 files...]
    ├── rover2/
    │   ├── brain.yaml
    │   └── [rover2 files...]
    └── my-custom-brain/
        ├── brain.yaml
        └── [brain files...]
```