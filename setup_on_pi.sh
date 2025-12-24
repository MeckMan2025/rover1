#!/bin/bash
set -e

USER_PASS="Orion2021!"

echo ">>> Starting Rover1 Setup on Pi..."

run_sudo() {
    echo "$USER_PASS" | sudo -S -p "" "$@"
}

# 1. Install Dependencies
echo ">>> Installing System Dependencies..."
export DEBIAN_FRONTEND=noninteractive
run_sudo apt update
# Added python3-colcon-common-extensions and mavros-msgs
run_sudo apt install -y ros-jazzy-ublox-dgnss ros-jazzy-ntrip-client python3-smbus2 i2c-tools git python3-colcon-common-extensions ros-jazzy-mavros-msgs ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-robot-localization ros-jazzy-sensor-msgs ros-jazzy-foxglove-bridge

# 2. Setup Workspace
echo ">>> Setting up ROS 2 Workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

if [ -d "rover1" ]; then
    echo ">>> Repo exists, pulling latest..."
    cd rover1
    git pull
else
    echo ">>> Cloning Rover1..."
    git clone https://github.com/MeckMan2025/rover1.git
fi

# 3. Build
echo ">>> Building Workspace..."
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 4. Setup Auto-Updater
echo ">>> Installing Auto-Updater Service..."
cd ~/ros2_ws/src/rover1/scripts
chmod +x auto_update.sh
run_sudo cp rover1-updater.service /etc/systemd/system/
run_sudo cp rover1-updater.timer /etc/systemd/system/
run_sudo systemctl daemon-reload
run_sudo systemctl enable rover1-updater.timer
run_sudo systemctl start rover1-updater.timer

# 5. I2C Permission & Udev Rules
run_sudo usermod -aG i2c $USER
echo ">>> Setting up U-Blox Udev Rules..."
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666"' | run_sudo tee /etc/udev/rules.d/99-ublox.rules
run_sudo udevadm control --reload-rules && run_sudo udevadm trigger

echo ">>> Setup Complete! Please reboot the Pi or log out/in to apply permissions."
echo ">>> To launch: ros2 launch rover1_bringup rover.launch.py"
