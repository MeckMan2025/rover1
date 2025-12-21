# Rover1

**Repo**: https://github.com/MeckMan2025/rover1
**Status**: Active Development

## Overview
Rover1 is a DIY autonomy kit designed to demonstrate artificial intelligence and robotics.
Features:
- Teleop Drive (Web UI)
- Waypoint Recording (RTK GPS)
- Autonomous Patrol

## Tech Stack
- Raspberry Pi 5
- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- U-Blox ZED-F9R
- BerryIMU v3
- Hiwonder Motor Module

## Setup
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MeckMan2025/rover1.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch rover1_bringup rover.launch.py
```
