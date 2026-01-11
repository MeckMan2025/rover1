# Rover2 - Pack Robot

**Repo**: https://github.com/MeckMan2025/rover2  
**Status**: Pack Robot Implementation Complete

## Overview
Rover2 is a minimal "Pack Robot" designed to follow humans using computer vision - essentially a mobile cargo carrier that follows its human colleague.

**Core Features:**
- **Person Following:** Uses Hailo-8L AI accelerator + YOLOv8 for real-time human detection and following
- **Web Teleop:** Mandatory web-based manual control (WASD keyboard + touch controls)  
- **Safety Systems:** Battery monitoring, emergency stop, teleop override protection
- **Minimal Footprint:** No SLAM, GPS, Nav2 bloat - just essential mobility components

**What's Excluded:** SLAM, GPS/RTK navigation, Nav2, patrol systems, EKF sensor fusion, Foxglove bridge

## Tech Stack
- **Hardware:** Raspberry Pi 5, Nuwa-HP60C Camera, Hailo-8L AI Accelerator, Hiwonder Motor Controller  
- **Software:** Ubuntu 24.04 (Noble), ROS 2 Jazzy, YOLOv8 person detection
- **Mobility:** Mecanum wheel kinematics for omnidirectional movement
- **Interface:** Web dashboard with mandatory teleop controls

## Quick Start

```bash
# Launch the pack robot system
./launch_pack_robot.sh

# Access web interface
# http://rover-ip:8080
```

## Components

### Essential Components (Always Running)
- **Motors:** `rover2_hardware/hiwonder_driver` + `mecanum_kinematics` 
- **Camera:** `ascamera` (Nuwa-HP60C for vision input)
- **Battery:** `rover2_hardware/battery_monitor` (safety critical)
- **Dashboard:** `rover2_dashboard/pack_robot_dashboard` (web teleop + controls)

### Optional Components (User Controlled)  
- **Person Follower:** `rover2_vision/person_follower` (enable via web dashboard)

## Safety Features
- **Emergency Stop:** Always-visible red button stops all movement instantly
- **Teleop Override:** Manual control automatically disables person following  
- **Battery Monitoring:** Warns of low voltage, recommends shutdown on critical levels
- **Detection/Following Separation:** Visual tracking can run without movement

## Pack Robot vs Rover1
- **Rover1:** Full autonomous rover (SLAM + GPS waypoints + patrol + vision)  
- **Rover2:** Minimal pack robot (vision-based human following only)

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
