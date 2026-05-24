# Rover2: Personal Pack Robot

**Status**: Ready for Testing  
**Mission**: Autonomous pack robot that follows you using computer vision

---

## 🚀 Overview

Rover2 is a simplified, focused version of rover1 designed as a personal pack robot. It follows you around using computer vision, allowing you to load it with items and walk to your destination while it autonomously follows behind.

## ✨ Key Features

### **Core Functionality**
- **Person Detection & Following**: Uses YOLOv8 on Hailo-8L AI accelerator to detect and follow humans
- **Bounding Box Tracking**: Maintains following distance by centering person in camera frame
- **Outdoor Optimized**: Designed for single-person outdoor use
- **Simple Workflow**: Load items → walk → rover follows → unload

### **Hardware**
- **Brain**: Raspberry Pi 5
- **OS**: Ubuntu Server 24.04 (ROS 2 Jazzy) 
- **Camera**: Nuwa60C with computer vision processing
- **Sensors**: U-Blox ZED-F9R (RTK GPS), BerryIMU v3 (Orientation)
- **Mobility**: Mecanum holonomic drive for agility

### **Interface**
- **Web Dashboard**: Live camera feed with person-following controls
- **Teleop Control**: WASD keyboard or touch controls for manual driving
- **GPS Tracking**: Real-time location display and RTK accuracy monitoring
- **Safety Features**: Teleop override, person loss detection, automatic stopping

---

## 🏗 Architecture

### **Package Structure**
```
rover2/
├── rover2_bringup/          # Launch files and configuration
├── rover2_hardware/         # Hardware drivers (GPS, IMU, motors, battery)
├── rover2_vision/           # Person detection and following (person_follower.py)  
├── rover2_dashboard/        # Web interface for control and monitoring
└── rover2_project_plan.md   # Development plan and requirements
```

### **Key Components**
- **person_follower.py**: Main AI node using Hailo-8L for real-time person detection
- **Web Dashboard**: Simplified UI with person-follow controls (no patrol complexity)
- **Hardware Drivers**: Reused from rover1 (GPS, IMU, motor control, battery monitoring)

---

## 🚀 Quick Start

### **1. Build**
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select rover2_bringup rover2_hardware rover2_vision rover2_dashboard
source install/setup.bash
```

### **2. Launch**
```bash
# Start the full rover2 system
ros2 launch rover2_bringup rover2.launch.py

# Or start without person following
ros2 launch rover2_bringup rover2.launch.py enable_person_follower:=false
```

### **3. Access Dashboard**
Open web browser to: `http://rover2.local:8080`

### **4. Use**
1. **Manual Control**: Use WASD keys or touch controls to drive
2. **Person Following**: 
   - Toggle "Detect" to enable person detection
   - Click "Follow" to start following mode
   - Walk around - rover follows automatically
   - Click "Stop" to disable following

---

## 🎛 Dashboard Controls

### **Person Follower**
- **Detect Toggle**: Enable/disable person detection and camera processing
- **Follow Button**: Start autonomous person following
- **Stop Button**: Stop following (keeps detection on for visualization)
- **Reset Button**: Reset all state to idle

### **Drive Controls**
- **WASD**: Manual driving (automatically overrides person following)
- **Speed Slider**: Adjust teleop speed (10-100%)
- **Status**: Shows current mode (idle, detecting, following, etc.)

### **Monitoring**
- **Live Camera**: Real-time feed with person bounding boxes
- **GPS**: Location, RTK status, satellite count
- **System**: Battery voltage, connection status

---

## ⚙️ Configuration

### **Person Following Parameters**
Located in `rover2.launch.py`:
```python
'confidence_threshold': 0.5,        # Person detection confidence
'target_foot_y_ratio': 0.70,        # Target distance (foot position in frame)
'linear_speed': 0.4,                # Max forward speed (m/s)
'angular_speed': 0.8,               # Max rotation speed (rad/s)
'center_tolerance': 0.12,           # Centering deadzone (12% of frame)
'detection_timeout': 2.0,           # Stop if person lost for 2s
'recovery_scan_timeout': 4.0        # Give up recovery scan after 4s
```

### **Camera Setup**
- Uses Nuwa60C camera at `/ascamera_nuwa/camera_publisher/rgb0/image`
- 640x480 resolution at 10fps for optimal AI performance
- YOLOv8 model at `/home/andrewmeckley/ros2_ws/src/rover2/models/yolov8s.hef`

---

## 🔧 Differences from Rover1

### **Removed Features**
- ❌ Patrol/waypoint following system
- ❌ Teach & repeat autonomy
- ❌ Nav2 navigation stack
- ❌ Complex mission planning
- ❌ Dog following mode

### **Simplified Features**
- ✅ **Person-only tracking** (COCO class 0)
- ✅ **Streamlined dashboard** (no patrol UI complexity)
- ✅ **Focused launch files** (rover2.launch.py)
- ✅ **Essential hardware only** (GPS, IMU, motors, camera)

### **Kept Features**
- ✅ RTK GPS for accurate positioning
- ✅ Web dashboard with live camera feed
- ✅ Teleop control via keyboard/touch
- ✅ Battery monitoring and system health
- ✅ Robust hardware drivers

---

## 🐛 Troubleshooting

### **Person Following Issues**
- **Not detecting person**: Check camera feed, ensure good lighting, verify Hailo-8L model path
- **Following erratically**: Adjust `center_tolerance` and speed parameters
- **Loses person easily**: Increase `detection_timeout` or check confidence threshold

### **Camera Issues**
- **No video feed**: Verify Nuwa60C camera connection and `ascamera_nuwa` namespace
- **Poor detection**: Ensure proper lighting, check YOLOv8 model is loaded

### **General Issues**
- **Dashboard won't load**: Check `rover2_web_dashboard` node is running
- **GPS no fix**: Verify antenna placement, check RTK corrections
- **Motor not responding**: Verify Hiwonder driver parameters and I2C connection

---

## 📋 Service Commands

```bash
# Enable person detection
ros2 service call /person_follower/detection_enable std_srvs/srv/Trigger

# Start following
ros2 service call /person_follower/enable std_srvs/srv/Trigger

# Stop following  
ros2 service call /person_follower/disable std_srvs/srv/Trigger

# Reset to idle
ros2 service call /person_follower/reset std_srvs/srv/Trigger
```

---

**Rover2**: Your autonomous pack companion. Simple. Focused. Reliable.
---

## Thermal budget (passive cooling)

This rover runs **without an active cooler**. Idle SoC sits around 78 °C; the
Pi 5 soft-throttles at 80 °C. Treat thermal as a hard constraint, not a
tunable.

- Boot config caps `arm_freq=2000` — see [`boot/`](boot/). Don't raise it.
- Check temperature anytime: `vcgencmd measure_temp`
- Check for throttle events since boot: `vcgencmd get_throttled`
  - `0x0` → never throttled (good)
  - non-zero → a thermal (or undervoltage) event occurred — investigate workload
- If temp climbs past 78 °C during a mission: disconnect remote
  `/video.mjpg` viewers (drops the encoder to 6 Hz) or pause driving for a
  minute to let it coast back down.
- Longer-term mitigations: physical airflow over the SoC, larger passive
  heatsink, or relocate the Pi to a less enclosed spot in the chassis.
