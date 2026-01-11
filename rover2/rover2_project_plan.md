# Rover2 Project Plan - Pack Robot Knowledge Base

**CRITICAL REFERENCE FOR ALL CODE AGENTS: READ THIS FIRST**

This document defines the complete intent, scope, and implementation strategy for Rover2. Any code agent working on this codebase must understand these fundamentals to avoid scope creep or misalignment.

---

## 🎯 **Project Definition**

**Rover2 = "Pack Robot"**
- **Purpose:** Minimal autonomous robot that follows humans using computer vision
- **Role:** Mobile cargo carrier that acts as a human companion/assistant
- **Scope:** MINIMAL - Only essential components for human following functionality

**NOT a general-purpose autonomous rover. NOT a clone of Rover1.**

---

## 🚫 **What Rover2 IS NOT**

**Rover2 explicitly EXCLUDES these systems:**
- ❌ SLAM (RTAB-Map, mapping, localization)
- ❌ GPS/RTK/NTRIP navigation  
- ❌ Nav2 path planning
- ❌ Patrol/waypoint systems
- ❌ EKF sensor fusion
- ❌ Foxglove bridge
- ❌ GNSS health monitoring
- ❌ Stadia controller support
- ❌ Complex web dashboard features

**Why excluded:** Pack robot doesn't need mapping, GPS navigation, or complex autonomy. It only needs to follow a human visually.

---

## ✅ **What Rover2 IS**

**Core Pack Robot Components (ONLY THESE):**

### **Essential Hardware Drivers**
- **Motors:** `rover2_hardware/hiwonder_driver` + `mecanum_kinematics`
- **Camera:** `ascamera` (Nuwa-HP60C for vision input)  
- **Battery:** `rover2_hardware/battery_monitor` (safety critical)

### **Core Functionality**
- **Person Following:** `rover2_vision/person_follower` (main pack robot feature)
- **Web Teleop:** `rover2_dashboard/pack_robot_dashboard` (mandatory manual control)

### **Safety Systems**
- Emergency stop (always-visible red button)
- Teleop override (manual control disables following)
- Battery monitoring with warnings
- Detection/following separation (can track without moving)

---

## 🏗️ **Architecture Overview**

**Package Structure:**
```
rover2_bringup/          # Minimal launch configuration
├── pack_robot.launch.py # MAIN LAUNCH FILE - only essential components

rover2_hardware/         # Essential drivers only  
├── hiwonder_driver.py   # Motor control via I2C
├── mecanum_kinematics.py # Twist → wheel speeds
└── battery_monitor.py   # Safety monitoring

rover2_vision/           # Core pack robot functionality
└── person_follower.py   # Human detection & following (Hailo-8L + YOLOv8)

rover2_dashboard/        # Minimal web interface
└── pack_robot_dashboard.py # Web teleop + person follower controls

rover2_description/      # Basic robot URDF (minimal)
```

**Launch Hierarchy:**
- `./launch_pack_robot.sh` → `pack_robot.launch.py` → starts ONLY essential nodes
- NO full rover launch, NO GPS launch, NO navigation launch

---

## 🖥️ **User Interface Strategy**

**Brain Selector Integration:**
1. **Boot:** Pi shows brain selector on 7" touchscreen (1024x600)
2. **Selection:** User taps "Pack Robot" → launches `./launch_pack_robot.sh`
3. **Operation:** Pack robot runs headless, 7" screen can go blank or show simple status
4. **Control:** User accesses web dashboard at `http://rover-ip:8080` from phone/laptop

**Web Dashboard Features (Minimal):**
- **WASD keyboard + touch controls** (mandatory for safety)
- **Emergency stop** (always visible)
- **Person follower enable/disable** (detection vs following separation)
- **Battery status** (safety monitoring)
- **Camera feed** (for monitoring person detection)

**NO complex dashboards, NO Foxglove integration, NO advanced telemetry**

---

## 🤖 **Person Following Behavior**

**Vision System:**
- **Hardware:** Hailo-8L AI accelerator + YOLOv8 model
- **Detection:** COCO class 0 (person) detection
- **Tracking:** Foot position (bottom of bounding box) for distance estimation
- **Control:** Generate cmd_vel to follow detected person

**Two-State Design:**
- **Detection Mode:** Visual tracking only (publishes annotated images)
- **Following Mode:** Active movement to follow person

**Safety Features:**
- Automatic disable on teleop override
- Timeout stop if person lost
- Recovery scanning behavior
- Distance-based speed control

---

## 🔄 **Development Workflow**

**For Code Agents:**

1. **ALWAYS start by reading this document**
2. **Verify scope:** Does the requested feature align with pack robot minimalism?
3. **Use existing packages:** rover2_* packages are the correct implementation
4. **Avoid rover1_* dependencies:** Pack robot should not depend on rover1 components
5. **Test minimal launch:** Changes should work with `pack_robot.launch.py`

**Common Mistakes to Avoid:**
- ❌ Adding GPS/SLAM features ("but it would be useful...")
- ❌ Creating complex dashboards ("users might want...")  
- ❌ Integrating rover1_* packages ("for code reuse...")
- ❌ Adding navigation capabilities ("what if they want...")

**The pack robot serves ONE purpose: following humans. Keep it minimal.**

---

## 📊 **Success Metrics**

**Pack Robot is successful when:**
- ✅ Launches with `./launch_pack_robot.sh` in <30 seconds
- ✅ Web teleop works reliably (WASD + touch)
- ✅ Person detection visually tracks humans accurately  
- ✅ Person following moves robot smoothly toward detected person
- ✅ Emergency stop instantly halts all movement
- ✅ Battery warnings prevent unexpected shutdown
- ✅ System runs for hours without crashes

**Pack Robot is NOT about:**
- Complex mapping accuracy
- GPS waypoint precision  
- Advanced path planning
- Multi-robot coordination
- Extensive sensor fusion

---

## 🔧 **Technical Constraints**

**Hardware Platform:**
- Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy
- Nuwa-HP60C camera, Hailo-8L AI accelerator
- Mecanum wheels with Hiwonder motor controller
- 7" touchscreen (brain selector only)

**Performance Requirements:**
- Person detection: 10+ FPS real-time inference
- Web dashboard: <200ms teleop latency
- Battery monitoring: 1Hz status updates
- Memory usage: <2GB total system

**Network Requirements:**
- Web dashboard on port 8080
- WebSocket on port 8765  
- Standard ROS 2 DDS topics

---

## 🚀 **Quick Reference**

**Key Commands:**
```bash
# Launch pack robot
./launch_pack_robot.sh

# Access web interface
http://rover-ip:8080

# Key launch file
rover2_bringup/launch/pack_robot.launch.py
```

**Key Topics:**
```bash
/cmd_vel                    # Motion control
/person_follower/status     # Following state
/battery/voltage           # Power monitoring  
/ascamera_hp60c/camera_publisher/rgb0/image # Camera feed
```

**Key Services:**
```bash
/person_follower/enable_detection   # Start visual tracking
/person_follower/enable_following   # Start movement
```

---

## 🎯 **Remember: MINIMAL IS THE GOAL**

Every feature request should be evaluated against: "Does this help the robot follow humans better?" If not, it doesn't belong in the pack robot.

**Rover1 = Full autonomous rover (SLAM + GPS + patrol)**
**Rover2 = Pack robot (human following only)**

Keep them separate. Keep rover2 minimal. Keep the focus clear.

---

*This document should be referenced by any code agent before making changes to rover2. The pack robot's success depends on maintaining this focused, minimal scope.*