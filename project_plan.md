# Rover1 Project Plan

**Goal**: Deliver a "Zero-Experience to Autonomy" kit with a professional-grade User Interface and reliable "Teach & Repeat" navigation.

**Timeline Strategy**:
- **Last Session**: 2025-12-24 - GPS/RTK Silent Fix Debugging & NTRIP Bridge Repair
- **Next**: Rebuild on Pi, verify RTK corrections flowing, then EKF Square Drive Validation.
- **Final**: Nav2 Autonomy + Web UI & "Patrol Mode" Logic.

---

## Phase 1: Core Platform (Hardware & Drivers)
*Status: 100% Complete*

The foundation of the robot. If this fails, nothing else works.

- [x] **ROS 2 Setup**: Ubuntu 24.04 + Jazzy.
- [x] **Motor Driver**: Custom `hiwonder_driver` (I2C 0x34).
- [x] **Kinematics**: Mecanum mixing enabled.
- [x] **Sensors**:
  - `berry_imu_driver` (Orientation).
  - `ublox_dgnss` + `ntrip_client` (RTK GPS).
- [x] **Verification**: User confirms wheels spin and GPS gets Fix.
- [x] **Verification**: User confirms wheels spin and GPS gets Fix.

## Phase 1.5: Manual Control Expansion (Bluetooth Controller)
*Status: Complete*

Adding professional-grade physical control for precise field maneuvers and demos.

### 1.5.1 Driver & Pairing
- [x] **Bluetooth Setup**: Pair Google Stadia controller via `bluetoothctl` (MAC: D1:71:42:54:CB:0F).
- [x] **Controller Mode**: Stadia controller in Bluetooth mode (unlocked after Stadia shutdown).
- [x] **ROS Integration**: Using `ros-jazzy-joy` with custom `stadia_teleop.py` node.

### 1.5.2 Mapping Configuration
Custom `stadia_teleop.py` node with Stadia-specific layout:
- **Left Stick Y (Axis 1)**: Forward/Backward (linear.x).
- **Left Stick X (Axis 0)**: Turn Left/Right (angular.z).
- **Right Stick X (Axis 2)**: Strafe Left/Right (linear.y).
- **L2 Trigger (Axis 5)**: Dead Man's Switch (must hold to enable movement).

### 1.5.3 Verification
- [x] **Calibration**: joy_node configured with 0.1 deadzone to prevent drift.
- [x] **Field Test**: Verified working Dec 23, 2025 - full Mecanum control operational.

## Phase 1.6: Field Networking & Failover
*Status: Complete*

Reliable connectivity across all environments for field demos and development.

### 1.6.1 Network Priority Chain
- [x] **Home WiFi**: "Lake Wifi" (priority 100) - auto-connects when in range.
- [x] **Phone Hotspot**: "AJM17ProMax" (priority 50) - fallback for field demos.
- [x] **Ethernet Tether**: Static IP 10.42.0.1 - emergency/direct laptop connection.

### 1.6.2 Implementation
- [x] **NetworkManager**: Installed and configured with `autoconnect-priority`.
- [x] **Setup Script**: `scripts/setup_network_failover.sh` (run once on Pi).
- [x] **Credentials**: Stored in `.env` file (gitignored).

### 1.6.3 Mac Development Setup
- [x] **Smart SSH**: `~/.zshrc` function auto-selects ethernet vs WiFi.
- [x] **Ethernet Config**: Manual IP 10.42.0.2 for direct tether.

## Phase 2: Sensor Fusion & Localization
*Status: 100% Complete*

Establishing the critical `map` -> `odom` -> `base_link` transform tree.

### 2.1 Transform Hierarchy (TF Tree)
*Status: Complete*
- **Structure**: `map` (GPS) -> `odom` (EKF) -> `base_link` (Robot) -> `imu_link`/`gps_link`.
- **Implementation**: Managed by `rover1_description/urdf/rover.urdf.xacro` and `robot_state_publisher`.
- [x] Create URDF model.
- [x] Replace static transforms in launch file.
- [x] **Calibration**: Physical `ticks_per_rev` calibrated (3171.44).

### 2.2 IMU Implementation
*Status: Complete*
- **Driver Update**: Enhance `berry_imu_driver.py` with:
    - [x] Temperature compensation.
    - [x] Bias calibration (Gyro/Accel).
    - [x] ROS coordinate frame conversion (NED -> ENU).
    - [x] Proper covariance matrix population.
    - [x] **Orientation Publishing**: Implement Madgwick or Complementary filter to publish Quaternions (Required for EKF).

### 2.3 Robot Localization (EKF)
*Status: 100% Complete*
- **Configuration**: Local and Global `robot_localization` nodes operational, linked to `ekf.yaml`/`ekf_global.yaml`.
- **Inputs**: Wheel Odometry (Twist) + IMU (Orientation/Angular Velocity) + GPS.

### 2.4 GPS Integration
*Status: 100% Complete (Debugged Dec 24, 2025)*
- **Goal**: `navsat_transform_node` to anchor `odom` to `map`.
- **Achievements**:
    - [x] RTK Float/Fixed state confirmed.
    - [x] `navsat_transform` linking GPS to TF tree.
    - [x] Heading correction from IMU verified.
    - [x] **QoS Fix**: `/fix` topic requires BEST_EFFORT QoS to read.
    - [x] **NMEA Bridge Fix**: `fix_to_nmea` now publishes `nmea_msgs/msg/Sentence` for NTRIP VRS handshake.

### 2.5 Battery Monitoring Integration
*Status: 100% Complete*
- [x] **Isolated Node**: Implemented `battery_monitor.py` for decoupled I2C telemetry.
- [x] **Verification**: Confirmed stable 14.5V reading on `/battery_voltage`.

## Phase 2.6: Engineering UI, Visualization & Tuning (Foxglove)
*Status: 100% Complete*

This phase establishes a professional-grade engineering interface for real-time inspection, tuning, and validation of Rover1's internal state during indoor and outdoor testing.

This UI is not customer-facing. It exists to ensure sensor fusion, localization, and navigation behaviors are correct, observable, and debuggable before autonomy is enabled.

## Phase 2.7: GNSS Health Monitor Integration
*Status: 100% Complete (Dec 26, 2025)*

Professional GPS/RTK dashboard integration for Foxglove replacing terminal-based monitoring.

### 2.7.1 GNSS Health Aggregation Package
- [x] **Custom Message Type**: `GnssHealth.msg` with satellite counts, RTCM rates, RTK state, and accuracy
- [x] **Aggregation Node**: `gnss_health_monitor_node.py` with robust topic fallback logic
- [x] **Multi-topic Subscription**: `/gps/filtered` → `/fix`, `/ntrip_client/rtcm` → `/rtcm`
- [x] **QoS Handling**: Proper BEST_EFFORT QoS for sensor topics
- [x] **Message Type Detection**: Automatic `rtcm_msgs/Message` → `std_msgs/ByteMultiArray` fallback

### 2.7.2 Real-time Statistics Engine  
- [x] **Rolling Windows**: 5-second RTCM rate calculation with deque-based implementation
- [x] **Correction Age Tracking**: Real-time seconds since last RTCM packet
- [x] **Accuracy Interpretation**: Proper NavSat covariance matrix → horizontal/vertical accuracy
- [x] **RTK State Heuristics**: Threshold-based NO_FIX/DGPS/FLOAT/FIXED determination

### 2.7.3 Foxglove Dashboard Optimization
- [x] **Single Topic Design**: `/gnss/health` replaces 5+ separate topic monitoring
- [x] **Clean Data Types**: Numeric fields for plots, string fields for state displays
- [x] **Historical Analysis**: Time-stamped data for RTK acquisition trends
- [x] **Professional UI**: Eliminates flashing terminal monitoring requirements

## Phase 2.8: GNSS Web Dashboard & Controller Auto-Connect
*Status: 100% Complete (Dec 26, 2025)*

Real-time web-based GNSS monitoring and automated Bluetooth controller connection.

### 2.8.1 GNSS Web Dashboard
- [x] **Web Server**: Real-time GNSS health dashboard at `http://rover-ip:8080/`
- [x] **WebSocket Integration**: Live data streaming from `/gnss/health` topic
- [x] **Clean UI**: Professional web interface showing satellite count, RTK status, accuracy
- [x] **Multi-Message Support**: UBXNavPVT + UBXNavSat + NavSat integration
- [x] **QoS Compatibility**: Mixed QoS profiles for different publisher types

### 2.8.2 Stadia Controller Auto-Connect Service  
- [x] **Systemd Service**: `stadia-controller.service` auto-connects on boot
- [x] **Retry Logic**: 10 attempts with 3s delays for robust connection
- [x] **Security Hardening**: Proper systemd security settings
- [x] **Installation Scripts**: One-command setup with validation
- [x] **Logging**: Comprehensive logs to `/var/log/stadia-controller.log`

### 2.8.3 Critical Bug Resolution (Engineering Achievement)
Complex multi-layered debugging session resolving 7 interconnected issues:
- [x] **Message Type Correction**: Fixed `ublox_msgs` → `ublox_ubx_msgs` import
- [x] **Field Name Fixes**: Corrected `msg.svs` → `msg.sv_info`, `sat.flags.sv_used`
- [x] **Numpy Array Handling**: Fixed truth value ambiguity errors
- [x] **QoS Matrix Implementation**: Topic-specific QoS profiles for compatibility
- [x] **Timing Race Resolution**: 3.0s timeout for 1 Hz message safety margin
- [x] **Fallback Logic Repair**: Always-accept fallback prevents intermittent failures
- [x] **Active Data Sources**: UBXNavPVT subscription for real-time satellite counts

**Engineering Documentation**: Complete debugging methodology documented in `ENGINEERING_LOG_GNSS_DEBUGGING.md`

2.5.1 Foxglove Bridge Integration
	•	Goal: Enable live ROS 2 introspection from laptop/tablet via Foxglove Studio.
	•	Implementation:
	•	Launch Foxglove Bridge as part of rover.launch.py
	•	Bridge remains LAN-only during development
	•	Security:
	•	No public port exposure
	•	VPN (Tailscale) considered for future remote access

2.5.2 Core Telemetry & Topics (Engineering Contract)

The following topics must be available and stable for dashboards and replay:

Transforms
	•	/tf
	•	/tf_static

Localization & Motion
	•	/odom
	•	/imu/data
	•	/ublox/fix
	•	/cmd_vel (visibility only at this stage)

Health & Diagnostics
	•	/diagnostics
	•	/parameter_events
	•	Battery voltage topic (source: Hiwonder HAT)

2.5.3 Foxglove Dashboard: “Rover1 Field Test”

A reusable dashboard layout for outdoor testing and tuning.

Panels
	•	3D View: Validate TF tree (map → odom → base_link → imu_link → gps_link)
	•	Plots:
	•	Yaw & yaw rate
	•	Wheel encoder velocities
	•	GPS accuracy / RTK status
	•	Raw Messages:
	•	IMU
	•	GPS fix
	•	Diagnostics
	•	Map View (post-navsat integration)

2.5.4 Parameter Tuning & Replay Workflow
	•	Runtime Parameter Tuning:
	•	EKF covariances
	•	IMU filter parameters
	•	Navigation controller gains (future)
	•	Recording indicated test runs using rosbag2
	•	Offline Replay used for tuning without requiring field presence

2.5.5 Acceptance Criteria
	•	Engineer can visually confirm:
	•	Correct TF hierarchy
	•	Stable yaw estimation
	•	GPS-to-map alignment
	•	No unbounded drift
	•	All Phase 3 navigation work must be validated through Foxglove before autonomy is enabled.

## Phase 3: Autonomous Navigation Stack
*Status: Planned*

### 3.1 Nav2 Configuration
- **Controller**: **Regulated Pure Pursuit** (Not DWB). Optimized for outdoor waypoint following.
- **Behavior**: "Rotate to Heading" enabled for precise path alignment.
- **Costmaps**: 2D Rolling Window (Inflation Layer only initially).

### 3.2 Mission Logic ("The Brain")
- **Node**: `mission_controller.py` State Machine.
- **States**: `IDLE`, `MANUAL`, `RECORDING`, `EXECUTING`, `EMERGENCY`.
- **Logic**:
    - **Recording**: Capture RTK-Fixed GPS points.
    - **Patrol**: Execute Path -> 180° Turn -> Reverse Path -> Loop.

## Phase 4: User Interface & Integration
Phase 4 focuses on delivering a polished, customer-facing Web UI, built only after rover behaviors are stable, validated, and observable through Foxglove.

### 4.1 "Meckman V4" Web UI
- **Tech**: React + Vite + `rosbridge_server`.
- **Performance**: Optimized WebSocket handling (separate High/Low freq topics).
- **Design**:
    - High-Contrast "Dark Lab" Theme (Outdoor Visibility).
    - **Virtual Joystick**: Large touch targets, "deadzone" logic for drift prevention.
    - **Mission Panel**: Live status of GPS Quality (RTK/Float/Single) and Mission State.

## Phase 5: Computer Vision & Perception
*Status: Planned*

This phase introduces visual intelligence to complement RTK-GPS navigation, providing obstacle avoidance, precise local positioning, and enhanced situational awareness.

### 5.1 Perception Foundation (Drivers & Transforms)
- **Driver Integration**: Build and deploy `ascam_ros2_ws` on Pi 5.
- **TF Tree Update**: Update URDF with precise camera mounting:
    - **Height**: 33cm from ground to bottom surface of housing.
    - **Pitch**: -7° (downward tilt towards ground).
- **Optimization**: Implement `image_transport` compression for low-latency Foxglove monitoring.

### 5.2 Feature Extraction
- **Obstacle Veto**: Implement color segmentation or depth-based traversability analysis.
- **Visual Landmarks**: AprilTag or marker detection for docking and sub-decimeter refinement.

### 5.3 Hybrid Navigation Controller
- **Vision Veto Layer**: Integrate "Stop/Swerve on detection" logic into the mission controller.
- **Local Planner Blending**: Use CV data to adjust GPS-based paths in real-time.
- **State Machine Modes**: Add `VISION_NAV` mode for tight spaces.

### 5.4 Observability & Safety
- **Web Cockpit**: Integrate live MJPEG stream or WebSocket frames directly into the `gnss_web_dashboard` (Port 8080).
- **Visual Overlays**: Display RTK status and accuracy as an "HUD" (Heads-Up Display) on top of the live video feed.
- **Safety Shield**: Logic to perform an immediate "Clean Stop" if the camera node heartbeats stall.

### 5.5 Implementation & Testing Plan (Dashboard-First Visuals)

#### 5.5.1 Step-by-Step Implementation
1. **Unpack & Build**: Deploy `ascam_ros2_ws` to Pi 5 and compile with `colcon build`.
2. **Dashboard Upgrade**: Enhance `web_dashboard.py` with `image_raw` subscription and JPEG encoding.
3. **HTML HUD**: Update `index.html` with a video window and dynamic HUD overlays.
4. **TF Alignment**: Update URDF with `33cm` height and `-7°` pitch offsets.
5. **Logic**: Develop `obstacle_veto_node.py` (Simple ROI-based thresholding for traversability).

#### 5.5.2 Testing & Validation
- **P0: Stream Latency**: Verify <200ms delay between reality and Web Dashboard at 640x480.
- **P0: Controller Sync**: Verify zero "input lag" when driving via Stadia controller while watching the web stream.
- **P1: Concurrency Audit**: Ensure the Pi 5 can handle the Web Server, WebSocket, GPS stack, and Video encoder simultaneously.
- **P2: Safety Failover**: Verify Mission Controller triggers "Clean Stop" if the video stream disconnects for >1s.

---

## Immediate Next Steps (Updated 2025-12-27)

### ✅ **RECENTLY COMPLETED (This Session)**
1. ~~**GNSS Web Dashboard**: Real-time web interface with satellite count and RTK status~~
2. ~~**Stadia Controller Auto-Connect**: Systemd service for automatic Bluetooth pairing~~  
3. ~~**Critical Bug Resolution**: 7-layer debugging session fixing message types, QoS, timing, and logic~~
4. ~~**Engineering Documentation**: Complete debugging methodology and lessons learned~~

### 🎯 **NEXT PRIORITIES**

#### **Phase 3A: EKF Validation & Tuning** *(Immediate - Next Session)*
1. **EKF Square Drive Test**: Execute precise 5m × 5m square pattern with Stadia controller
2. **Foxglove Analysis**: Record and analyze odometry drift, GPS alignment, heading accuracy
3. **Parameter Tuning**: Adjust EKF covariances based on real-world performance data
4. **TF Tree Validation**: Verify stable `map` → `odom` → `base_link` transforms

#### **Phase 3B: Nav2 Stack Configuration** *(Week 1, Jan 2025)*
5. **Nav2 Setup**: Install and configure Navigation2 with Regulated Pure Pursuit controller
6. **Costmap Configuration**: Set up 2D rolling window costmaps for outdoor navigation
7. **Path Planning**: Configure global and local planners for waypoint following
8. **Safety Systems**: Emergency stop, obstacle avoidance, timeout handling

#### **Phase 3C: Mission Logic Implementation** *(Week 2-3, Jan 2025)*
9. **Mission Controller**: Implement state machine (`IDLE`, `MANUAL`, `RECORDING`, `EXECUTING`)
10. **Teach & Repeat**: GPS waypoint recording and playback functionality
11. **Patrol Mode**: Autonomous path execution with turnaround logic


