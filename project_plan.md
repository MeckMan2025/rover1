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

---

## Immediate Next Steps (Updated 2025-12-26)
1.  ~~**Rebuild & Deploy**: Run `colcon build` on Pi to apply NMEA bridge fix, restart rover service.~~ ✅ **COMPLETE**
2.  ~~**Professional GPS Dashboard**: Implement GNSS health monitor for Foxglove integration.~~ ✅ **COMPLETE** 
3.  **Deploy GNSS Health Monitor**: Build and integrate new package on Pi, test Foxglove dashboard.
4.  **EKF Square Drive Validation**: Perform the 5m square test and analyze the digital trail in Foxglove.
5.  **Path Planning (Nav2)**: Configure Navigation2 stack for autonomous waypoint following.
6.  **Mission Controller**: Implement the first "Patrol" state machine.


