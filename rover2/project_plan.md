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

---

## Phase 6: Indoor Autonomy Demo (RTAB-Map + Nav2)
*Status: Planning*

Winter indoor demonstration system enabling visual SLAM mapping and autonomous navigation in GPS-denied environments. Target venues: home (kitchen/living room loop) and high school engineering classroom (desk navigation).

### 6.1 Product Vision

**User Experience Flow:**
1. Power on rover → All systems auto-start
2. Open `http://rover1.local` on any device (phone/tablet/laptop)
3. Select demo preset from dropdown (e.g., "Kitchen Loop", "Classroom Demo")
4. Drive rover with Stadia controller while watching live map build on dashboard
5. When map quality indicator shows "Ready", press **A button** to start autonomous loop
6. Rover follows recorded breadcrumb path indefinitely, rerouting around obstacles
7. Press **B button** for immediate E-Stop (full motor cut)

**Target Environments:**
| Demo | Location | Path Description |
|------|----------|------------------|
| Home Demo | Kitchen ↔ Living Room | Loop through walkways between rooms |
| School Demo | Engineering Classroom | Weave between desk rows |

### 6.2 Technical Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        WEB DASHBOARD                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │ GNSS Health │  │ Computer    │  │ Rover       │  ← Hamburger    │
│  │             │  │ Vision      │  │ Systems     │    Menu         │
│  └─────────────┘  └─────────────┘  └─────────────┘                 │
│        │                │                │                          │
│        ▼                ▼                ▼                          │
│  [Sat Count]      [Live Map]       [Battery]                       │
│  [RTK Status]     [Camera Feed]    [Mode: TELEOP]                  │
│  [Accuracy]       [Save/Clear]     [System Health]                 │
│                   [Demo Select ▼]                                   │
└─────────────────────────────────────────────────────────────────────┘
                              │
                    WebSocket (rosbridge)
                              │
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS 2 STACK                                 │
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐          │
│  │   HP60C      │───▶│  RTAB-Map    │───▶│    Nav2      │          │
│  │  RGB-D Cam   │    │  (SLAM)      │    │ (Navigation) │          │
│  └──────────────┘    └──────────────┘    └──────────────┘          │
│         │                   │                   │                   │
│         ▼                   ▼                   ▼                   │
│    /camera/rgb         /map              /cmd_vel                   │
│    /camera/depth       /odom             /path                      │
│                        /tf                                          │
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐          │
│  │  Breadcrumb  │───▶│   Mission    │───▶│   Motor      │          │
│  │  Recorder    │    │  Controller  │    │   Driver     │          │
│  └──────────────┘    └──────────────┘    └──────────────┘          │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.3 Controller Mapping (Stadia)

| Input | Function | Mode |
|-------|----------|------|
| Left Stick Y | Forward/Backward | TELEOP |
| Left Stick X | Turn Left/Right | TELEOP |
| Right Stick X | Strafe Left/Right | TELEOP |
| L2 Trigger | Dead Man's Switch | TELEOP |
| **A Button (Green)** | Start Autonomous Loop | MAPPING → AUTONOMOUS |
| **B Button (Red)** | E-Stop (Full Motor Cut) | ALL MODES |

### 6.4 Web Dashboard Redesign

#### 6.4.1 Responsive Design Requirements
- **Mobile (< 768px)**: Single column, touch-optimized controls
- **Tablet (768px - 1024px)**: Two-column layout, larger touch targets
- **Desktop (> 1024px)**: Full multi-panel layout with side-by-side views

#### 6.4.2 Navigation Menu (Hamburger)
```
☰ Menu
├── GNSS Health
│   ├── Satellite Count
│   ├── RTK Status
│   └── Position Accuracy
├── Computer Vision
│   ├── Live Camera Feed
│   ├── Map Visualization
│   ├── Save Map
│   ├── Clear Map / Reset
│   └── Demo Presets [Dropdown]
└── Rover Systems
    ├── Battery Level
    ├── Mode Indicator
    ├── Motor Status
    └── System Health
```

#### 6.4.3 Mode Indicator (Always Visible)
| Mode | Color | Description |
|------|-------|-------------|
| `TELEOP` | Blue | Manual driving, no mapping |
| `MAPPING` | Yellow | Building map while driving |
| `AUTONOMOUS` | Green | Following breadcrumb path |
| `E-STOP` | Red | Motors cut, awaiting reset |

#### 6.4.4 Map Quality Indicator
Visual feedback showing when map is ready for autonomous navigation:
- **Red**: Insufficient coverage / poor loop closure
- **Yellow**: Partial coverage, continue mapping
- **Green + Checkmark**: Good loop closure, ready for autonomous

### 6.5 Map Management

#### 6.5.1 Map Persistence
Maps saved to: `/home/rover/maps/`
```
/home/rover/maps/
├── kitchen_loop.db          # RTAB-Map database
├── kitchen_loop.yaml        # Map metadata
├── kitchen_loop_path.yaml   # Breadcrumb waypoints
├── classroom_demo.db
├── classroom_demo.yaml
└── classroom_demo_path.yaml
```

#### 6.5.2 Demo Preset Selection (Web Dashboard)
Dropdown menu under Computer Vision showing:
- Kitchen Loop
- Living Room Circuit
- Classroom Demo
- [+ Create New]

Selecting a preset:
1. Loads saved RTAB-Map database
2. Loads associated breadcrumb path
3. RTAB-Map enters localization mode (not mapping)
4. Path preview displayed on map
5. Ready for autonomous loop on A button press

### 6.6 Breadcrumb Path Recording

#### 6.6.1 Recording Workflow
1. User selects "Create New" demo or enters MAPPING mode
2. User drives rover through desired path with Stadia controller
3. System records pose at 1 Hz (position + orientation from RTAB-Map odometry)
4. Breadcrumb points displayed as trail on live map
5. When user completes loop (returns near start), system detects loop closure
6. User saves map + path via dashboard

#### 6.6.2 Path Data Structure
```yaml
# kitchen_loop_path.yaml
metadata:
  name: "Kitchen Loop"
  created: "2025-01-15T14:30:00"
  total_distance_m: 12.5
  estimated_duration_s: 45
waypoints:
  - {x: 0.0, y: 0.0, theta: 0.0}
  - {x: 0.5, y: 0.1, theta: 0.05}
  - {x: 1.0, y: 0.2, theta: 0.1}
  # ... continuous breadcrumb trail
loop_closure: true
```

### 6.7 Autonomous Navigation

#### 6.7.1 Path Following
- Nav2 waypoint follower executes breadcrumb path sequentially
- Regulated Pure Pursuit controller for smooth trajectory tracking
- Upon reaching final waypoint, automatically restart from beginning (infinite loop)

#### 6.7.2 Obstacle Handling
| Scenario | Behavior |
|----------|----------|
| Static obstacle detected | Nav2 local planner reroutes around obstacle |
| Path blocked completely | Stop, wait 5s, attempt reroute, alert user if still blocked |
| Dynamic obstacle (person) | Slow down, reroute, resume path when clear |

#### 6.7.3 Safety Systems
- **Battery Warning**: Alert at 20%, auto-stop at 15%
- **E-Stop**: B button = immediate motor cut, requires manual reset
- **Watchdog**: If no heartbeat from Nav2 for 2s, stop motors
- **Obstacle Timeout**: If stuck for 30s, stop and alert user

### 6.8 Implementation Phases

#### Phase 6.1: Dashboard Redesign Foundation
*Priority: High | Estimated Complexity: Medium*

**Tasks:**
- [ ] 6.1.1 Implement responsive CSS framework (mobile-first)
- [ ] 6.1.2 Create hamburger menu navigation component
- [ ] 6.1.3 Build GNSS Health panel (migrate existing)
- [ ] 6.1.4 Build Rover Systems panel (battery, mode indicator)
- [ ] 6.1.5 Implement mode indicator component (TELEOP/MAPPING/AUTONOMOUS/E-STOP)
- [ ] 6.1.6 Test across phone, tablet, laptop screen sizes

**Acceptance Criteria:**
- Dashboard loads and is usable on iPhone, iPad, MacBook
- Hamburger menu opens/closes smoothly
- Mode indicator visible on all screen sizes

#### Phase 6.2: RTAB-Map Integration
*Priority: High | Estimated Complexity: High*

**Tasks:**
- [ ] 6.2.1 Install RTAB-Map ROS 2 package on Pi 5
- [ ] 6.2.2 Configure HP60C camera topics for RTAB-Map input
- [ ] 6.2.3 Create `rtabmap.launch.py` with proper parameters
- [ ] 6.2.4 Verify occupancy grid output on `/map` topic
- [ ] 6.2.5 Test mapping in controlled indoor environment
- [ ] 6.2.6 Implement map save/load functionality
- [ ] 6.2.7 Add COLCON_IGNORE to ros2_orbslam (not needed)

**Acceptance Criteria:**
- RTAB-Map produces valid occupancy grid from HP60C
- Maps can be saved to disk and reloaded
- Localization works when map is loaded

#### Phase 6.3: Computer Vision Dashboard Panel
*Priority: High | Estimated Complexity: Medium*

**Tasks:**
- [ ] 6.3.1 Build Computer Vision menu section
- [ ] 6.3.2 Integrate live camera feed (existing)
- [ ] 6.3.3 Add real-time map visualization (occupancy grid render)
- [ ] 6.3.4 Implement Save Map button + naming dialog
- [ ] 6.3.5 Implement Clear Map / Reset button
- [ ] 6.3.6 Build demo preset dropdown with saved maps
- [ ] 6.3.7 Add map quality indicator (loop closure detection)
- [ ] 6.3.8 Implement path preview overlay on map

**Acceptance Criteria:**
- User can see camera feed and map building simultaneously
- Save/Clear map functions work from dashboard
- Demo presets populate from saved maps directory

#### Phase 6.4: Breadcrumb Path System
*Priority: High | Estimated Complexity: Medium*

**Tasks:**
- [ ] 6.4.1 Create `breadcrumb_recorder` node
- [ ] 6.4.2 Subscribe to RTAB-Map odometry for pose tracking
- [ ] 6.4.3 Implement 1 Hz waypoint recording during MAPPING mode
- [ ] 6.4.4 Add loop closure detection (start/end proximity)
- [ ] 6.4.5 Implement path serialization to YAML
- [ ] 6.4.6 Visualize breadcrumb trail on dashboard map
- [ ] 6.4.7 Associate paths with saved maps

**Acceptance Criteria:**
- Driving path recorded as breadcrumb waypoints
- Path saved alongside map database
- Path visible as trail on map visualization

#### Phase 6.5: Nav2 Indoor Configuration
*Priority: High | Estimated Complexity: High*

**Tasks:**
- [ ] 6.5.1 Configure Nav2 for indoor operation (no GPS)
- [ ] 6.5.2 Set up costmap with RTAB-Map as source
- [ ] 6.5.3 Configure Regulated Pure Pursuit for indoor speeds
- [ ] 6.5.4 Implement waypoint follower for breadcrumb execution
- [ ] 6.5.5 Add infinite loop logic (restart path on completion)
- [ ] 6.5.6 Configure obstacle avoidance (reroute behavior)
- [ ] 6.5.7 Tune local planner for tight indoor spaces

**Acceptance Criteria:**
- Nav2 follows breadcrumb path accurately
- Rover reroutes around obstacles
- Path loops continuously until stopped

#### Phase 6.6: Mission Controller Enhancement
*Priority: High | Estimated Complexity: Medium*

**Tasks:**
- [ ] 6.6.1 Add MAPPING state to mission controller
- [ ] 6.6.2 Implement A button → Start autonomous loop
- [ ] 6.6.3 Implement B button → E-Stop (full motor cut)
- [ ] 6.6.4 Add state transitions: TELEOP ↔ MAPPING → AUTONOMOUS
- [ ] 6.6.5 Implement battery level check before autonomous start
- [ ] 6.6.6 Add watchdog timer for Nav2 heartbeat
- [ ] 6.6.7 Broadcast mode changes to dashboard via WebSocket

**Acceptance Criteria:**
- Controller buttons trigger correct state changes
- E-Stop immediately cuts motors
- Mode indicator on dashboard reflects current state

#### Phase 6.7: Safety & Polish
*Priority: Medium | Estimated Complexity: Low*

**Tasks:**
- [ ] 6.7.1 Implement battery warning overlay (< 20%)
- [ ] 6.7.2 Add auto-stop on low battery (< 15%)
- [ ] 6.7.3 Implement stuck detection (30s timeout)
- [ ] 6.7.4 Add user alerts for error conditions
- [ ] 6.7.5 Test full workflow end-to-end
- [ ] 6.7.6 Document demo setup procedure

**Acceptance Criteria:**
- Safety systems prevent unsafe autonomous operation
- Full demo workflow works reliably
- Documentation sufficient for classroom demo

### 6.9 Technical Dependencies

| Component | Package | Status |
|-----------|---------|--------|
| Visual SLAM | `ros-jazzy-rtabmap-ros` | To Install |
| Navigation | `ros-jazzy-navigation2` | To Install |
| Depth Camera | `ascamera` (HP60C) | Installed |
| Web Framework | Flask + WebSocket | Installed |
| Controller | `ros-jazzy-joy` | Installed |

### 6.10 Success Metrics

| Metric | Target |
|--------|--------|
| Map build time | < 5 minutes for typical room |
| Localization accuracy | < 10cm position error |
| Path following accuracy | < 20cm deviation from recorded path |
| Obstacle reroute time | < 3 seconds |
| Dashboard latency | < 200ms for all visualizations |
| Demo reliability | 95% success rate for complete loop |

### 6.11 Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Pi 5 CPU overload | Profile and optimize; consider offloading map visualization |
| RTAB-Map drift | Ensure good loop closure; add visual landmarks if needed |
| Narrow passages | Tune costmap inflation; reduce robot footprint margin |
| Lighting variation | Test in different lighting; consider IR projection |
| Network latency | Optimize WebSocket payloads; compress map data |

---

