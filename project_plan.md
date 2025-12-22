# Rover1 Project Plan

**Goal**: Deliver a "Zero-Experience to Autonomy" kit with a professional-grade User Interface and reliable "Teach & Repeat" navigation.

**Timeline Strategy**:
- **Today**: 2025-12-22 ~15:55 - Update Project Plan & Address Code Cautions
- **Next**: Autonomous Navigation Stack (EKF, Nav2).
- **Final**: Web UI & "Patrol Mode" Logic.

---

## Phase 1: Core Platform (Hardware & Drivers)
*Status: 90% Complete (Verifying on Device)*

The foundation of the robot. If this fails, nothing else works.

- [x] **ROS 2 Setup**: Ubuntu 24.04 + Jazzy.
- [x] **Motor Driver**: Custom `hiwonder_driver` (I2C 0x34).
- [x] **Kinematics**: Mecanum mixing enabled.
- [x] **Sensors**:
  - `berry_imu_driver` (Orientation).
  - `ublox_dgnss` + `ntrip_client` (RTK GPS).
- [x] **Verification**: User confirms wheels spin and GPS gets Fix.

## Phase 2: Sensor Fusion & Localization
*Status: In Progress*

Establishing the critical `map` -> `odom` -> `base_link` transform tree.

### 2.1 Transform Hierarchy (TF Tree)
*Status: In Progress - Fixing Static Transforms with URDF*
- **Structure**: `map` (GPS) -> `odom` (EKF) -> `base_link` (Robot) -> `imu_link`/`gps_link`.
- **Implementation**: ~Static transforms defined in `rover.launch.py`.~ **Needs URDF.**

### 2.2 IMU Implementation
*Status: Complete*
- **Driver Update**: Enhance `berry_imu_driver.py` with:
    - [x] Temperature compensation.
    - [x] Bias calibration (Gyro/Accel).
    - [x] ROS coordinate frame conversion (NED -> ENU).
    - [x] Proper covariance matrix population.
    - [x] **Orientation Publishing**: Implement Madgwick or Complementary filter to publish Quaternions (Required for EKF).

### 2.3 Robot Localization (EKF)
*Status: Configured (Requires 2.2)*
- **Configuration**: `robot_localization` nodes created and linked to `ekf.yaml`/`ekf_global.yaml`.
- **Inputs**: Wheel Odometry (Twist) + IMU (Orientation/Angular Velocity).

### 2.4 GPS Integration
*Status: Configured (Requires 2.2)*
- **Goal**: `navsat_transform_node` to anchor `odom` to `map`.
- **Status**:
    - [x] `LIBUSB_ERROR_BUSY` Resolved.
    - [x] Launch file configured.
    - [x] **Dependency**: Needs IMU Heading to function.

Phase 2.5: Engineering UI, Visualization & Tuning (Foxglove)

Status: Planned (Required before Phase 3)

This phase establishes a professional-grade engineering interface for real-time inspection, tuning, and validation of Rover1’s internal state during indoor and outdoor testing.

This UI is not customer-facing. It exists to ensure sensor fusion, localization, and navigation behaviors are correct, observable, and debuggable before autonomy is enabled.

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

## Immediate Next Steps (Phase 2 Continued)
1.  **URDF**: Implement `rover.urdf.xacro` to replace hacky static transforms.
2.  **Odometry Tuning**: Create helper script to calibrate `ticks_per_rev`.
3.  **EKF Validation**: Verify fusion in Foxglove.
