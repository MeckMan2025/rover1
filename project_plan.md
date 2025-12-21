# Rover1 Project Plan

**Goal**: Deliver a "Zero-Experience to Autonomy" kit with a professional-grade User Interface and reliable "Teach & Repeat" navigation.

**Timeline Strategy**:
- **Today**: Verify Hardware (Motor/GPS) & Basic Teleop.
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
- [ ] **Verification**: User confirms wheels spin and GPS gets Fix.

## Phase 2: Sensor Fusion & Localization
*Status: Planned (Next Priority)*

Establishing the critical `map` -> `odom` -> `base_link` transform tree.

### 2.1 Transform Hierarchy (TF Tree)
- **Goal**: Establish static transforms FIRST.
- **Structure**: `map` (GPS) -> `odom` (EKF) -> `base_link` (Robot) -> `imu_link`/`gps_link`.

### 2.2 IMU Implementation
- **Driver Update**: Enhance `berry_imu_driver.py` with:
    - Temperature compensation.
    - Bias calibration (Gyro/Accel).
    - ROS coordinate frame conversion (NED -> ENU).
    - Proper covariance matrix population.

### 2.3 Robot Localization (EKF)
- **Configuration**: `robot_localization` node.
- **Inputs**: Wheel Odometry (Twist) + IMU (Orientation/Angular Velocity).
- **Tuning**: Conservative covariance start.

### 2.4 GPS Integration
- **Goal**: `navsat_transform_node` or custom `GPSIntegration` node.
- **Logic**: Set Datum (Origin) on first RTK Fix -> Publish `map`->`odom` transform.
- **Fix**: Resolve `LIBUSB_ERROR_BUSY` (udev rules).

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
*Status: Planned*

### 4.1 "Meckman V4" Web UI
- **Tech**: React + Vite + `rosbridge_server`.
- **Performance**: Optimized WebSocket handling (separate High/Low freq topics).
- **Design**:
    - High-Contrast "Dark Lab" Theme (Outdoor Visibility).
    - **Virtual Joystick**: Large touch targets, "deadzone" logic for drift prevention.
    - **Mission Panel**: Live status of GPS Quality (RTK/Float/Single) and Mission State.

---

## Immediate Next Steps (Phase 2 Start)
1.  **System Fix**: Resolve GPS USB `LIBUSB_ERROR_BUSY`.
2.  **TF Tree**: Define static transforms in `rover.launch.py`.
3.  **IMU**: Implement robust driver with calibration.
4.  **EKF**: Fuse Odometry and IMU.
