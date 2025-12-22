# Phase 2: Sensor Fusion & Localization To-Do List

**Goal:** Establish a robust coordinate system (`map` -> `odom` -> `base_link`) using EKF Sensor Fusion (Wheel Odometry + IMU) and RTK GPS integration.

---

## 2.1 System Setup & Cleanup (Day 1)
- [x] **Fix GPS USB Conflict (`LIBUSB_ERROR_BUSY`)**
    - [x] Create `/etc/udev/rules.d/99-ublox-gnss.rules` to detach `cdc_acm` driver from ZED-F9R.
    - [ ] Verify `ublox_dgnss_node` starts cleanly without error.
- [x] **TF Tree Foundation**
    - [x] Add `static_transform_publisher` to `rover.launch.py` for `base_link` -> `imu_link`.
    - [x] Add `static_transform_publisher` to `rover.launch.py` for `base_link` -> `gps_link`.
    - [ ] Verify tree with `ros2 run tf2_tools view_frames`.

## 2.2 IMU Driver Enhancement (Day 2)
The current driver is a stub. It requires physics implementation.
- [x] **Data Calibration**
    - [ ] Implement `apply_calibration()` method for Accel/Gyro bias subtraction.
    - [x] Create `imu_calibration_tool.py` to calculate static offsets.
- [x] **Coordinate Conversion**
    - [x] Ensure driver outputs **ENU** (East-North-Up) convention (standard ROS).
    - [x] Verify: +X is Forward, +Y is Left, +Z is Up.
    - [ ] Check: Turning Left = Positive Z angular velocity.
- [x] **Covariance Matrices**
    - [x] Populate `orientation_covariance` (if using Madgwick/Mahony filter inside driver) or leave strictly for angular velocity/accel if using EKF for orientation.
    - [x] Populate `angular_velocity_covariance` and `linear_acceleration_covariance`.

## 2.3 Sensor Fusion (EKF) Setup (Day 3)
- [x] **Install Packages**
    - [x] `sudo apt install ros-jazzy-robot-localization`
- [x] **Configure `ekf.yaml`**
    - [x] Create config file in `rover1_bringup/config/ekf.yaml`.
    - [x] **Odom0 (Wheel Twist)**: Enable X, Y (False), Yaw (False), VX, VY, VYaw.
    - [x] **Imu0**: Enable Roll, Pitch, Yaw (Orientation) + VRoll, VPitch, VYaw (Velocity).
    - [x] **Frame IDs**: `odom_frame: odom`, `base_link_frame: base_link`.
- [x] **Integration**
    - [x] Add `ekf_node` to `rover.launch.py`.
    - [x] Verify `odom` -> `base_link` transform is being published by EKF (not kinematics node anymore).

## 2.4 GPS Integration (Day 4)
- [ ] **Datum / Origin Logic**
    - [ ] Decide: use standard `navsat_transform_node` OR custom `GPSIntegration` node (as per Mentor advice).
    - [ ] *Recommendation:* Start with `navsat_transform_node` for standard compliance.
- [ ] **NTRIP Verification**
    - [ ] Confirm `401 Unauthorized` is gone (now that `authenticate: True` is set).
    - [ ] Verify `/ublox/fix` status is 4 (RTK Fixed) or similar.
- [x] **Map Transform**
    - [x] Configure `navsat_transform_node` to publish `map` -> `odom`.
    - [ ] Verify robot position in Rviz2 relative to map frame.

## 2.5 Field Validation (Day 5)
- [ ] **Static Test (10 mins)**
    - [ ] Place robot outside.
    - [ ] Verify position drift is minimal (< 10cm).
- [ ] **Driving Test**
    - [ ] Drive 10m Forward -> Turn 90° -> Drive 10m.
    - [ ] Compare `odom` path vs `gps` path.
    - [ ] Tune EKF covariance if `odom` diverges significantly from GPS reality.

---

## 2.6 Definition of Done (Phase 2)
1.  **Transform Tree**: `map` -> `odom` -> `base_link` exists and is connected.
2.  **Odometry**: Wheel encoders provide smooth local velocity.
3.  **IMU**: Provides stable orientation (no drift when stationary).
4.  **GPS**: RTK Fixed mode achieved; reliable global position available.
