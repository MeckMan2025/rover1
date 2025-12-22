# Rover1 Engineering Journal & Technical Specifications

**Maintainer:** MeckMan2025
**Last Updated:** 2025-12-21
**Purpose:** Living documentation of the hardware verification, driver protocols, and system architecture for the Rover1 autonomous vehicle. This document serves as the sole source of truth for AI agents and engineering rebuilds.

---

## 1. Hardware Architecture

### 1.1 Compute Unit
- **Device:** Raspberry Pi 5
- **OS:** Ubuntu 24.04 (Noble Numbat)
- **Host Name:** `rover1.local`
- **User:** `andrewmeckley`
- **Work Directory:** `~/ros2_ws/src/rover1`
- **ROS Distribution:** ROS 2 Jazzy Jalisco

### 1.2 Motor Control System
- **Controller:** Hiwonder Motor Driver HAT (Connected via I2C)
- **Communication Protocol:** I2C Bus 1, Address `0x34`
- **Driver Type:** Custom ROS 2 Node (`rover1_hardware/hiwonder_driver.py`) using direct register addressing.

#### Register Protocol (Confirmed Dec 21, 2025)
Standard Hiwonder protocols (broadcast speed) failed. Direct register addressing is required for independent wheel control.

| Wheel Position | I2C Register (Dec) | I2C Register (Hex) | Driver Index | Direction Invert? |
| :--- | :--- | :--- | :--- | :--- |
| **Rear Left** | **51** | `0x33` | 2 | NO |
| **Front Left** | **52** | `0x34` | 0 | **YES** |
| **Rear Right** | **53** | `0x35` | 3 | **YES** |
| **Front Right** | **54** | `0x36` | 1 | NO |

- **Safety Mechanism:** Driver implements a **0.5s Watchdog**.
- **Physical Motors:** 12V Encoder DC Motors.
- **Encoder Mapping (Confirmed Dec 21, 2025):**
  - **Register:** `0x3C` (4x 32-bit Signed Ints).
  - **Front Left**: Index 1 (Motor 2) | Fwd = Decrease
  - **Front Right**: Index 3 (Motor 4) | Fwd = Increase
  - **Rear Left**: Index 0 (Motor 1) | Fwd = Increase
  - **Rear Right**: Index 2 (Motor 3) | Fwd = Decrease

### 1.3 Navigation & Orientation
- **IMU:** OzzMaker BerryIMU v3 (Stackable header on GPIO)
    - **Magnetometer:** LIS3MDL (I2C `0x1c`)
    - **Accel/Gyro:** LSM6DSL (I2C `0x6a`)
    - **Barometer:** BMP388 (I2C `0x77`)
    - **Driver:** `rover1_hardware/berry_imu_driver.py`
- **GNSS (GPS):** u-blox ZED-F9R (High Precision RTK)
    - **Connection:** USB (Port 1)
    - **Device ID:** `idVendor=1546`, `idProduct=01a9`
    - **Correction Source:** NTRIP Client (Iowa RTN)

### 1.4 Power System
- **Power Source:** 3S or 4S LiPo (~12-16V).
- **Voltage Monitoring:**
  - **Register:** `0x00` (2-byte unsigned int).
  - **Unit:** Millivolts (mV).
  - **Note:** Register values confirm ~14.5V readings for 4S battery.
- **Regulation:** Hiwonder HAT provides 5V to Raspberry Pi via Pins.

---

## 2. Software Architecture

### 2.1 Package Structure
`rover1/`
├── `rover1_bringup/`       # Launch files and high-level configurations
│   ├── `launch/rover.launch.py`   # Master launch (Hardware + Kinematics)
│   └── `launch/gps.launch.py`     # GPS & NTRIP specific stack
├── `rover1_control/`       # (Planned) PID and Feedback controllers
├── `rover1_hardware/`      # Physical Device Drivers
│   ├── `hiwonder_driver.py`       # I2C Motor Controller
│   ├── `berry_imu_driver.py`      # IMU Data Publisher
│   └── `mecanum_kinematics.py`    # Twist -> Wheel Speeds conversion
├── `scripts/`              # System Utilities
│   ├── `auto_update.sh`           # Git Pull automation
│   └── `rover1-updater.service`   # Systemd service definition

### 2.2 Critical Configurations

#### Deployment (Auto-Update)
- Systemd timer checks GitHub every 5 minutes.
- Service: `rover1-updater.service`
- Target: `https://github.com/MeckMan2025/rover1`

#### GPS / NTRIP
- **Network:** Iowa DOT RTN (IaRTN)
- **Caster:** `165.206.203.10`
- **Port:** `10000`
- **Mountpoint:** `RTCM3_IMAX`
- **Auth:** Basic Auth (User/Pass stored in `gps.launch.py`)
- **Note:** `authenticate: True` parameter is **REQUIRED** for `ntrip_ros.py` to transmit credentials.

### 2.3 Known Issues & Fixes
- **GPS USB Busy:** `LIBUSB_ERROR_BUSY`.
    - *Cause:* Linux kernel driver `cdc_acm` grabs the USB device before ROS `ublox_dgnss` can claim it via `libusb`.
    - *Fix:* [SOLVED] Created `/etc/udev/rules.d/99-ublox-gnss.rules` to detach `cdc_acm`.
- **Motor Runaway:**
    - *Cause:* I2C latching on Hiwonder board.
    - *Fix:* Implemented software watchdog in `hiwonder_driver.py`.
- **NTRIP / RTK Topic Mismatch (Ghost Network Issue):**
    - *Symptoms:* NTRIP client connects successfully (log shows connection), but GPS remains in Float/Single mode.
    - *False Lead:* Suspected Ubuntu 24.04 firewall/network stack dropping packets (Ping failed due to wrong IP, actual internet was fine).
    - *Root Cause:* Topic namespace mismatch. `ntrip_client` publishes to `/rtcm`. Standard U-Blox launch file (`ublox_rover_hpposllh_navsatfix.launch.py`) expects `/ntrip_client/rtcm`.
    - *Fix:* Added `GroupAction` and `SetRemap` in `gps.launch.py` to bridge `/ntrip_client/rtcm` -> `/rtcm`.
    - *Lesson:* Always verify `ros2 topic info` subscribers before debugging network layers.

---

## 3. Operations Manual

### 3.1 Startup Procedure
1.  **Power On:** Toggle main switch. Pi boots automatically.
2.  **Verify Services:**
    ```bash
    systemctl status rover1-updater.service
    ```
3.  **Launch Stack:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch rover1_bringup rover.launch.py
    ```

### 3.2 Troubleshooting Commands
- **Check I2C Devices:** `i2cdetect -y 1`
- **Manually Spin M1 (Rear Left):**
    ```bash
    python3 probe_motors.py 51 50
    ```
- **View GPS Data:** `ros2 topic echo /ublox/fix`

---

## 4. Development Log (Key Milestones)
- **Phase 1 (Dec 2025):** Core Hardware Verification.
    - Established SSH and Auto-Update workflow.
    - Reverse-engineered Hiwonder Motor Protocol (Registers 51-54).
    - Verified IMU connectivity.
    - Configured NTRIP client for RTK corrections.
- **Phase 2 (Dec 21, 2025):** Sensor Fusion Setup.
    - Resolved GPS USB conflict (`LIBUSB_ERROR_BUSY`).
    - Established static TF tree (`base_link` -> `imu_link`/`gps_link`).
    - **Bench Test (Network):** Confirmed connectivity (Ping 8.8.8.8 OK, Port 10000 OK). "Ubuntu Connectivity Regression" was a false alarm.
    - **Bench Test (GPS):** Identified topic mismatch (`/rtcm` vs `/ntrip_client/rtcm`).
    - **Action:** Remapped NTRIP client in `gps.launch.py` to feed U-Blox driver.

