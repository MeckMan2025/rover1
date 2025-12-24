# Rover1 Engineering Journal & Technical Specifications

**Maintainer:** MeckMan2025
**Last Updated:** 2025-12-24
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

- **Safety Mechanism:** Driver implements a **0.5s Watchdog** (Fine-tuned on Dec 22, 2025 to balance responsiveness vs OS key-repeat delay).
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
│   ├── `load_env.sh`              # Environment variable loader
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
- **Auth:** Basic Auth (Credentials stored in `.env` file)
- **Security:** Environment variables loaded via `scripts/load_env.sh`
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
3.  **Launch Stack (Automated):**
    ```bash
    # GPS only
    ./launch_gps.sh
    
    # Full rover stack
    ./launch_rover.sh
    ```
4.  **Launch Stack (Manual):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    source scripts/load_env.sh  # Load credentials
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

### 4.1 RTK/GPS Integration & Debugging (Dec 21, 2025)
**Problem:** NTRIP Client connected but provided no RTK corrections (Float/Fixed status never achieved).
**Discovery:**
1.  **"Ghost" Connection:** The NTRIP client was silent because it requires the Rover's position (NMEA GPGGA) to request VRS corrections from the Caster. The U-Blox driver does NOT publish this by default in a format the client consumes.
2.  **QoS Mismatch:** The U-Blox driver publishes `/fix` using `SensorData` (Best Effort) QoS. Standard subscribers use `Reliable`. This caused the bridge node to never receive messages.
3.  **Protocol Strictness:** The NTRIP Caster rejects NMEA sentences longer than 82 characters. This happens if the altitude has too many decimal places.

**Solution (The "Bridge" Pattern):**
1.  **Created `fix_to_nmea` Node:** A custom node in `rover1_hardware` that subscribes to `/fix` (with `SensorData` QoS) and publishes valid GPGGA sentences to `/nmea`.
2.  **Formatting:** Truncated altitude to 2 decimal places to ensure NMEA sentence length < 82 chars.
3.  **Persistence:** Created `rover1_bringup/config/ublox_gps.yaml` to enforce `CFG_USBINPROT_RTCM3X` and `CFG_USBOUTPROT_NMEA` on boot.
4.  **Launch:** Integrating `fix_to_nmea` into `gps.launch.py`.

### 4.2 IMU Driver Upgrade (Dec 21, 2025)
**Problem:** `robot_localization` EKF requires Orientation (Quaternion) and Covariance matrices, but raw IMU driver only provided Angular Velocity and Linear Acceleration.
**Solution:**
1.  **Calibration:** Implemented a 200-step startup routine to calculate and subtract static Gyroscope bias.
2.  **Sensor Fusion (Internal):** Implemented a Complementary Filter (Alpha 0.98) to fuse Accelerometer (Gravity Vector - stable long term) and Gyroscope (Rate - accurate short term) to estimate stable Pitch and Roll.
3.  **Frame Conversion:** Standardized output to ENU (East-North-Up) frame.
4.  **Result:** `berry_imu_driver.py` now publishes `geometry_msgs/Quaternion` on `/imu/data`, enabling the EKF to function.

### 4.3 Security Hardening & Repository Management (Dec 22, 2025)
**Problem:** Hardcoded NTRIP credentials exposed in public GitHub repository.
**Discovery:** Security audit revealed sensitive credentials in `gps.launch.py`:
- Username: `grease454`  
- Password: `nacceb-xekva6-cuTbux`
- IP Address: `165.206.203.10`

**Solution (Environment Variable Pattern):**
1. **Credential Removal:** Replaced hardcoded values with `os.getenv()` calls in launch file
2. **Environment Management:** Created `.env` file pattern with `.env.example` template
3. **Git Protection:** Added `.env` to `.gitignore` to prevent future credential exposure
4. **Automation Scripts:** Created convenience scripts that auto-load environment variables:
   - `scripts/load_env.sh` - Environment loader utility
   - `launch_gps.sh` - Auto-load env + GPS launch
   - `launch_rover.sh` - Auto-load env + full rover launch
5. **Repository Cleanup:** Removed all markdown documentation from public repo tracking

**Lessons Learned:**
- **Security First:** Always audit code for hardcoded secrets before initial push
- **Environment Variables:** Use `.env` files for sensitive configuration data
- **Documentation Security:** Personal notes and credentials should never be tracked in public repos
- **Automation:** Convenience scripts eliminate manual steps and reduce credential exposure

**Future Mistake-Proofing:**
- **Pre-commit Hook:** Consider implementing credential scanning in git hooks
- **Template Pattern:** Always create `.env.example` files to document required environment variables
- **Security Checklist:** Audit all configuration files before committing to public repos

### 4.4 Hardware-in-the-Loop Validation & Calibration (Dec 22, 2025)
**Status:** System is now fully localized and calibrated for physical movement.

**Key Achievements:**
1.  **URDF & TF Tree Standardization:**
    *   Created `rover1_description/urdf/rover.urdf.xacro`.
    *   Replaced static transform publishers in `rover.launch.py` with `robot_state_publisher`.
    *   **Baseline:** `base_link` -> `imu_link` / `gps_link` / `wheel links` are now geometrically correct.
    *   **Wheel Spec:** Verified 97mm (0.0485m radius) from official chassis documentation.
2.  **Physical Odometry Calibration:**
    *   **Test:** 1.0 meter forward drive (human-verified measurement).
    *   **Discovery:** `Average Delta Ticks for 1.0m = 12618.75`.
    *   **Calculated Constant (Updated):** `ticks_per_rev = 3845.33` (Based on 97mm wheel circumference).
    *   **Action:** Updated `mecanum_kinematics.py` and URDF with official 97mm dimensions.
3.  **Kinematics & Control Tuning:**
    *   **Rotation Power:** Added `rotation_scale = 2.0` to compensate for Mecanum friction.
    *   **Watchdog Timing:** Settled on **0.5s** timeout in `hiwonder_driver.py` for snappy but stable control.
    *   **Strafing:** Confirmed full 4-axis Mecanum support.
4.  **Infrastructure Robustness:**
    *   **Missing Dependencies:** Added `robot_localization`, `xacro`, and `robot_state_publisher` to `setup_on_pi.sh`.
    *   **Code Bugs:** Resolved `NameError: Odometry` and `SetRemap` import errors.

### 4.5 Battery Monitoring Strategic Pivot (Dec 22, 2025)
**Status:** Independent node implemented; Reliability prioritized.

**Strategic Pivot (Senior Engineer Feedback):**
- **Decoupling:** Battery telemetry moved out of the motor driver to prevent coupling of control and monitoring.
- **Simplification:** Switched from `sensor_msgs/BatteryState` to `std_msgs/Float32`.
- **Topic:** `/battery_voltage` (1Hz).
- **Isolation:** Standalone node `battery_monitor.py` with its own I2C direct access.

**Updated Implementation:**
1.  **Node:** Created `rover1_hardware/battery_monitor.py`.
2.  **Launch:** Added to `rover.launch.py`.
3.  **Clean Up:** Removed telemetry logic from `hiwonder_driver.py` to restore focused motor control.

**Verification Status:**
- **STABILITY:** Stable publication at 1Hz observed via `ros2 topic echo /battery_voltage`.

### 4.6 Field Networking & Failover Strategy (Dec 23, 2025)
**Status:** VERIFIED WORKING - Multi-tier network failover for field demos.

**Problem:** Need reliable connectivity across multiple environments:
- Home lab (WiFi)
- Field demos (phone hotspot)
- Emergency access (direct Ethernet tether)

**Solution (Priority-Based Failover):**
NetworkManager handles WiFi with `autoconnect-priority`, Netplan handles Ethernet static fallback.

| Priority | Network | Type | Connection |
| :--- | :--- | :--- | :--- |
| 1 (Highest) | Lake Wifi | Home WiFi | Auto-connect when in range |
| 2 | AJM17ProMax | Phone Hotspot | Fallback when home unavailable |
| 3 (Always) | Ethernet Tether | Static 10.42.0.1 | Direct laptop connection |

**Prerequisites (Ubuntu 24.04 on Pi):**
```bash
sudo apt update && sudo apt install -y network-manager
```

**Credentials (.env file format):**
- Stored in `.env` file (not tracked in git)
- **IMPORTANT:** Values with spaces must be quoted
```bash
# WiFi Networks (Priority Order)
WIFI_HOME_SSID="Lake Wifi"
WIFI_HOME_PASS="your_password"
WIFI_HOTSPOT_SSID="AJM17ProMax"
WIFI_HOTSPOT_PASS="your_password"
```

**Setup (Run Once on Pi):**
```bash
cd ~/ros2_ws/src/rover1

# Create .env with credentials (values with spaces need quotes!)
nano .env

# Run the setup script
./scripts/setup_network_failover.sh

# Fix netplan permissions warning
sudo chmod 600 /etc/netplan/*.yaml
```

**Verification:**
```bash
# Check configured connections
nmcli connection show

# Check current network status
nmcli device status

# Verify ethernet static IP
ip addr show eth0 | grep inet
# Should show: inet 10.42.0.1/24
```

**Behavior:**
- **At Home:** Connects to Lake Wifi automatically (priority 100)
- **In Field:** When home network unavailable, connects to phone hotspot (priority 50)
- **Emergency:** Ethernet tether always works at `10.42.0.1`

**Manual Network Switching:**
```bash
# Check current connection
nmcli device status

# Force switch to specific network
nmcli connection up "Lake Wifi"
nmcli connection up "AJM17ProMax"

# Disconnect from WiFi to test failover
nmcli connection down "Lake Wifi"
```

**Laptop Ethernet Setup (Mac):**
1. System Settings → Network → Ethernet
2. Configure IPv4: Manually
3. IP Address: `10.42.0.2`
4. Subnet Mask: `255.255.255.0`
5. Apply

**Mac Smart SSH (Auto-Selects Ethernet vs WiFi):**
Added to `~/.zshrc` on Mac - automatically uses ethernet when available:
```bash
# Smart rover SSH - ethernet first, then WiFi
ssh() {
    if [[ "$1" == "andrewmeckley@rover1.local" || "$1" == "rover1.local" ]]; then
        if ping -c1 -W1 10.42.0.1 &>/dev/null; then
            echo "→ Using ethernet (10.42.0.1)"
            command ssh andrewmeckley@10.42.0.1 "${@:2}"
        else
            echo "→ Using WiFi (rover1.local)"
            command ssh andrewmeckley@rover1.local "${@:2}"
        fi
    else
        command ssh "$@"
    fi
}
```

**Usage:** Just type `ssh andrewmeckley@rover1.local` - it auto-detects the best path.

**Netplan Config Created (`/etc/netplan/99-rover-network.yaml`):**
```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: true
      optional: true
      addresses:
        - 10.42.0.1/24
```

**Legacy Script:** `scripts/setup_ethernet_tether.sh` (ethernet-only, superseded by `setup_network_failover.sh`)

### 4.7 Manual Control: Google Stadia Controller Integration (Dec 23, 2025)
**Status:** VERIFIED WORKING - Full Mecanum teleop operational.

**Hardware Specs:**
- **Device:** Google Stadia Controller (Unlocked Bluetooth Mode).
- **MAC Address:** `D1:71:42:54:CB:0F`.
- **System Service:** `bluez` (Bluetoothctl used for pairing/trusting/connecting).

**Software Stack:**
- **Driver:** `ros-jazzy-joy` (`joy_node`) with `deadzone: 0.1` and `autorepeat_rate: 20.0`.
- **Control Logic:** `rover1_hardware/stadia_teleop.py` (Subscribes to `/joy`, Publishes to `/cmd_vel`).

**Controller Mapping (Verified Dec 23, 2025):**
| Input | Axis Index | Range | ROS Mapping | Behavior |
| :--- | :--- | :--- | :--- | :--- |
| **Left Stick Y** | Axis 1 | -1.0 to +1.0 | `linear.x` | Forward (+), Backward (-) |
| **Left Stick X** | Axis 0 | -1.0 to +1.0 | `angular.z` | Rotate Left (+), Rotate Right (-) |
| **Right Stick X** | Axis 2 | -1.0 to +1.0 | `linear.y` | Strafe Left (+), Strafe Right (-) |
| **L2 Trigger** | Axis 5 | +1.0 to -1.0 | Deadman Switch | Released = 1.0, Pressed = -1.0 |

**Tuned Parameters (rover.launch.py):**
- `max_linear_speed: 0.4` m/s
- `max_angular_speed: 0.8` rad/s
- `deadman_threshold: 0.0` (L2 must be pressed past halfway)
- `debug_axes: False` (Set `True` to log raw axis values for recalibration)

**Safety Mechanism:**
- **Dead-Man Switch:** Motion is only permitted when the **L2 Trigger** is depressed (axis < 0.0).
- **Auto-Stop:** Releasing L2 immediately publishes a zero-velocity `Twist` message to `/cmd_vel` to prevent runaway.
- **Axis Validation:** Node checks for sufficient axes before processing to prevent index errors.

**Usage:**
```bash
# Standard launch (joystick enabled by default)
ros2 launch rover1_bringup rover.launch.py use_joy:=true

# Disable joystick for autonomous-only mode
ros2 launch rover1_bringup rover.launch.py use_joy:=false
```

**Bluetooth Reconnection (if controller disconnects):**
```bash
bluetoothctl connect D1:71:42:54:CB:0F
```

**Calibration/Debug Mode:**
```bash
# Verify controller is publishing
ros2 topic echo /joy --field axes

# Enable runtime axis logging
ros2 param set /stadia_teleop debug_axes true
```

**Rebuild Instructions (if code changes):**
```bash
cd ~/ros2_ws
colcon build --packages-select rover1_hardware rover1_bringup --symlink-install
source install/setup.bash
```

### 4.8 Observability & Autonomous Deployment (Dec 24, 2024)
**Status:** SYSTEM VERIFIED - Mission Control Dashboard & Autonomous Startup Operational.

**Key Achievements:**
1.  **Foxglove Bridge Integration:**
    *   Successfully integrated `foxglove_bridge` into `rover.launch.py` (Port 8765).
    *   Resolved buffer overflow issues (`Maximum frame size reached`) by increasing `send_buffer_limit` to 100MB.
    *   **Dashboard Discovery:** Foxglove WebSocket protocol (not Rosbridge) is required for full bandwidth.
2.  **Autonomous Core (The "Brain"):**
    *   Implemented `rover1.service` (Systemd) for hands-free startup.
    *   **Safety Shield:** Built a 15-second "Rescue Delay" into `startup_launch.sh` with a `STOP_ROVER` file bypass and systemd crash backoff.
    *   **Auto-Maintenance:** Service now performs `git pull` automatically on every boot/restart, ensuring the field rover is always on the latest code.
3.  **Sensor Fusion Stabilization:**
    *   Increased Gyroscope calibration from 2s to 15s (1000 samples) to eliminate stationary "yaw creep."
    *   Adjusted EKF Covariance for Yaw (0.5) to allow GPS heading to correct IMU drift over time.
4.  **Stadia Teleop "Uncorking":**
    *   **Ghosting Fix:** Implemented an initialization gate in `stadia_teleop.py`. Node now ignores all input until it sees the L2 trigger report its released state (1.0).
    *   **Performance Mode:** Increased `max_linear_speed` to 2.0 m/s and `max_angular_speed` to 4.0 rad/s based on user performance request.
5.  **Engineering Observability (The "Cockpit"):**
    *   Created `scripts/rover_monitor.sh`.
    *   Provides a high-speed terminal dashboard for:
        *   Systemd service status.
        *   Core Node Health (12 nodes verified).
        *   Topic Heartbeats (Hz rates for IMU, GPS, Battery, and Cmd_Vel).

**Operational Notes:**
- **Refresh Required:** If Foxglove loses connection during a software update, a browser/app refresh is required to clear the message buffer.
- **Wait for Ignition:** Monitor the dashboard; nodes will stay "MISSING" for the first 15 seconds of boot due to the safety window.


## 5. Additional Hardware Capabilities (Extracted from Documentation)
The following capabilities were discovered during PDF audit on Dec 22, 2025:

### 5.1 Motor Driver Board (0x34)
- **Battery Monitoring:** Register `0x00` returns the battery voltage in millivolts.
- **Fixed Speed Mode:** Register `0x33` allows for closed-loop speed control (using internal PID) rather than raw PWM.
- **Total Encoder Ticks:** Register `0x3C` provides total pulse values for all four encoders at once.

### 5.2 IMU (LSM6DSL)
- **Advanced Motion Sensing:** Hardware support for:
    - Free-fall detection.
    - Single/Double tap detection.
    - Built-in pedometer/step counter.
    - Tilt detection (6D/4D orientation).

### 5.3 Physical Limits
- **Payload:** 5.0 kg maximum.
- **Current Load:** Motors draws up to 3.0A at stall (requires robust watchdog for safety).
- **Wheels:** 97mm Diameter, Mecanum type.

**Diagnostic Tools:**
*   `scripts/rover_interactive_check.py`: Full system health check (I2C, Nodes, Topics).
*   `scripts/calibrate_odometry.py`: Re-run if changing wheel sizes or gearboxes.
*   `scripts/debug_nodes.sh`: Individual node startup testing.

---

## 6. RTK GPS Technical Specification (CRITICAL REFERENCE)

**Last Verified:** December 24, 2025
**Hardware:** u-blox ZED-F9R + Iowa DOT RTN (IaRTN)
**Purpose:** This section exists because RTK integration has failed multiple times due to subtle misconfigurations. Follow this specification exactly.

### 6.1 System Architecture (The RTK Data Flow)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         RTK CORRECTION FLOW                                 │
│                                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐ │
│  │  ZED-F9R    │───▶│ublox_dgnss │───▶│  /fix       │───▶│fix_to_nmea │ │
│  │  (USB)      │    │  driver     │    │ (NavSatFix) │    │  bridge     │ │
│  └─────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘ │
│        ▲                                                          │        │
│        │ RTCM                                                     ▼        │
│        │ Corrections                                        ┌───────────┐  │
│  ┌─────┴─────────┐                                          │  /nmea    │  │
│  │/ntrip_client/ │◀───────────────────────────────────────  │ (Sentence)│  │
│  │    rtcm       │         VRS Position Handshake           └─────┬─────┘  │
│  └───────────────┘                                                │        │
│        ▲                                                          ▼        │
│        │                                                   ┌─────────────┐ │
│  ┌─────┴─────────┐                                         │ntrip_client │ │
│  │  Iowa DOT     │◀────────────────────────────────────────│  (VRS)      │ │
│  │  RTN Caster   │         NMEA GPGGA Position             └─────────────┘ │
│  │ 165.206.203.10│                                                         │
│  └───────────────┘                                                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 6.2 Critical Topic Configuration

| Topic | Publisher | Subscriber | Message Type | QoS Reliability |
|-------|-----------|------------|--------------|-----------------|
| `/fix` | ublox_nav_sat_fix_hp | fix_to_nmea, navsat_transform | `sensor_msgs/NavSatFix` | **BEST_EFFORT** |
| `/nmea` | fix_to_nmea | ntrip_client | `nmea_msgs/Sentence` | RELIABLE |
| `/ntrip_client/rtcm` | ntrip_client | ublox_dgnss | `rtcm_msgs/Message` | RELIABLE |

### 6.3 The Three RTK Failure Modes (And How to Diagnose)

#### Failure Mode 1: QoS Mismatch on /fix
**Symptom:** `ros2 topic echo /fix` returns nothing (hangs or times out).
**Root Cause:** The ublox_dgnss driver publishes with BEST_EFFORT QoS. Default subscribers use RELIABLE.
**Diagnosis:**
```bash
# This will FAIL (hang/timeout):
ros2 topic echo /fix --once

# This will SUCCEED:
ros2 topic echo /fix --qos-reliability best_effort --once
```
**Fix:** All subscribers to `/fix` must use BEST_EFFORT QoS:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
self.create_subscription(NavSatFix, '/fix', callback, qos)
```

#### Failure Mode 2: Message Type Mismatch on /nmea
**Symptom:** NTRIP client stays IDLE, no RTCM corrections flow.
**Root Cause:** `fix_to_nmea` publishes wrong message type.
**Diagnosis:**
```bash
ros2 topic info /nmea -v
# Check that BOTH publisher AND subscriber show: nmea_msgs/msg/Sentence
# If publisher shows std_msgs/msg/String → MISMATCH
```
**Fix:** `fix_to_nmea.py` MUST publish `nmea_msgs/msg/Sentence`, NOT `std_msgs/msg/String`:
```python
from nmea_msgs.msg import Sentence
nmea_msg = Sentence()
nmea_msg.header.stamp = self.get_clock().now().to_msg()
nmea_msg.header.frame_id = 'gps_link'
nmea_msg.sentence = "$GPGGA,..."
```

#### Failure Mode 3: RTCM Topic Routing Mismatch
**Symptom:** NTRIP receives RTCM but GPS stays in Standard 3D mode (no RTK).
**Root Cause:** NTRIP publishes to `/rtcm`, but ublox subscribes to `/ntrip_client/rtcm`.
**Diagnosis:**
```bash
# Check what ntrip_client publishes:
ros2 node info /ntrip_client | grep Publishers
# Check what ublox_dgnss subscribes to:
ros2 node info /ublox_dgnss | grep -A3 Subscribers
# If they don't match → no RTCM delivery
```
**Fix:** Add remapping in `gps.launch.py`:
```python
Node(
    package='ntrip_client',
    executable='ntrip_ros.py',
    remappings=[('/rtcm', '/ntrip_client/rtcm')]
)
```

### 6.4 Required Package Dependencies

```bash
# Install these on the Pi:
sudo apt install -y ros-jazzy-nmea-msgs ros-jazzy-rtcm-msgs ros-jazzy-ntrip-client ros-jazzy-ublox-dgnss
```

In `rover1_hardware/package.xml`:
```xml
<depend>nmea_msgs</depend>
```

### 6.5 RTK Verification Checklist

Run these commands IN ORDER after every software rebuild:

```bash
# 1. Source environment
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

# 2. Verify USB device is claimed by driver (not cdc_acm)
lsusb | grep u-blox  # Should show device
sudo lsof /dev/bus/usb/002/*  # Should show "component" process

# 3. Verify raw GPS is working (28+ satellites expected outdoors)
timeout 5 ros2 topic echo /ubx_nav_pvt --once | grep -E "num_sv|fix_type"
# Expected: num_sv: 25+, fix_type: 3

# 4. Verify /fix is publishing (MUST use best_effort QoS)
timeout 5 ros2 topic echo /fix --qos-reliability best_effort --once
# Expected: latitude/longitude values

# 5. Verify NMEA bridge is working (correct message type)
ros2 topic info /nmea -v | grep "Topic type"
# Expected: nmea_msgs/msg/Sentence (BOTH publisher AND subscriber)

timeout 3 ros2 topic echo /nmea --once
# Expected: $GPGGA sentence

# 6. Verify RTCM is flowing to correct topic
timeout 5 ros2 topic echo /ntrip_client/rtcm --once
# Expected: Binary RTCM data (not empty)

# 7. Verify RTK status
timeout 5 ros2 topic echo /ubx_nav_pvt --once | grep -E "diff_soln|carr_soln"
# Expected: diff_soln: true, carr_soln.status: 1 (float) or 2 (fixed)

# 8. Verify accuracy improvement
timeout 5 ros2 topic echo /ubx_nav_pvt --once | grep h_acc
# Expected: h_acc < 100 (sub-meter), ideally < 50 (RTK float) or < 20 (RTK fixed)
```

### 6.6 NTRIP Caster Configuration (Iowa DOT RTN)

```yaml
Host: 165.206.203.10
Port: 10000
Mountpoint: RTCM3_IMAX
Authentication: Basic Auth (username/password in .env file)
Protocol: NTRIP v1 with VRS
```

**VRS Requirement:** This is a Virtual Reference Station network. The caster REQUIRES the rover's position (NMEA GPGGA) to generate corrections. Without the `fix_to_nmea` bridge, NTRIP will connect but send no data.

### 6.7 RTK Status Codes Reference

| Field | Value | Meaning |
|-------|-------|---------|
| `gps_fix.fix_type` | 0 | No fix |
| `gps_fix.fix_type` | 2 | 2D fix |
| `gps_fix.fix_type` | 3 | 3D fix |
| `diff_soln` | false | No differential corrections |
| `diff_soln` | true | RTCM corrections being applied |
| `carr_soln.status` | 0 | No carrier solution |
| `carr_soln.status` | 1 | **RTK Float** (~5-50cm accuracy) |
| `carr_soln.status` | 2 | **RTK Fixed** (~1-2cm accuracy) |

### 6.8 Expected Accuracy by Mode

| Mode | Horizontal Accuracy (h_acc) | Typical Value |
|------|----------------------------|---------------|
| Standard 3D Fix | 1000-5000 mm | ~1.2 meters |
| RTK Float | 20-100 mm | ~5 cm |
| RTK Fixed | 10-30 mm | ~2 cm |

### 6.9 Troubleshooting Decision Tree

```
GPS not working?
├── Is USB device visible? (lsusb | grep u-blox)
│   └── NO → Check USB cable, power, physical connection
│   └── YES → Continue
├── Is /ubx_nav_pvt publishing? (ros2 topic echo /ubx_nav_pvt --once)
│   └── NO → Driver issue. Check journalctl -u rover1.service
│   └── YES → Continue
├── Is /fix publishing? (ros2 topic echo /fix --qos-reliability best_effort --once)
│   └── NO → QoS mismatch or ublox_nav_sat_fix_hp not running
│   └── YES → Continue
├── Is /nmea publishing correct type? (ros2 topic info /nmea -v)
│   └── Wrong type → Fix fix_to_nmea.py to use nmea_msgs/Sentence
│   └── Correct type → Continue
├── Is /ntrip_client/rtcm receiving data? (ros2 topic echo /ntrip_client/rtcm --once)
│   └── NO → Check NTRIP credentials, network, topic remapping
│   └── YES → Continue
├── Is diff_soln: true?
│   └── NO → RTCM not reaching GPS. Check topic routing.
│   └── YES → RTK is working! Check carr_soln.status for float/fixed.
```

---

### 4.9 GPS/RTK "Silent Fix" Debugging Session (Dec 24, 2025)
**Status:** ROOT CAUSE IDENTIFIED & FIXED - GPS was working the whole time!

**Initial Symptoms:**
1. `ros2 topic echo /fix` returned nothing (appeared silent).
2. Monitor dashboard hung when checking GPS Hz rates.
3. NTRIP client remained IDLE (no RTCM corrections flowing).
4. System appeared to have no GPS lock despite hardware being confirmed.

**Diagnostic Process:**

**Step 1: Hardware Layer Verification**
```bash
lsusb | grep -i u-blox
# Result: Bus 002 Device 002: ID 1546:01a9 U-Blox AG u-blox GNSS receiver ✓

sudo dmesg | grep cdc_acm
# Result: "probe of 2-1:1.0 failed with error -16" (EBUSY)
# Interpretation: cdc_acm tried but FAILED - something else has the device ✓

sudo lsof /dev/bus/usb/002/002
# Result: component PID 62728 has the device
# Interpretation: ublox_dgnss driver successfully claimed USB via libusb ✓
```

**Step 2: ROS Topic Investigation**
```bash
ros2 node list | grep ublox
# Result: /ublox_dgnss, /ublox_nav_sat_fix_hp both running ✓

ros2 topic list | grep ubx
# Result: 30+ ubx_ topics exist ✓

timeout 3 ros2 topic echo /ubx_nav_pvt --once
# Result: FULL GPS DATA! 28 satellites, 3D fix, valid coordinates ✓
```

**Critical Discovery:** The GPS was fully operational. The `/ubx_nav_pvt` topic showed:
- `gps_fix.fix_type: 3` (3D fix)
- `gnss_fix_ok: true`
- `num_sv: 28` (28 satellites!)
- Valid Iowa coordinates (41.59°N, -90.49°W)
- Horizontal accuracy: 1.2 meters

**Root Cause #1: QoS Mismatch on /fix Topic**
```bash
timeout 3 ros2 topic echo /fix --once
# Result: SILENT (timeout)

timeout 3 ros2 topic echo /fix --qos-reliability best_effort --once
# Result: FULL DATA! NavSatFix with lat/lon/alt
```

The `ublox_dgnss` driver publishes `/fix` with **BEST_EFFORT** QoS (SensorData profile).
Default `ros2 topic echo` uses **RELIABLE** QoS. These are incompatible in ROS 2 DDS.

**Root Cause #2: Message Type Mismatch on /nmea Topic**
```bash
ros2 topic info /nmea -v
# Publisher: fix_to_nmea → std_msgs/msg/String
# Subscriber: ntrip_client → nmea_msgs/msg/Sentence
```

The NTRIP client expects `nmea_msgs/msg/Sentence` but `fix_to_nmea` was publishing `std_msgs/msg/String`.
These types are incompatible - the messages were never delivered.

**The Fix:**

1. **fix_to_nmea.py v2.0:**
   - Changed publisher from `std_msgs/msg/String` to `nmea_msgs/msg/Sentence`
   - Subscriber QoS set to BEST_EFFORT (matches ublox_dgnss)
   - Publisher QoS set to RELIABLE (matches ntrip_client)
   - Added proper header with timestamp and frame_id

2. **rover_monitor.sh v2.7:**
   - Added `--qos-reliability best_effort` to `/fix` topic commands
   - Increased timeouts from 0.8s to 1.5s for Pi 5 performance
   - Fixed typo: "HEARTBEETS" → "HEARTBEATS"

3. **package.xml:**
   - Added `nmea_msgs` dependency to rover1_hardware

**Lessons Learned:**
- **QoS Matters:** Always check publisher QoS before assuming a topic is dead.
- **Type Safety:** ROS 2 message types must match exactly between publisher/subscriber.
- **Debug Command:** `ros2 topic info /topic_name -v` shows QoS and type for both ends.
- **Diagnostic Sequence:**
  1. Check hardware layer first (lsusb, dmesg, lsof)
  2. Check node existence (ros2 node list)
  3. Check raw driver topics before processed topics
  4. Always specify QoS when echoing sensor topics

