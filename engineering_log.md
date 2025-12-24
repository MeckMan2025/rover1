# Rover1 Engineering Journal & Technical Specifications

**Maintainer:** MeckMan2025
**Last Updated:** 2025-12-22
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

