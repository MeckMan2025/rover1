# Rover1 Motor Mapping & Hardware Registry

**Last Updated:** 2025-12-21
**Hardware:** Hiwonder Driver HAT (I2C 0x34) + Custom Chassis Wiring

## 1. I2C Register Protocol
The Hiwonder driver has been patched to use direct register addressing for reliable individual motor control.
The standard `[MotorID, Speed]` packet protocol was found to be unreliable (broadcasted to M1 only).

| Register Address | Function | Data Type | Range |
| :--- | :--- | :--- | :--- |
| **0x33 (51)** | **Rear Left** Speed | Signed Byte | -100 to 100 |
| **0x34 (52)** | **Front Left** Speed | Signed Byte | -100 to 100 |
| **0x35 (53)** | **Rear Right** Speed | Signed Byte | -100 to 100 |
| **0x36 (54)** | **Front Right** Speed | Signed Byte | -100 to 100 |

*> Note: Registers are decimal 51-54.*

## 2. Physical Wiring & Mapping
The physical wiring determines which motor corresponds to which register.

| Physical Wheel | I2C Register | Driver Index (0-3) |
| :--- | :--- | :--- |
| **Front Left** | **52** | 0 |
| **Front Right** | **54** | 1 |
| **Rear Left** | **51** | 2 |
| **Rear Right** | **53** | 3 |

## 3. Polarity Settings
Due to motor orientation on the chassis, some motors require inversion in software (`rover.launch.py`) to ensure positive values move the rover **FORWARD**.

| Wheel | Native Direction* | Software Invert? | Parameter Name |
| :--- | :--- | :--- | :--- |
| **Front Left** | Backward | **YES** | `invert_fl` |
| **Front Right** | Forward | NO | `invert_fr` |
| **Rear Left** | Forward | NO | `invert_rl` |
| **Rear Right** | Backward | **YES** | `invert_rr` |

*> Native Direction = Direction spun when sent +100 raw command.*

## 4. Software Implementation
- **Driver:** `rover1_hardware/hiwonder_driver.py`
- **Launch:** `rover1_bringup/launch/rover.launch.py`

### Safety Watchdog
The driver tracks the last message time. If no `wheel_speeds_raw` message is received for **0.5 seconds**, the driver automatically writes `0` to all registers (51-54) to prevent runaway conditions.
