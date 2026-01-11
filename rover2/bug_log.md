# Rover1 Bug Log & Remediation

## 🔴 CRITICAL: Runaway Motor & Mapping Issue (2025-12-21)

### Incident Description
**STATUS: RESOLVED (2025-12-21)**
- **Solution**: Updated `hiwonder_driver.py` to use direct register addressing (51-54) and added polarity parameters.
- **Verification**: All 4 wheels spin forward on command and stop on timeout.
- **Documentation**: See `motor_mapping.md`.

During initial motor verification:
1.  **Command Sent**: `ros2 topic pub ... /wheel_speeds_raw ... [100, 0, 0, 0]` (Forward M1).
2.  **Observation**:
    - Front Left (FL) motor spun **backward rapidly**.
    - Rear Left (RL) motor spun **backward slowly** (unexpected, as command was 0).
3.  **Command Sent**: `... [0, 0, 0, 0]` (STOP).
4.  **Observation**: RL motor **continued spinning**.
5.  **Reboot Attempt**: `sudo reboot` failed to execute immediately (likely hung on process termination).
6.  **Resolution**: Hard power cycle required.

### Root Cause Analysis
1.  **Safety Watchdog Missing**: `hiwonder_driver.py` has no timeout. If the ROS node dies, crashes, or loses connection, the I2C bus retains the last written value (Motor Latching).
2.  **Mapping/Polarity Error**:
    - M1 (FL) is inverted (Wiring vs Code).
    - RL behavior implies cross-talk or register addressing issues (Slow backward spin on 0 command).

### Immediate Remediation Plan (Effect upon Reboot)
1.  **Update `hiwonder_driver.py`**:
    - **Add Safety Watchdog**: If no `wheel_speeds_raw` message received for >0.5s, write `0` to all motors repeatedly.
    - **Add Polarity Parameters**: Allow flipping signs via launch file.
    - **Verify Register Map**: Confirm Register 51 usage or switch to individual motor control registers if available.

### Remediation Code (Implementation Pending)
```python
# Pseudo-code for Watchdog
def timer_callback(self):
    if (self.get_clock().now() - self.last_msg_time).nanoseconds > 0.5e9:
        self.stop_motors()
```

## 🟠 GPS NTRIP Authorization Failure (2025-12-21)
- **Error**: `HTTP/1.0 401 Unauthorized` for `165.206.203.10` Mount: `RTCM3_MAX`.
- **Status**: Credentials updated. Retesting next session.
