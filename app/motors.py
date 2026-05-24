"""Hiwonder 4-channel I2C motor + battery board — standalone, no ROS.

Extracted from the legacy ROS 2 nodes:
  rover1_hardware/rover1_hardware/hiwonder_driver.py
  rover1_hardware/rover1_hardware/battery_monitor.py

Register map, polarity handling, and signed-byte protocol preserved verbatim
from the version the user verified on hardware.
"""

from __future__ import annotations

import threading

import smbus2

_BUS_NUM = 1
_ADDRESS = 0x34

# Battery voltage register: 2 bytes little-endian, millivolts.
_REG_BATTERY = 0x00

# Motor registers — verified on hardware. Do not "clean up" these numbers.
# Mapping based on User Verification:
#   FL (Input 0) -> Reg 52
#   FR (Input 1) -> Reg 54
#   RL (Input 2) -> Reg 51
#   RR (Input 3) -> Reg 53
_REG_FL = 52
_REG_FR = 54
_REG_RL = 51
_REG_RR = 53

_PWM_LIMIT = 100


def _clamp(v: int) -> int:
    return max(-_PWM_LIMIT, min(_PWM_LIMIT, int(v)))


def _signed_to_unsigned_byte(v: int) -> int:
    # Hiwonder protocol: negative values are encoded as 256 + value.
    return 256 + v if v < 0 else v


class HiwonderHardware:
    """Thread-safe direct I2C access to the Hiwonder motor/battery board."""

    # Default polarity for the rover1 chassis. Front-Left and Rear-Right motors
    # are physically wired backward (per motor_mapping.md section 3), so a raw
    # +100 makes them spin reverse. We flip their signs in software so that
    # positive = forward at the API boundary. The legacy ROS launch files
    # passed invert_fl=True, invert_rr=True for the same reason.
    _DEFAULT_POLARITY = (-1, +1, +1, -1)

    def __init__(
        self,
        bus_num: int = _BUS_NUM,
        address: int = _ADDRESS,
        polarity: tuple[int, int, int, int] = _DEFAULT_POLARITY,
    ) -> None:
        self.address = address
        self._polarity = polarity  # (fl, fr, rl, rr), each +1 or -1
        self._lock = threading.Lock()
        self._bus = smbus2.SMBus(bus_num)

    def set_wheel_speeds(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """Drive the four wheels at the given speeds (driver units, ±100).

        Polarity supplied at construction time is applied here so callers
        always speak in "logical" forward = positive.
        """
        p_fl, p_fr, p_rl, p_rr = self._polarity
        writes = (
            (_REG_FL, _signed_to_unsigned_byte(_clamp(fl * p_fl))),
            (_REG_FR, _signed_to_unsigned_byte(_clamp(fr * p_fr))),
            (_REG_RL, _signed_to_unsigned_byte(_clamp(rl * p_rl))),
            (_REG_RR, _signed_to_unsigned_byte(_clamp(rr * p_rr))),
        )
        with self._lock:
            for reg, val in writes:
                self._bus.write_byte_data(self.address, reg, val)

    def read_battery_voltage(self) -> float:
        """Battery voltage in volts (register 0x00, 2 bytes LE mV)."""
        with self._lock:
            data = self._bus.read_i2c_block_data(self.address, _REG_BATTERY, 2)
        mv = data[0] + (data[1] << 8)
        return mv / 1000.0

    def stop(self) -> None:
        self.set_wheel_speeds(0, 0, 0, 0)

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self._bus.close()

    def __enter__(self) -> "HiwonderHardware":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
