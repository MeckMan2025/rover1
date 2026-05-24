"""Mecanum inverse kinematics — pure math, no I/O.

Translates body-frame velocity (vx, vy, omega) into 4 wheel speeds in driver
units (±PWM_LIMIT). Formulas and constants preserved verbatim from the legacy
ROS node at rover1_hardware/rover1_hardware/mecanum_kinematics.py.
"""

from __future__ import annotations

# Robot geometry
WHEEL_SEPARATION_WIDTH = 0.20   # meters, left↔right
WHEEL_SEPARATION_LENGTH = 0.16  # meters, front↔back
WHEEL_RADIUS = 0.04             # meters

# Motor / driver characteristics
MAX_RPM = 200
PWM_LIMIT = 100

# Angular response boost — kept identical to the legacy node's default.
ROTATION_SCALE = 2.0


def cartesian_to_wheels(
    vx: float, vy: float, omega: float
) -> tuple[int, int, int, int]:
    """Body-frame velocity → (fl, fr, rl, rr) wheel speeds in driver units.

    vx:    m/s, +forward
    vy:    m/s, ROS convention is +left. Confirm sign against rover behavior
           during bench testing — if observed direction is opposite, the
           legacy code may have been driven with non-ROS convention.
    omega: rad/s, ROS convention is +CCW about z.

    Returns ints clamped to ±PWM_LIMIT.
    """
    lx = WHEEL_SEPARATION_WIDTH / 2.0
    ly = WHEEL_SEPARATION_LENGTH / 2.0
    r = WHEEL_RADIUS
    k = lx + ly
    wz = omega * ROTATION_SCALE

    w_fl = (vx - vy - k * wz) / r
    w_fr = (vx + vy + k * wz) / r
    w_rl = (vx + vy - k * wz) / r
    w_rr = (vx - vy + k * wz) / r

    return (
        _to_driver_units(w_fl),
        _to_driver_units(w_fr),
        _to_driver_units(w_rl),
        _to_driver_units(w_rr),
    )


def _to_driver_units(rad_s: float) -> int:
    rpm = rad_s * 9.55  # rad/s → rpm
    norm = rpm / MAX_RPM
    val = int(norm * PWM_LIMIT)
    return max(-PWM_LIMIT, min(PWM_LIMIT, val))
