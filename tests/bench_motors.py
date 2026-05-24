#!/usr/bin/env python3
"""Hiwonder bench test.

ROVER MUST BE ON STANDS OR OFF THE GROUND. Wheels will spin.

Reads battery voltage, then runs six gentle motions (~1.5s each) with brief
stops between them. Prints the commanded wheel speeds so any unexpected
behavior can be diagnosed against the kinematics math, not just observed.

Run from the repo root:
    python tests/bench_motors.py
"""

from __future__ import annotations

import os
import sys
import time

# Make `app` importable when running this script directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.kinematics import cartesian_to_wheels  # noqa: E402
from app.motors import HiwonderHardware         # noqa: E402

LINEAR = 0.2   # m/s
ANGULAR = 0.5  # rad/s
HOLD = 1.5     # seconds of motion
PAUSE = 0.4    # seconds of stop between motions


def step(label: str, vx: float, vy: float, omega: float, hw: HiwonderHardware) -> None:
    wheels = cartesian_to_wheels(vx, vy, omega)
    print(
        f"[{label}]  vx={vx:+.2f}  vy={vy:+.2f}  omega={omega:+.2f}  "
        f"-> FL={wheels[0]:+4d} FR={wheels[1]:+4d} "
        f"RL={wheels[2]:+4d} RR={wheels[3]:+4d}"
    )
    hw.set_wheel_speeds(*wheels)
    time.sleep(HOLD)
    hw.stop()
    time.sleep(PAUSE)


def main() -> None:
    with HiwonderHardware() as hw:
        v = hw.read_battery_voltage()
        print(f"Battery: {v:.2f} V")
        if v < 9.0:
            print("WARNING: battery looks low; charge before extended testing.")
        try:
            input("Rover is on stands? Press ENTER to start (Ctrl-C aborts) ... ")
        except (KeyboardInterrupt, EOFError):
            print("aborted before motion")
            return
        try:
            step("FORWARD",                          +LINEAR,       0,        0, hw)
            step("BACKWARD",                         -LINEAR,       0,        0, hw)
            step("VY POSITIVE (ROS: strafe LEFT)",         0, +LINEAR,        0, hw)
            step("VY NEGATIVE (ROS: strafe RIGHT)",        0, -LINEAR,        0, hw)
            step("OMEGA POSITIVE (ROS: rotate CCW)",       0,       0, +ANGULAR, hw)
            step("OMEGA NEGATIVE (ROS: rotate CW)",        0,       0, -ANGULAR, hw)
        finally:
            hw.stop()
        print("Done. Wheels stopped.")


if __name__ == "__main__":
    main()
