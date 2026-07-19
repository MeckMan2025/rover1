"""GPS waypoint follower — drive a surveyed path, stop at the end.

Consumes the same status file the dashboard chip reads (written by
scripts/ntrip_rtk.py at 2 Hz, now with full-precision lat/lon plus course
over ground and speed) and steers the rover along a pre-surveyed path CSV
from field_data/ (see field_data/sidewalk-test-2026-07-19/).

Heading without a compass
-------------------------
The rover has no magnetometer, so heading comes from the receiver's course
over ground (NAV-PVT headMot): valid whenever the rover is actually moving.
That only equals body heading if we never strafe — so this follower drives
forward-only (vy = 0 always), like a car. Between 2 Hz GPS samples the
heading estimate is propagated by integrating the commanded yaw rate; each
fresh sample with believable course (speed and headAcc gates) snaps the
estimate back to truth. A short ALIGN phase at the start creeps straight
forward until the receiver reports a confident course, because a parked
receiver has no course at all.

Control
-------
Pure pursuit on the densified path: find the nearest path point (searched
monotonically forward so the rover can't lock onto a later, parallel leg),
aim at the point LOOKAHEAD_M further along, and steer the heading error
down with a P term. Forward speed backs off as heading error grows and the
rover pivots in place beyond 60°.

Safety
------
Commands go through MotorTarget.set_drive(source="follow"), so the E-stop
latch and the manual-joystick override behave exactly as they do for the
camera follower. On top of that the loop itself:
  * requires RTK-grade accuracy (hAcc <= HACC_GATE_M) and pauses — motors
    zeroed — the moment fix quality or freshness drops, resuming only when
    it recovers (faults out if it doesn't within PAUSE_FAULT_S);
  * refuses to start further than START_RADIUS_M from the path start;
  * aborts if the rover strays XTRACK_ABORT_M off the path, makes no
    progress for PROGRESS_STALL_S, or exceeds MISSION_TIMEOUT_S overall.
The 10 Hz tick also simply keeps the 150 ms stale-target motor watchdog
fed — if this thread dies for any reason, the wheels stop on their own.
"""

from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

FIELD_DATA_DIR = Path(__file__).resolve().parent.parent / "field_data"
GPS_STATUS_PATH = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "rover-gps-status.json"

TICK_HZ = 10.0            # control/command rate; >> the 150 ms motor watchdog
CRUISE_VX = 0.5           # normalized; 0.5 * MAX_LINEAR = 0.25 m/s at power 1.0
ALIGN_VX = 0.35           # gentler creep while establishing heading
OMEGA_MAX = 0.5           # normalized yaw clamp while tracking
YAW_FULL_ERR_DEG = 45.0   # heading error that saturates the yaw command (P gain)
PIVOT_ERR_DEG = 60.0      # beyond this, stop and pivot in place
LOOKAHEAD_M = 1.2         # pure-pursuit carrot distance along the path
STOP_RADIUS_M = 0.5       # arrival: this close to the final point = done
START_RADIUS_M = 3.0      # refuse to start further than this from the path start
HACC_GATE_M = 0.5         # required accuracy — RTK float passes, plain 3D won't
GPS_STALE_S = 4.0         # status-file age beyond which we pause (writer is 2 Hz)
PAUSE_FAULT_S = 30.0      # mid-run RTK loss longer than this -> abort
ACQUIRE_FAULT_S = 180.0   # initial fix budget — float convergence takes ~2 min
XTRACK_ABORT_M = 2.0      # this far off the path -> abort (wrong sidewalk!)
ALIGN_MAX_DIST_M = 3.0    # creep distance allowed to establish heading
COG_MIN_SPEED_MPS = 0.12  # course over ground is noise below this speed
COG_ACC_GATE_DEG = 30.0   # receiver's own headAcc must be at least this good
PROGRESS_STALL_S = 20.0   # no forward progress for this long -> abort
PROGRESS_EPS_M = 0.3      # what counts as progress
MISSION_TIMEOUT_S = 300.0 # hard ceiling on a run
NEAREST_WINDOW = 80       # path points searched ahead of the current index

# Commanded-omega -> expected yaw rate, for dead reckoning between GPS
# samples. Mirrors kinematics: omega_norm * MAX_ANGULAR * ROTATION_SCALE.
# Open-loop and slip-blind, but every confident GPS course resets the error.
NOMINAL_YAW_RATE = 1.0 * 2.0  # rad/s at omega_norm == 1.0


def _wrap_deg(a: float) -> float:
    """Wrap an angle difference to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


class _Path:
    """A surveyed path in local ENU meters with cumulative distance."""

    def __init__(self, csv_path: Path) -> None:
        lats, lons = [], []
        with open(csv_path, newline="") as f:
            for row in csv.DictReader(f):
                lats.append(float(row["lat"]))
                lons.append(float(row["lon"]))
        if len(lats) < 2:
            raise ValueError(f"path {csv_path.name} has fewer than 2 points")
        self.name = csv_path.name
        self.lat0, self.lon0 = lats[0], lons[0]
        # Local-tangent-plane scale factors (same series the path generator
        # used, so the CSV's own x/y columns and ours agree to the mm).
        self.mlat = 111132.95 - 559.85 * math.cos(2 * math.radians(self.lat0))
        self.mlon = (111412.88 * math.cos(math.radians(self.lat0))
                     - 93.5 * math.cos(3 * math.radians(self.lat0)))
        self.xy = [self.to_enu(la, lo) for la, lo in zip(lats, lons)]
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(self.xy, self.xy[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        self.total_m = self.cum[-1]

    def to_enu(self, lat: float, lon: float) -> tuple[float, float]:
        return ((lon - self.lon0) * self.mlon, (lat - self.lat0) * self.mlat)


class GpsWaypointNav:
    """Background GPS path follower. One instance for the app's lifetime;
    start() begins a run, stop() aborts it, status() feeds /telemetry."""

    def __init__(self, motor_target, *,
                 status_path: Path = GPS_STATUS_PATH) -> None:
        self._target = motor_target
        self._status_path = status_path

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        # Serializes start/stop/close against each other (mirrors BoxFollower's
        # lifecycle lock) so rapid API calls can't leave two loops running.
        self._lifecycle = threading.Lock()

        self._lock = threading.Lock()
        self._state = "idle"      # idle|acquiring|aligning|tracking|paused|done|fault
        self._msg = ""
        self._path_name: Optional[str] = None
        self._progress_m = 0.0
        self._total_m = 0.0
        self._dist_to_goal_m: Optional[float] = None
        self._xtrack_m: Optional[float] = None
        self._heading_deg: Optional[float] = None

    # --- public API ---------------------------------------------------------

    @property
    def enabled(self) -> bool:
        t = self._thread
        return t is not None and t.is_alive()

    def status(self) -> dict:
        with self._lock:
            return {
                "nav_state": self._state,
                "nav_msg": self._msg,
                "nav_path": self._path_name,
                "nav_progress_m": round(self._progress_m, 2),
                "nav_total_m": round(self._total_m, 2),
                "nav_dist_to_goal_m": (round(self._dist_to_goal_m, 2)
                                       if self._dist_to_goal_m is not None else None),
                "nav_xtrack_m": (round(self._xtrack_m, 2)
                                 if self._xtrack_m is not None else None),
                "nav_heading_deg": (round(self._heading_deg, 1)
                                    if self._heading_deg is not None else None),
            }

    def start(self, rel_path: str) -> tuple[bool, str]:
        """Begin following `rel_path` (relative to field_data/). Returns
        (ok, message); refusals are messages, not exceptions, so the API
        layer can hand them straight to the operator."""
        with self._lifecycle:
            self._halt_locked()
            try:
                csv_path = (FIELD_DATA_DIR / rel_path).resolve()
                if not str(csv_path).startswith(str(FIELD_DATA_DIR.resolve()) + os.sep):
                    return False, "path must live under field_data/"
                path = _Path(csv_path)
            except (OSError, ValueError, KeyError) as e:
                return False, f"could not load path: {e}"

            with self._lock:
                self._state = "acquiring"
                self._msg = "waiting for RTK-grade fix"
                self._path_name = path.name
                self._progress_m = 0.0
                self._total_m = path.total_m
                self._dist_to_goal_m = None
                self._xtrack_m = None
                self._heading_deg = None
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run, args=(path,), daemon=True, name="gps-nav")
            self._thread.start()
            return True, f"following {path.name} ({path.total_m:.1f} m)"

    def stop(self) -> None:
        with self._lifecycle:
            self._halt_locked()

    def close(self) -> None:
        self.stop()

    def _halt_locked(self) -> None:
        """Stop the run thread and zero the target. Caller holds _lifecycle."""
        self._stop_event.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        self._thread = None
        self._target.set_drive(0.0, 0.0, 0.0, source="follow")
        with self._lock:
            if self._state not in ("done", "fault"):
                self._state = "idle"
                self._msg = ""

    # --- GPS input ----------------------------------------------------------

    def _read_gps(self) -> Optional[dict]:
        """Latest usable RTK sample, or None (missing/stale/inaccurate).
        Freshness rides on the status file's mtime — the writer replaces it
        atomically at 2 Hz, so a stale mtime means the NTRIP client is down."""
        try:
            st = self._status_path.stat()
            if time.time() - st.st_mtime > GPS_STALE_S:
                return None
            s = json.loads(self._status_path.read_text())
        except (OSError, ValueError):
            return None
        if s.get("lat") is None or s.get("lon") is None:
            return None
        hacc = s.get("hacc_m")
        if hacc is None or hacc > HACC_GATE_M:
            return None
        s["_mtime"] = st.st_mtime
        return s

    # --- control loop -------------------------------------------------------

    def _run(self, path: _Path) -> None:
        period = 1.0 / TICK_HZ
        t_start = time.monotonic()
        last_mtime = 0.0

        pos: Optional[tuple[float, float]] = None  # ENU meters
        heading: Optional[float] = None            # deg, 0=N, CW positive
        idx = 0                                    # monotonic nearest-point index
        phase = "acquiring"
        align_origin: Optional[tuple[float, float]] = None
        pause_since: Optional[float] = None
        last_progress = (0.0, time.monotonic())    # (meters, when it advanced)
        omega_cmd = 0.0
        last_tick = time.monotonic()

        def publish(state: str, msg: str = "") -> None:
            with self._lock:
                self._state = state
                self._msg = msg
                self._progress_m = path.cum[idx]
                self._heading_deg = heading
                if pos is not None:
                    gx, gy = path.xy[-1]
                    self._dist_to_goal_m = math.hypot(gx - pos[0], gy - pos[1])
                    px, py = path.xy[idx]
                    self._xtrack_m = math.hypot(px - pos[0], py - pos[1])

        def finish(state: str, msg: str) -> None:
            self._target.set_drive(0.0, 0.0, 0.0, source="follow")
            publish(state, msg)

        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now

            if now - t_start > MISSION_TIMEOUT_S:
                finish("fault", f"mission timeout ({MISSION_TIMEOUT_S:.0f}s)")
                return
            if self._target.estopped:
                finish("fault", "e-stop")
                return

            gps = self._read_gps()
            if gps is None:
                # Lost RTK / stale file: freeze and wait for it to come back.
                if pause_since is None:
                    pause_since = now
                    omega_cmd = 0.0
                    self._target.set_drive(0.0, 0.0, 0.0, source="follow")
                limit = ACQUIRE_FAULT_S if phase == "acquiring" else PAUSE_FAULT_S
                if now - pause_since > limit:
                    finish("fault", "no RTK-grade fix" if phase == "acquiring"
                           else "GPS quality lost and did not recover")
                    return
                if phase != "acquiring":
                    publish("paused", "waiting for RTK-grade fix")
                time.sleep(period)
                continue
            if pause_since is not None:
                pause_since = None
                # The rover sat still through the blackout (heading estimate
                # survives), but the stall clock must not count the pause.
                last_progress = (path.cum[idx], now)

            fresh = gps["_mtime"] > last_mtime
            if fresh:
                last_mtime = gps["_mtime"]
                pos = path.to_enu(gps["lat"], gps["lon"])
                spd = gps.get("speed_mps") or 0.0
                cacc = gps.get("cog_acc_deg")
                cog = gps.get("cog_deg")
                if (cog is not None and spd >= COG_MIN_SPEED_MPS
                        and cacc is not None and cacc <= COG_ACC_GATE_DEG):
                    heading = cog  # truth beats dead reckoning
                # Advance the nearest-point index, forward-only.
                best_d, best_j = None, idx
                for j in range(idx, min(idx + NEAREST_WINDOW, len(path.xy))):
                    px, py = path.xy[j]
                    d = math.hypot(px - pos[0], py - pos[1])
                    if best_d is None or d < best_d:
                        best_d, best_j = d, j
                idx = best_j
                # Not while acquiring — the start-radius check owns that case
                # and gives the operator a "move it to spot 1" message instead.
                if (phase != "acquiring" and best_d is not None
                        and best_d > XTRACK_ABORT_M):
                    finish("fault", f"{best_d:.1f} m off path — aborting")
                    return
                if path.cum[idx] >= last_progress[0] + PROGRESS_EPS_M:
                    last_progress = (path.cum[idx], now)

            if pos is None:
                time.sleep(period)
                continue

            # Arrival check runs every tick — never drive past the goal
            # waiting for the next phase transition.
            gx, gy = path.xy[-1]
            dist_goal = math.hypot(gx - pos[0], gy - pos[1])
            if dist_goal <= STOP_RADIUS_M and phase in ("tracking", "aligning"):
                finish("done", f"arrived — {dist_goal:.2f} m from the final point")
                return

            if phase == "acquiring":
                d_start = math.hypot(path.xy[0][0] - pos[0], path.xy[0][1] - pos[1])
                if d_start > START_RADIUS_M:
                    finish("fault",
                           f"rover is {d_start:.1f} m from the path start "
                           f"(limit {START_RADIUS_M:.0f} m) — move it to spot 1")
                    return
                phase = "aligning"
                align_origin = pos

            if phase == "aligning":
                if heading is not None:
                    phase = "tracking"
                    # Acquisition + alignment can legitimately take longer than
                    # the stall window; the clock starts with tracking itself.
                    last_progress = (path.cum[idx], now)
                else:
                    if align_origin is None:
                        align_origin = pos
                    crept = math.hypot(pos[0] - align_origin[0], pos[1] - align_origin[1])
                    if crept > ALIGN_MAX_DIST_M:
                        finish("fault", "could not establish heading while creeping "
                               f"{crept:.1f} m — check GPS course output")
                        return
                    # Straight, gentle creep until the receiver reports course.
                    omega_cmd = 0.0
                    self._target.set_drive(ALIGN_VX, 0.0, 0.0, source="follow")
                    publish("aligning", "creeping forward to establish heading")
                    time.sleep(period)
                    continue

            # --- tracking ---------------------------------------------------
            if now - last_progress[1] > PROGRESS_STALL_S:
                finish("fault", "no progress for "
                       f"{PROGRESS_STALL_S:.0f}s — stuck or blocked")
                return

            # Dead-reckon heading between GPS samples from the commanded yaw.
            # (+omega is CCW, which *decreases* a compass bearing.)
            heading -= math.degrees(omega_cmd * NOMINAL_YAW_RATE * dt)

            # Carrot: the path point LOOKAHEAD_M further along than where we are.
            carrot = idx
            target_s = path.cum[idx] + LOOKAHEAD_M
            while carrot < len(path.xy) - 1 and path.cum[carrot] < target_s:
                carrot += 1
            cx, cy = path.xy[carrot]
            desired = math.degrees(math.atan2(cx - pos[0], cy - pos[1]))
            err = _wrap_deg(desired - heading)

            # P steering: positive error = carrot is clockwise of our nose =
            # steer CW = negative omega.
            omega_cmd = max(-OMEGA_MAX, min(OMEGA_MAX, -err / YAW_FULL_ERR_DEG))
            if abs(err) >= PIVOT_ERR_DEG:
                vx = 0.0  # pivot in place until roughly pointed down the path
            else:
                vx = CRUISE_VX * (1.0 - abs(err) / 90.0)
            self._target.set_drive(vx, 0.0, omega_cmd, source="follow")
            publish("tracking", f"{path.cum[idx]:.1f}/{path.total_m:.1f} m")
            time.sleep(period)

        # stop() requested — _halt_locked zeros the target and sets state.
