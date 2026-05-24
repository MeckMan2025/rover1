"""FastAPI server: drive the rover from a browser with a live camera feed.

Architecture
------------
The asyncio loop only handles network I/O. Motor writes live on a dedicated
background thread that ticks at 50 Hz regardless of what the loop is doing.
MJPEG encoding runs in a thread executor so cv2.imencode can never starve
the WebSocket handler. Result: drive commands and motor updates are
independent of camera load, network jitter, or any one slow call.

Endpoints
---------
GET  /              static HTML UI
GET  /video.mjpg    multipart MJPEG live feed
GET  /telemetry     JSON: battery_v, camera_fps, camera_gain
WS   /ws            client → server:
                      {"type": "drive", "vx": -1..1, "vy": -1..1, "omega": -1..1}
                      {"type": "estop"}

Safety
------
- Motor target is zeroed if no drive command arrives within MOTOR_TIMEOUT
  (handled inside the motor thread, no separate watchdog needed).
- WebSocket disconnect immediately zeros the target.
- E-stop latches a zero target until the next drive command clears it.
- Joystick values are normalized [-1, 1] and scaled to MAX_LINEAR /
  MAX_ANGULAR before kinematics; the rover never gets "full speed" by
  accident.

Launch:
    uvicorn app.main:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import asyncio
import glob
import io
import logging
import os
import socket
import subprocess
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

import segno
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, StreamingResponse

from app.camera import Camera
from app.kinematics import cartesian_to_wheels
from app.motors import HiwonderHardware


def _primary_ip() -> str:
    """Return the IP a remote client would use to reach this host.

    Uses a UDP "connect" (no packets actually sent) so we get the kernel's
    choice of source address for outbound traffic — which on a Pi with
    Wi-Fi up is the Wi-Fi IP, exactly what a phone on the same LAN needs.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()

MAX_LINEAR = 0.5    # m/s when |joystick| == 1.0 (legacy stadia_teleop default)
MAX_ANGULAR = 1.0   # rad/s when |joystick| == 1.0 (legacy default; kinematics
                    # multiplies internally by ROTATION_SCALE=2.0)

MOTOR_HZ = 50.0     # motor-thread write rate; well above the hiwonder board's
                    # internal timeouts and the human reaction time
MOTOR_TIMEOUT = 0.15 # seconds before stale drive command -> zero target.
                     # Client sends at 20 Hz (50 ms), so 150 ms tolerates losing
                     # ~3 packets in a row before the motor stops on its own.
                     # Belt-and-suspenders backup for the JS-side explicit-zero
                     # send on joystick release.

WEB_DIR = Path(__file__).resolve().parent.parent / "web"


class MotorTarget:
    """Target velocity shared between the WebSocket handler and the motor thread.

    Also owns the power scale — a multiplier in [0.05, 1.0] that the UI's
    speed slider sends so the driver can throttle the rover for tight spaces
    without losing joystick range.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._last_update = 0.0
        self._estop = False
        self._power = 1.0  # slider scale, 0.05..1.0

    def set_drive(self, vx: float, vy: float, omega: float) -> None:
        with self._lock:
            self._vx, self._vy, self._omega = vx, vy, omega
            self._last_update = time.monotonic()
            self._estop = False

    def set_power(self, value: float) -> None:
        with self._lock:
            self._power = max(0.05, min(1.0, float(value)))

    def trigger_estop(self) -> None:
        with self._lock:
            self._estop = True
            self._vx = self._vy = self._omega = 0.0

    def get(self) -> tuple[float, float, float]:
        """Return (vx, vy, omega) — already power-scaled, zero if stale or e-stopped."""
        with self._lock:
            if self._estop:
                return (0.0, 0.0, 0.0)
            if time.monotonic() - self._last_update > MOTOR_TIMEOUT:
                return (0.0, 0.0, 0.0)
            p = self._power
            return (self._vx * p, self._vy * p, self._omega * p)

    @property
    def power(self) -> float:
        with self._lock:
            return self._power


class BatterySmoother:
    """Low-pass filter on the battery voltage so motor-load droops don't flicker the UI.

    With /telemetry polled at 2 Hz and alpha=0.15:
      ~58% of a step settled after 2.5 s, ~95% after ~7 s.
    Fast enough to register a real discharge, slow enough that sub-second
    current spikes don't push the indicator into amber and back.
    """

    def __init__(self, alpha: float = 0.15) -> None:
        self._alpha = alpha
        self._value: Optional[float] = None

    def update(self, raw: Optional[float]) -> Optional[float]:
        if raw is None:
            return self._value
        if self._value is None:
            self._value = raw
        else:
            self._value = (1.0 - self._alpha) * self._value + self._alpha * raw
        return self._value


def _wayland_socket() -> Optional[str]:
    """Find cage's active Wayland socket name (e.g. 'wayland-0').

    Hardcoding wayland-1 was wrong — cage opens at wayland-0 unless something
    else opens a Wayland session first, in which case cage gets bumped to
    wayland-1, wayland-2, etc. Looking up the socket at runtime is robust to
    whatever order things come up in.
    """
    uid = os.getuid()
    socks = sorted(glob.glob(f"/run/user/{uid}/wayland-[0-9]*"))
    if not socks:
        return None
    # Lowest-numbered first; cage typically gets it at boot.
    return os.path.basename(socks[0])


def _set_display(on: bool) -> bool:
    """Toggle the HDMI output via wlr-randr. Returns True on success.

    Requires XDG_RUNTIME_DIR to be set in the systemd unit so we can find
    /run/user/$UID; WAYLAND_DISPLAY is auto-detected at call time from the
    sockets in that directory. Output name HDMI-A-1 matches the rover's
    7" touchscreen.
    """
    arg = "--on" if on else "--off"
    socket_name = _wayland_socket()
    if socket_name is None:
        logger.warning("wlr-randr: no Wayland socket found in /run/user/%d", os.getuid())
        return False
    env = os.environ.copy()
    env["WAYLAND_DISPLAY"] = socket_name
    try:
        result = subprocess.run(
            ["wlr-randr", "--output", "HDMI-A-1", arg],
            env=env,
            capture_output=True,
            timeout=3.0,
        )
        if result.returncode != 0:
            logger.warning(
                "wlr-randr (%s) exit=%d stderr=%s",
                socket_name,
                result.returncode,
                result.stderr.decode("utf-8", errors="replace").strip(),
            )
            return False
        return True
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        logger.warning("wlr-randr failed: %s", e)
        return False


def _shutdown(action: str) -> bool:
    """Trigger a system poweroff or reboot via systemctl. Returns True on success.

    Requires passwordless sudo for systemctl (already true on this Pi for the
    user andrewmeckley — verified during earlier setup).
    """
    if action not in ("poweroff", "reboot"):
        return False
    try:
        result = subprocess.run(
            ["sudo", "systemctl", "--no-block", action],
            capture_output=True,
            timeout=3.0,
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


state: dict = {
    "camera": None,
    "hardware": None,
    "target": MotorTarget(),
    "motor_thread": None,
    "motor_stop": None,
    "battery_smoother": BatterySmoother(),
    # Bumped each time the rover screen comes back on. The kiosk polls this
    # in /telemetry and forces an MJPEG reconnect when it changes — chromium
    # doesn't auto-recover a stream that dropped while HDMI was off.
    "display_on_count": 0,
    # Count of remote (non-localhost) MJPEG viewers currently streaming.
    # When > 0 we encode at ~12 Hz; when 0 we drop to ~6 Hz — encoding for
    # nobody (or only the on-board kiosk) is the biggest CPU waste on the Pi.
    "mjpeg_viewers": 0,
}


def _motor_loop(hw: HiwonderHardware, target: MotorTarget, stop_event: threading.Event) -> None:
    """50 Hz: read target, compute wheel speeds, write I²C. Never touches asyncio."""
    period = 1.0 / MOTOR_HZ
    next_tick = time.monotonic()
    while not stop_event.is_set():
        try:
            vx, vy, omega = target.get()
            fl, fr, rl, rr = cartesian_to_wheels(
                vx * MAX_LINEAR, vy * MAX_LINEAR, omega * MAX_ANGULAR
            )
            hw.set_wheel_speeds(fl, fr, rl, rr)
        except Exception:
            # I²C hiccups must not kill the loop; the next tick will retry.
            pass

        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            # We fell behind; resync rather than spin.
            next_tick = time.monotonic()


@asynccontextmanager
async def lifespan(app: FastAPI):
    state["camera"] = Camera()
    state["hardware"] = HiwonderHardware()
    state["motor_stop"] = threading.Event()
    state["motor_thread"] = threading.Thread(
        target=_motor_loop,
        args=(state["hardware"], state["target"], state["motor_stop"]),
        daemon=True,
    )
    state["motor_thread"].start()
    try:
        yield
    finally:
        state["motor_stop"].set()
        state["motor_thread"].join(timeout=1.0)
        try:
            state["hardware"].close()
        finally:
            state["camera"].close()


app = FastAPI(lifespan=lifespan)


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.get("/qr")
async def qr_page() -> HTMLResponse:
    """Kiosk-side passive view: QR code linking to /, plus the live camera.

    The kiosk loads this so it's not a second WebSocket client fighting the
    phone for the motor target. Scan the QR → phone opens / → phone is the
    sole drive client.
    """
    url = f"http://{_primary_ip()}:8080/"
    html = (WEB_DIR / "qr.html").read_text().replace("__URL__", url)
    return HTMLResponse(html)


@app.get("/qr.svg")
async def qr_svg() -> Response:
    url = f"http://{_primary_ip()}:8080/"
    qr = segno.make(url, error="L")
    buf = io.BytesIO()
    qr.save(buf, kind="svg", scale=10, border=2, dark="#111")
    return Response(content=buf.getvalue(), media_type="image/svg+xml")


@app.get("/telemetry")
async def telemetry() -> JSONResponse:
    hw: Optional[HiwonderHardware] = state["hardware"]
    cam: Optional[Camera] = state["camera"]
    # Reading battery_v acquires the I²C lock too; quick (~1 ms) so fine inline.
    target: MotorTarget = state["target"]
    smoother: BatterySmoother = state["battery_smoother"]
    battery_raw = hw.read_battery_voltage() if hw else None
    battery_smoothed = smoother.update(battery_raw)
    return JSONResponse({
        # Smoothed value is what the UI colors the indicator from.
        "battery_v": battery_smoothed,
        # Raw is exposed too for diagnostics — useful for spotting load droop.
        "battery_v_raw": battery_raw,
        "camera_fps": cam.fps() if cam else 0.0,
        "camera_gain": cam.gain() if cam else None,
        "camera_target_mean": cam.target_mean() if cam else None,
        "power": target.power,
        # Increments each time the kiosk screen comes back on. The qr.html
        # poller watches this and forces an MJPEG reconnect on change.
        "display_on_count": state["display_on_count"],
        # Visible for verification that the adaptive framerate is doing its
        # thing — 0 means we're running the kiosk-only slow path.
        "mjpeg_viewers": state["mjpeg_viewers"],
    })


@app.get("/video.mjpg")
async def video_mjpg(request: Request) -> StreamingResponse:
    boundary = "frame"
    # Localhost = the on-board kiosk Chromium. Same SoC, so encoding for it
    # is a pure CPU tax. We still serve it (the kiosk needs preview frames)
    # but halve the rate when no real remote viewer is on the line.
    is_remote = bool(request.client) and request.client.host != "127.0.0.1"

    async def gen():
        if is_remote:
            state["mjpeg_viewers"] += 1
        loop = asyncio.get_event_loop()
        try:
            while True:
                cam: Optional[Camera] = state["camera"]
                # Offload the JPEG encode so it can't block the event loop —
                # otherwise drive commands queue up behind cv2.imencode.
                jpeg = await loop.run_in_executor(None, cam.get_jpeg) if cam else None
                if jpeg:
                    head = (
                        f"--{boundary}\r\n"
                        f"Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n\r\n"
                    ).encode()
                    yield head + jpeg + b"\r\n"
                # 12 Hz when a remote phone/iPad is watching, 6 Hz when only
                # the kiosk is. Halves CPU during idle / docked operation.
                await asyncio.sleep(0.08 if state["mjpeg_viewers"] > 0 else 0.16)
        finally:
            if is_remote:
                state["mjpeg_viewers"] -= 1

    return StreamingResponse(
        gen(), media_type=f"multipart/x-mixed-replace; boundary={boundary}"
    )


@app.websocket("/ws")
async def ws(websocket: WebSocket) -> None:
    await websocket.accept()
    target: MotorTarget = state["target"]
    loop = asyncio.get_event_loop()
    try:
        while True:
            msg = await websocket.receive_json()
            t = msg.get("type")
            if t == "drive":
                # No I²C, no kinematics — just store the normalized target.
                # The motor thread picks it up on its next 50 Hz tick.
                vx = float(msg.get("vx", 0.0))
                vy = float(msg.get("vy", 0.0))
                omega = float(msg.get("omega", 0.0))
                target.set_drive(vx, vy, omega)
            elif t == "set_power":
                target.set_power(float(msg.get("value", 1.0)))
            elif t == "set_display":
                # wlr-randr is a quick subprocess — off-thread it so the WS
                # loop stays responsive for the next drive command.
                on = bool(msg.get("on", True))
                ok = await loop.run_in_executor(None, _set_display, on)
                if ok and on:
                    # Signal the kiosk to reconnect its dead MJPEG stream.
                    state["display_on_count"] += 1
            elif t == "shutdown":
                # Acknowledged power actions from the UI. systemctl --no-block
                # returns immediately so the response goes back before the box dies.
                action = str(msg.get("action", ""))
                if action in ("poweroff", "reboot"):
                    await loop.run_in_executor(None, _shutdown, action)
            elif t == "estop":
                target.trigger_estop()
    except WebSocketDisconnect:
        target.set_drive(0.0, 0.0, 0.0)
