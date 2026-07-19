"""FastAPI server: drive the rover from a browser with a live camera feed.

Architecture
------------
The asyncio loop only handles network I/O. Motor writes live on a dedicated
background thread that ticks at 50 Hz regardless of what the loop is doing.
MJPEG encoding runs in a thread executor so cv2.imencode can never starve
the WebSocket handler. The H.264 path delegates to app.streamer.H264Streamer,
which owns the ffmpeg subprocess and fans out fragments to all WS viewers.
Result: drive commands and motor updates are independent of camera load,
network jitter, or any one slow call.

Endpoints
---------
GET  /                    static HTML UI
GET  /video.mjpg          multipart MJPEG live feed (legacy / fallback)
GET  /api/snapshot        single low-res JPEG (Cellular-mode escape hatch)
WS   /ws/video            fragmented MP4 H.264 stream for MediaSource
GET  /api/video/config    current encoder config
POST /api/video/config    {"width","height","fps","bitrate_kbps"} → reconfigure
GET  /telemetry           JSON: battery_v, camera_fps, camera_gain, ...
WS   /ws                  client → server:
                            {"type": "drive", "vx": -1..1, "vy": -1..1, "omega": -1..1}
                            {"type": "estop"}

Safety
------
- Motor target is zeroed if no drive command arrives within MOTOR_TIMEOUT
  (handled inside the motor thread, no separate watchdog needed).
- The /ws handler zeros the target on every exit path (disconnect, error,
  shutdown) via finally; per-message errors are dropped without killing
  the socket.
- E-stop is a sticky latch: all drive commands are ignored until an
  explicit estop_clear message releases it (R1).
- Drive values are validated server-side in MotorTarget.set_drive —
  non-finite rejected, clamped to [-1, 1] — then scaled to MAX_LINEAR /
  MAX_ANGULAR before kinematics; the rover never gets "full speed" by
  accident, even from a non-browser client.

Launch:
    uvicorn app.main:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import asyncio
import glob
import io
import json
import logging
import math
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
from app.detector import BoxFollower, TARGET_CLASSES
from app.kinematics import cartesian_to_wheels
from app.motors import HiwonderHardware
from app.streamer import H264Streamer


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
MANUAL_HOLD = 0.5    # seconds a non-zero manual command holds priority over the
                     # follow loop. The client streams at 20 Hz, so continuous
                     # steering keeps refreshing the window; on release the
                     # follower resumes ~0.5 s later. (R2 manual override.)

WEB_DIR = Path(__file__).resolve().parent.parent / "web"

# Written by scripts/ntrip_rtk.py (rover-ntrip.service) every ~2 s. A stale
# mtime means the NTRIP client (and therefore GPS reporting) is down.
GPS_STATUS_PATH = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "rover-gps-status.json"
GPS_STATUS_MAX_AGE = 10.0


def _gps_status() -> dict:
    """GPS/RTK state for /telemetry: fix label, sat count, accuracy, caster."""
    try:
        if time.time() - GPS_STATUS_PATH.stat().st_mtime > GPS_STATUS_MAX_AGE:
            return {"gps_fix": "offline", "gps_sats": 0,
                    "gps_hacc_m": None, "gps_caster": False}
        s = json.loads(GPS_STATUS_PATH.read_text())
        return {"gps_fix": s.get("fix", "offline"),
                "gps_sats": s.get("sats", 0),
                "gps_hacc_m": s.get("hacc_m"),
                "gps_caster": bool(s.get("caster"))}
    except (OSError, ValueError):
        return {"gps_fix": "offline", "gps_sats": 0,
                "gps_hacc_m": None, "gps_caster": False}


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
        self._manual_hold_until = 0.0  # monotonic time until which manual wins

    def set_drive(self, vx: float, vy: float, omega: float,
                  source: str = "manual") -> None:
        """Store the target velocity. `source` is "manual" (joystick/UI) or
        "follow" (the BoxFollower control loop). Manual has precedence: a non-zero
        manual command opens a MANUAL_HOLD window during which follow writes are
        dropped, so the operator can steer without first toggling follow off.

        Values are validated here — not in the /ws handler — so every writer is
        covered. The socket is unauthenticated and json.loads accepts Infinity/
        NaN literals; an unchecked non-finite value survives to int() inside
        cartesian_to_wheels, raises there every motor tick (swallowed by the
        loop's I²C-hiccup except), and freezes the wheels at their last speed
        while the fresh _last_update keeps the stale-target watchdog fed."""
        vx, vy, omega = float(vx), float(vy), float(omega)
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(omega)):
            return
        clamp = lambda v: max(-1.0, min(1.0, v))
        vx, vy, omega = clamp(vx), clamp(vy), clamp(omega)
        with self._lock:
            # E-STOP is a sticky latch: while set, ignore every drive command —
            # the manual joystick stream *and* the follow loop — so nothing can
            # re-arm motion until an explicit clear_estop(). This is the fix for
            # the safety bug where set_drive cleared the latch, letting the 10 Hz
            # follower resume pursuit ~100 ms after E-STOP.
            if self._estop:
                return
            now = time.monotonic()
            if source == "follow":
                # Yield to a recent manual command (R2). The follower runs at
                # 10 Hz; without this it would stomp manual input every ~100 ms.
                if now < self._manual_hold_until:
                    return
            elif vx or vy or omega:
                # Any non-zero manual command (re)opens the priority window.
                self._manual_hold_until = now + MANUAL_HOLD
            self._vx, self._vy, self._omega = vx, vy, omega
            self._last_update = now

    def set_power(self, value: float) -> None:
        with self._lock:
            self._power = max(0.05, min(1.0, float(value)))

    def trigger_estop(self) -> None:
        with self._lock:
            self._estop = True
            self._vx = self._vy = self._omega = 0.0

    def clear_estop(self) -> None:
        """Release the E-STOP latch. The pre-estop target is discarded and marked
        stale, so a fresh drive command is required before the rover moves again —
        releasing the stop cannot lurch the rover on a held/queued command."""
        with self._lock:
            self._estop = False
            self._vx = self._vy = self._omega = 0.0
            self._last_update = 0.0

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estop

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


def _cpu_temp_c() -> Optional[float]:
    """Read SoC temperature in °C from the kernel's thermal zone.

    Cheap (sysfs read, no subprocess) — fine to call on every /telemetry hit.
    Returns None on Pis that don't expose this path; UI hides the value then.
    """
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return float(f.read().strip()) / 1000.0
    except (FileNotFoundError, ValueError, OSError):
        return None


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
    # BoxFollower instance — created in lifespan, owns the Hailo pipeline and
    # the detection/control thread. None when follow mode has never been
    # enabled (Hailo init is lazy on first toggle). Target class (dog vs
    # person) is set per-toggle from the UI via /ws.
    "follower": None,
    # H264Streamer — created in lifespan, encoder starts lazily on the first
    # /ws/video connection and stops after a grace period when the last
    # viewer leaves. None means the stream has never been requested.
    "streamer": None,
}

# Seconds to wait after the last /ws/video viewer disconnects before stopping
# the encoder. Avoids spin-up cost when a viewer briefly drops and reconnects.
H264_IDLE_GRACE = 10.0


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
    # Follower is constructed but NOT started — Hailo init happens on first
    # set_enabled(True) so the app boots fast. Default target is dog; UI
    # may flip it to person via set_follow_mode.
    state["follower"] = BoxFollower(
        camera=state["camera"],
        motor_target=state["target"],
    )
    # Streamer is constructed but its ffmpeg subprocess is NOT started — the
    # encoder spins up on the first /ws/video connect.
    state["streamer"] = H264Streamer(camera=state["camera"])
    try:
        yield
    finally:
        try:
            if state["streamer"] is not None:
                await state["streamer"].stop()
        finally:
            try:
                if state["follower"] is not None:
                    state["follower"].close()
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
    follower: Optional[BoxFollower] = state["follower"]
    battery_raw = hw.read_battery_voltage() if hw else None
    battery_smoothed = smoother.update(battery_raw)
    follow_fields = follower.status() if follower else {
        "follow_enabled": False, "follow_status": "idle",
        "follow_target": None, "target_label": "dog",
        "target_seen": False, "target_bbox": None,
        "target_confidence": None, "target_area_ratio": None,
        "detector_fps": 0.0,
    }
    return JSONResponse({
        # Smoothed value is what the UI colors the indicator from.
        "battery_v": battery_smoothed,
        # Raw is exposed too for diagnostics — useful for spotting load droop.
        "battery_v_raw": battery_raw,
        "camera_fps": cam.fps() if cam else 0.0,
        "camera_gain": cam.gain() if cam else None,
        "camera_target_mean": cam.target_mean() if cam else None,
        # Stall-watchdog recoveries since boot — nonzero means the camera
        # dropped off USB at some point and was reopened automatically.
        "camera_reopens": cam.reopens() if cam else 0,
        "power": target.power,
        # Increments each time the kiosk screen comes back on. The qr.html
        # poller watches this and forces an MJPEG reconnect on change.
        "display_on_count": state["display_on_count"],
        # Visible for verification that the adaptive framerate is doing its
        # thing — 0 means we're running the kiosk-only slow path.
        "mjpeg_viewers": state["mjpeg_viewers"],
        # SoC temperature — surfaces thermal headroom (or lack of it) in the UI.
        "cpu_temp_c": _cpu_temp_c(),
        **_gps_status(),
        **follow_fields,
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


@app.get("/api/snapshot")
async def api_snapshot() -> Response:
    """One-shot low-res JPEG for Cellular mode — never streams.

    480x360 (4:3 downscale of the 640x480 native) at q=60 lands around
    40-80 KB per call, which is the whole point: a phone on LTE can pull
    a fresh frame on demand without burning the SIM's data plan.
    """
    cam: Optional[Camera] = state["camera"]
    loop = asyncio.get_event_loop()
    jpeg = await loop.run_in_executor(None, cam.get_jpeg, 480, 360, 60) if cam else None
    if not jpeg:
        return Response(status_code=503)
    return Response(content=jpeg, media_type="image/jpeg",
                    headers={"Cache-Control": "no-store"})


async def _h264_idle_shutdown(streamer: H264Streamer) -> None:
    """Stop the encoder if no viewers remain after a grace period.

    Multiple disconnects schedule multiple checks; the second one is a no-op
    because streamer.stop() returns early when the subprocess is already gone.
    """
    await asyncio.sleep(H264_IDLE_GRACE)
    if streamer.viewer_count == 0:
        try:
            await streamer.stop()
        except Exception as e:
            logger.warning("h264 idle stop failed: %s", e)


@app.websocket("/ws/video")
async def ws_video(websocket: WebSocket) -> None:
    """Push fragmented MP4 chunks to a MediaSource consumer.

    Wire protocol: binary WS frames. First message is the init segment
    (ftyp+moov); every subsequent message is one or more moof+mdat
    fragments concatenated. The browser appends them to a SourceBuffer
    of type 'video/mp4; codecs="avc1.42E01E"'.
    """
    await websocket.accept()
    streamer: Optional[H264Streamer] = state["streamer"]
    if streamer is None:
        await websocket.close(code=1011)
        return
    # Lazy start — first connection brings ffmpeg up.
    if not streamer.running:
        try:
            await streamer.start()
        except FileNotFoundError:
            # ffmpeg not installed — tell the client cleanly so it can fall
            # back to MJPEG instead of retrying forever.
            await websocket.close(code=1011, reason="ffmpeg not installed")
            return
        except Exception as e:
            logger.error("h264 start failed: %s", e)
            await websocket.close(code=1011)
            return

    queue: "asyncio.Queue[Optional[bytes]]" = asyncio.Queue(maxsize=60)
    init = streamer.register_client(queue)
    try:
        # Wait briefly for the init segment if we beat the encoder to it.
        wait_deadline = time.monotonic() + 3.0
        while not init and time.monotonic() < wait_deadline:
            await asyncio.sleep(0.05)
            init = streamer.current_init_segment()
        if not init:
            logger.warning("h264: no init segment after 3s; closing client")
            await websocket.close(code=1011)
            return
        await websocket.send_bytes(init)

        while True:
            chunk = await queue.get()
            if chunk is None:
                # Sentinel: encoder stopped or we got detached as a slow client.
                break
            await websocket.send_bytes(chunk)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        logger.warning("ws_video send error: %s", e)
    finally:
        streamer.unregister_client(queue)
        if streamer.viewer_count == 0:
            asyncio.create_task(_h264_idle_shutdown(streamer))


@app.get("/api/video/config")
async def get_video_config() -> JSONResponse:
    streamer: Optional[H264Streamer] = state["streamer"]
    if streamer is None:
        return JSONResponse({"error": "streamer not initialized"}, status_code=503)
    return JSONResponse(streamer.config())


@app.post("/api/video/config")
async def set_video_config(payload: dict) -> JSONResponse:
    """Change encoder bitrate/resolution/fps at runtime.

    Reconfigure restarts the ffmpeg subprocess, which forces every connected
    viewer to re-init MSE on its next chunk (sentinel triggers reconnect on
    the client). Tradeoff is brief (~500 ms) interruption per knob turn.
    """
    streamer: Optional[H264Streamer] = state["streamer"]
    if streamer is None:
        return JSONResponse({"error": "streamer not initialized"}, status_code=503)
    try:
        # Values are clamped to safe bounds inside the streamer (R5); non-numeric
        # input raises here and is a client error, not a server fault.
        await streamer.reconfigure(
            width=payload.get("width"),
            height=payload.get("height"),
            fps=payload.get("fps"),
            bitrate_kbps=payload.get("bitrate_kbps"),
        )
    except (ValueError, TypeError) as e:
        return JSONResponse({"error": f"invalid config: {e}"}, status_code=400)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    return JSONResponse(streamer.config())


@app.websocket("/ws")
async def ws(websocket: WebSocket) -> None:
    await websocket.accept()
    target: MotorTarget = state["target"]
    loop = asyncio.get_event_loop()

    async def dispatch(msg: dict) -> None:
        t = msg.get("type")
        if t == "drive":
            # No I²C, no kinematics — just store the normalized target.
            # The motor thread picks it up on its next 50 Hz tick.
            # (Range/finiteness validation lives in MotorTarget.set_drive.)
            vx = float(msg.get("vx", 0.0))
            vy = float(msg.get("vy", 0.0))
            omega = float(msg.get("omega", 0.0))
            # While follow mode is on, ignore idle joystick heartbeats so
            # they don't count as manual input. A non-zero command instead
            # opens the MANUAL_HOLD window (see MotorTarget.set_drive), which
            # makes the follow loop yield — instant manual override (R2).
            follower: Optional[BoxFollower] = state["follower"]
            if (follower is not None and follower.enabled
                    and vx == 0.0 and vy == 0.0 and omega == 0.0):
                return
            target.set_drive(vx, vy, omega, source="manual")
        elif t == "set_follow_mode":
            # Wire format: {"type":"set_follow_mode", "mode": "dog"|"person"|null}
            # null (or missing) disables. dog/person sets the target class
            # and enables — they're mutually exclusive by construction.
            mode = msg.get("mode")
            follower = state["follower"]
            if follower is None:
                return
            if mode in TARGET_CLASSES:
                follower.set_target_class(TARGET_CLASSES[mode], mode)
                # Hailo init can take ~3 s on first enable — off-thread it
                # so the WS loop stays responsive.
                if not follower.enabled:
                    await loop.run_in_executor(None, follower.set_enabled, True)
            else:
                # Any other value (None, "off", garbage) = stop following.
                if follower.enabled:
                    await loop.run_in_executor(None, follower.set_enabled, False)
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
            # Disable the follower too. The sticky latch already blocks its
            # set_drive writes, but leaving it enabled means it would resume
            # the instant the operator clears the stop. Off-thread because
            # set_enabled(False) stops AND joins the control-loop thread.
            follower = state["follower"]
            if follower is not None and follower.enabled:
                await loop.run_in_executor(None, follower.set_enabled, False)
        elif t == "estop_clear":
            target.clear_estop()

    try:
        while True:
            try:
                msg = await websocket.receive_json()
            except ValueError:
                # Non-JSON frame from a flaky client — drop the frame, keep
                # the control socket alive.
                logger.warning("ws: ignoring non-JSON frame")
                continue
            try:
                await dispatch(msg)
            except (AttributeError, KeyError, TypeError, ValueError) as e:
                # One garbled message (non-dict payload, non-numeric field —
                # note JS serializes Infinity to null, so float(None) lands
                # here) must not tear down the whole drive channel.
                logger.warning("ws: ignoring malformed message %r: %s", msg, e)
    except WebSocketDisconnect:
        pass
    finally:
        # Zero the target on EVERY exit path — clean disconnect, an unexpected
        # fault, or server shutdown — not just WebSocketDisconnect. This is the
        # first layer; the 150 ms stale-target watchdog is the backstop.
        target.set_drive(0.0, 0.0, 0.0)
