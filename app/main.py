"""FastAPI server: drive the rover from a browser with a live camera feed.

Endpoints
---------
GET  /              static HTML UI (phone-friendly)
GET  /video.mjpg    multipart MJPEG live feed
GET  /telemetry     JSON snapshot: battery voltage, camera fps
WS   /ws            drive commands from the client:
                      {"type": "drive", "vx": -1..1, "vy": -1..1, "omega": -1..1}
                      {"type": "estop"}

Safety
------
- Motors stop if no drive command arrives within CMD_TIMEOUT seconds (watchdog).
- WebSocket disconnect immediately stops motors.
- Joystick values are normalized [-1, 1] and scaled to MAX_LINEAR / MAX_ANGULAR
  before kinematics; the rover never receives "go full speed" by accident.

Launch:
    uvicorn app.main:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse

from app.camera import Camera
from app.kinematics import cartesian_to_wheels
from app.motors import HiwonderHardware

MAX_LINEAR = 0.4    # m/s when |joystick| == 1.0
MAX_ANGULAR = 1.5   # rad/s when |joystick| == 1.0
CMD_TIMEOUT = 0.5   # motor watchdog window

WEB_DIR = Path(__file__).resolve().parent.parent / "web"

state: dict = {
    "camera": None,
    "hardware": None,
    "last_cmd_time": 0.0,
}


async def _watchdog() -> None:
    while True:
        hw: Optional[HiwonderHardware] = state["hardware"]
        if hw is not None and time.monotonic() - state["last_cmd_time"] > CMD_TIMEOUT:
            hw.stop()
        await asyncio.sleep(0.1)


@asynccontextmanager
async def lifespan(app: FastAPI):
    state["camera"] = Camera()
    state["hardware"] = HiwonderHardware()
    state["last_cmd_time"] = time.monotonic()
    watchdog_task = asyncio.create_task(_watchdog())
    try:
        yield
    finally:
        watchdog_task.cancel()
        try:
            state["hardware"].close()
        finally:
            state["camera"].close()


app = FastAPI(lifespan=lifespan)


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.get("/telemetry")
async def telemetry() -> JSONResponse:
    hw: Optional[HiwonderHardware] = state["hardware"]
    cam: Optional[Camera] = state["camera"]
    return JSONResponse({
        "battery_v": hw.read_battery_voltage() if hw else None,
        "camera_fps": cam.fps() if cam else 0.0,
        "camera_gain": cam.gain() if cam else None,
    })


@app.get("/video.mjpg")
async def video_mjpg() -> StreamingResponse:
    boundary = "frame"

    async def gen():
        while True:
            cam: Optional[Camera] = state["camera"]
            jpeg = cam.get_jpeg() if cam else None
            if jpeg:
                head = (
                    f"--{boundary}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n\r\n"
                ).encode()
                yield head + jpeg + b"\r\n"
            await asyncio.sleep(0.08)  # ~12 Hz, capped by camera's ~10 fps

    return StreamingResponse(
        gen(), media_type=f"multipart/x-mixed-replace; boundary={boundary}"
    )


@app.websocket("/ws")
async def ws(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            msg = await websocket.receive_json()
            t = msg.get("type")
            hw: Optional[HiwonderHardware] = state["hardware"]
            if hw is None:
                continue
            if t == "drive":
                vx = float(msg.get("vx", 0.0)) * MAX_LINEAR
                vy = float(msg.get("vy", 0.0)) * MAX_LINEAR
                omega = float(msg.get("omega", 0.0)) * MAX_ANGULAR
                fl, fr, rl, rr = cartesian_to_wheels(vx, vy, omega)
                hw.set_wheel_speeds(fl, fr, rl, rr)
                state["last_cmd_time"] = time.monotonic()
            elif t == "estop":
                hw.stop()
                state["last_cmd_time"] = 0.0
    except WebSocketDisconnect:
        hw = state["hardware"]
        if hw is not None:
            hw.stop()
