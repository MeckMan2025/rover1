"""H.264 over fragmented-MP4 video streamer for the FastAPI app.

Architecture
------------
A single ffmpeg subprocess is shared across all WebSocket viewers. The
existing Camera class delivers BGR numpy frames; a feeder thread writes
them to ffmpeg's stdin. ffmpeg emits a fragmented MP4 stream on stdout
that a reader thread broadcasts to per-client asyncio queues. WebSocket
handlers drain their queues and push bytes to the browser, where
MediaSource Extensions appends each fragment to a <video> element.

Why subprocess ffmpeg
---------------------
- ffmpeg is one apt-get away; no Python dep to vendor (PyAV pulls in
  libav headers, GStreamer pulls in dozens of plugins).
- libx264 ultrafast + tune=zerolatency is the same encoder PyAV/aiortc
  would invoke under the hood, with the same CPU cost.
- Process isolation: an encoder hang doesn't take down FastAPI; we just
  restart the subprocess.

Init segment vs media fragments
-------------------------------
The first bytes ffmpeg emits are the ftyp+moov "initialization segment".
Every subsequent moof+mdat pair is a media fragment. Browsers using MSE
must append the init segment exactly once before any fragment. We parse
MP4 box headers to find where init ends (start of the first moof box),
cache that prefix, and replay it to every new client.

Slow client handling
--------------------
Each WebSocket viewer gets a bounded asyncio.Queue. The reader thread
posts each fragment to every queue via loop.call_soon_threadsafe. If a
queue is full (client is too slow to drain), we detach that client and
push a sentinel so its task closes the WS. The browser will reconnect,
get a fresh init segment and the next keyframe, and resync — better than
letting one bad client stall the encoder for everyone else.
"""

from __future__ import annotations

import asyncio
import logging
import subprocess
import threading
import time
from typing import Optional, Set, Tuple

import cv2

logger = logging.getLogger(__name__)


# Per-client queue depth. 60 fragments × 100 ms each = ~6 s of buffer
# headroom before we declare a client too slow and detach it.
CLIENT_QUEUE_DEPTH = 60

# Encoder defaults — picked to fit ≤ 2 Mbps at native camera resolution
# while keeping libx264 ultrafast comfortable on a Pi 5 core.
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 15
DEFAULT_BITRATE_KBPS = 1500

# Bounds for the runtime-reconfigurable encoder params (R5). The /api/video/config
# endpoint is unauthenticated, so any device on the LAN/Tailscale/LTE path can POST
# these; without bounds a zero/negative/huge width makes cv2.resize raise every
# feeder tick and ffmpeg fail to launch (-video_size 0x480), DoS-ing the stream.
MIN_DIM = 160
MAX_WIDTH = 1920
MAX_HEIGHT = 1080
MAX_FPS = 60
MAX_BITRATE_KBPS = 20000


def _clamp_even(value: int, lo: int, hi: int) -> int:
    """Clamp to [lo, hi] and force even — H.264 yuv420p requires even dimensions."""
    v = max(lo, min(hi, int(value)))
    return v - (v & 1)
DEFAULT_KEYINT_SECONDS = 2.0


class H264Streamer:
    """Single shared H.264/fMP4 encoder, fan-out to N WebSocket clients."""

    def __init__(
        self,
        camera,
        *,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: int = DEFAULT_FPS,
        bitrate_kbps: int = DEFAULT_BITRATE_KBPS,
        keyint_seconds: float = DEFAULT_KEYINT_SECONDS,
    ) -> None:
        self.camera = camera
        self.width = _clamp_even(width, MIN_DIM, MAX_WIDTH)
        self.height = _clamp_even(height, MIN_DIM, MAX_HEIGHT)
        self.fps = max(1, min(MAX_FPS, int(fps)))
        self.bitrate_kbps = max(64, min(MAX_BITRATE_KBPS, int(bitrate_kbps)))
        self.keyint_seconds = float(keyint_seconds)

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._proc: Optional[subprocess.Popen] = None
        self._feeder: Optional[threading.Thread] = None
        self._reader: Optional[threading.Thread] = None
        self._stderr_pump: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lifecycle_lock = asyncio.Lock()

        self._init_segment: bytes = b""
        self._init_lock = threading.Lock()

        # Clients are touched by both the asyncio loop (register/unregister)
        # and the reader thread (broadcast). Hold the lock when iterating.
        self._clients_lock = threading.Lock()
        self._clients: Set["asyncio.Queue[Optional[bytes]]"] = set()

    # --- introspection ------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def viewer_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def config(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "bitrate_kbps": self.bitrate_kbps,
            "keyint_seconds": self.keyint_seconds,
            "running": self.running,
            "viewers": self.viewer_count,
        }

    # --- lifecycle ----------------------------------------------------------

    async def start(self) -> None:
        async with self._lifecycle_lock:
            if self.running:
                return
            self._loop = asyncio.get_running_loop()
            self._stop.clear()
            cmd = self._build_ffmpeg_cmd()
            logger.info("h264: starting ffmpeg (%dx%d @ %dfps, %dkbps)",
                        self.width, self.height, self.fps, self.bitrate_kbps)
            try:
                self._proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
            except FileNotFoundError:
                logger.error("h264: ffmpeg binary not found — apt install ffmpeg on the rover")
                raise
            with self._init_lock:
                self._init_segment = b""
            self._feeder = threading.Thread(target=self._feed_loop, daemon=True, name="h264-feeder")
            self._reader = threading.Thread(target=self._read_loop, daemon=True, name="h264-reader")
            self._stderr_pump = threading.Thread(target=self._stderr_loop, daemon=True, name="h264-stderr")
            self._feeder.start()
            self._reader.start()
            self._stderr_pump.start()

    async def stop(self) -> None:
        async with self._lifecycle_lock:
            if self._proc is None:
                return
            logger.info("h264: stopping ffmpeg")
            self._stop.set()
            proc = self._proc
            try:
                if proc.stdin and not proc.stdin.closed:
                    proc.stdin.close()
            except Exception:
                pass
            try:
                proc.terminate()
            except ProcessLookupError:
                pass
            loop = asyncio.get_running_loop()
            try:
                await loop.run_in_executor(None, proc.wait, 2.0)
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                    await loop.run_in_executor(None, proc.wait, 1.0)
                except Exception:
                    pass
            for t in (self._feeder, self._reader, self._stderr_pump):
                if t is not None and t.is_alive():
                    t.join(timeout=1.0)
            self._proc = None
            self._broadcast_sentinel()

    async def reconfigure(
        self,
        *,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
        bitrate_kbps: Optional[int] = None,
    ) -> None:
        """Restart the encoder with new params. Viewers must re-init MSE."""
        # Clamp every runtime knob to safe bounds (R5) — this endpoint is
        # unauthenticated, so treat the input as hostile.
        if width is not None:        self.width = _clamp_even(width, MIN_DIM, MAX_WIDTH)
        if height is not None:       self.height = _clamp_even(height, MIN_DIM, MAX_HEIGHT)
        if fps is not None:          self.fps = max(1, min(MAX_FPS, int(fps)))
        if bitrate_kbps is not None: self.bitrate_kbps = max(64, min(MAX_BITRATE_KBPS, int(bitrate_kbps)))
        was_running = self.running
        await self.stop()
        if was_running:
            await self.start()

    # --- client registration ------------------------------------------------

    def register_client(self, queue: "asyncio.Queue[Optional[bytes]]") -> bytes:
        """Add a client queue; returns the cached init segment (may be empty)."""
        with self._clients_lock:
            self._clients.add(queue)
        with self._init_lock:
            return self._init_segment

    def unregister_client(self, queue: "asyncio.Queue[Optional[bytes]]") -> None:
        with self._clients_lock:
            self._clients.discard(queue)

    def current_init_segment(self) -> bytes:
        with self._init_lock:
            return self._init_segment

    # --- internals ----------------------------------------------------------

    def _build_ffmpeg_cmd(self) -> list:
        keyint = max(1, int(round(self.fps * self.keyint_seconds)))
        return [
            "ffmpeg", "-hide_banner", "-loglevel", "warning",
            # Input: raw BGR frames from stdin at our chosen size/rate.
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-video_size", f"{self.width}x{self.height}",
            "-framerate", str(self.fps),
            "-i", "pipe:0",
            # Encode H.264 — baseline profile so Safari on iOS handles it
            # without any tone-mapping fuss; ultrafast/zerolatency for the
            # lowest possible encoder lag.
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-profile:v", "baseline",
            "-pix_fmt", "yuv420p",
            "-b:v", f"{self.bitrate_kbps}k",
            "-maxrate", f"{self.bitrate_kbps}k",
            "-bufsize", f"{self.bitrate_kbps * 2}k",
            "-g", str(keyint),
            "-keyint_min", str(keyint),
            "-x264-params", "scenecut=0:rc-lookahead=0",
            # Output: fragmented MP4 to stdout. The flags below produce a
            # stream with an empty moov up front (the init segment) followed
            # by self-contained moof+mdat fragments — exactly what MSE needs.
            "-f", "mp4",
            "-movflags",
            "+frag_keyframe+empty_moov+default_base_moof+omit_tfhd_offset",
            "-frag_duration", "100000",  # microseconds → ~100 ms fragments
            "pipe:1",
        ]

    def _feed_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stdin is None:
            return
        stdin = proc.stdin
        period = 1.0 / self.fps
        target_size = (self.width, self.height)
        next_tick = time.monotonic()
        while not self._stop.is_set():
            try:
                frame = self.camera.read()
                if frame is not None:
                    if frame.shape[1] != target_size[0] or frame.shape[0] != target_size[1]:
                        frame = cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)
                    try:
                        stdin.write(frame.tobytes())
                    except (BrokenPipeError, ValueError, OSError):
                        # ffmpeg exited; reader will notice and we'll stop.
                        break
            except Exception as e:
                logger.warning("h264 feeder: %s", e)
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
        try:
            stdin.close()
        except Exception:
            pass

    def _read_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        stdout = proc.stdout
        buffer = b""
        init_captured = False
        while not self._stop.is_set():
            chunk = stdout.read(16384)
            if not chunk:
                break
            if not init_captured:
                buffer += chunk
                init, rest = _split_init_segment(buffer)
                if init is not None:
                    with self._init_lock:
                        self._init_segment = init
                    init_captured = True
                    buffer = b""
                    logger.info("h264: captured init segment (%d bytes)", len(init))
                    if rest:
                        self._broadcast(rest)
            else:
                self._broadcast(chunk)
        self._broadcast_sentinel()

    def _stderr_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stderr is None:
            return
        while not self._stop.is_set():
            line = proc.stderr.readline()
            if not line:
                break
            decoded = line.decode("utf-8", errors="replace").rstrip()
            if decoded:
                logger.info("ffmpeg: %s", decoded)

    def _broadcast(self, data: bytes) -> None:
        loop = self._loop
        if loop is None:
            return
        with self._clients_lock:
            queues = list(self._clients)
        for q in queues:
            try:
                loop.call_soon_threadsafe(_try_put, q, data, self._clients_lock, self._clients)
            except RuntimeError:
                # Event loop closed during shutdown.
                pass

    def _broadcast_sentinel(self) -> None:
        loop = self._loop
        if loop is None:
            return
        with self._clients_lock:
            queues = list(self._clients)
        for q in queues:
            try:
                loop.call_soon_threadsafe(_try_put, q, None, self._clients_lock, self._clients)
            except RuntimeError:
                pass


def _try_put(
    q: "asyncio.Queue[Optional[bytes]]",
    data: Optional[bytes],
    clients_lock: threading.Lock,
    clients: Set["asyncio.Queue[Optional[bytes]]"],
) -> None:
    """Put-or-detach. A slow client is removed so it can't backpressure the encoder."""
    try:
        q.put_nowait(data)
    except asyncio.QueueFull:
        with clients_lock:
            clients.discard(q)
        # Drain a slot so we can push the sentinel without raising again.
        try:
            q.get_nowait()
        except Exception:
            pass
        try:
            q.put_nowait(None)
        except Exception:
            pass


def _split_init_segment(buf: bytes) -> Tuple[Optional[bytes], bytes]:
    """Return (init_segment, remainder) once the first moof box is found.

    Walks MP4 box headers (4-byte size + 4-byte type) until it hits 'moof'.
    Returns (None, buf) while more bytes are needed.
    """
    idx = 0
    n = len(buf)
    while idx + 8 <= n:
        size = int.from_bytes(buf[idx:idx + 4], "big")
        box_type = buf[idx + 4:idx + 8]
        if box_type == b"moof":
            return buf[:idx], buf[idx:]
        # size == 1 means the real size is the next 8 bytes (64-bit).
        if size == 1:
            if idx + 16 > n:
                return None, buf
            size = int.from_bytes(buf[idx + 8:idx + 16], "big")
        # size == 0 means box extends to EOF — shouldn't happen in fMP4 init.
        if size < 8 or size > n - idx:
            return None, buf
        idx += size
    return None, buf
