"""Threaded camera wrapper for the FastAPI app.

Backed by the vendor-SDK binding at `app.ascam.Camera`. That underlying
binding already runs an SDK-owned thread delivering frames into a buffer;
this wrapper layers on:

- a small background thread that samples frame_id() so fps() stays current
  even if no consumer is actively polling,
- a stall watchdog on that same thread: if frame_id() stops advancing the
  SDK has lost the device (observed 2026-07-19: the HP60C dropped off the
  USB bus and re-enumerated; the SDK kept a dead handle and delivered
  frames never again). The watchdog closes and reopens the camera so every
  consumer of this wrapper recovers without a service restart,
- get_jpeg() — downscale + JPEG encode for the MJPEG endpoint,
- a context-manager surface.

Direct V4L2 capture via OpenCV was tried first and was empirically shown to
deliver banded garbage on the Nuwa60C (verified across four independent
test runs). The vendor C++ SDK is the only working path.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import cv2
import numpy as np

from app.ascam import Camera as _AscamCamera

logger = logging.getLogger(__name__)

# frame_id() flat for this long → the SDK handle is dead; reopen it. The
# camera delivers 10-15 fps normally, so 4 s of silence is unambiguous.
STALL_REOPEN_SECONDS = 4.0
# Wait this long between reopen attempts while the device is still absent
# (e.g. mid re-enumeration, or genuinely unplugged).
REOPEN_RETRY_SECONDS = 5.0


class Camera:
    def __init__(self) -> None:
        self._cam = _AscamCamera()
        self._lock = threading.Lock()
        self._last_frame_id = 0
        self._fps_window_start = time.monotonic()
        self._fps_window_count = 0
        self._current_fps = 0.0
        self._last_advance = time.monotonic()
        self._reopen_count = 0

        self._stop = threading.Event()
        self._fps_thread = threading.Thread(target=self._fps_loop, daemon=True)
        self._fps_thread.start()

    def _fps_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.1)
            try:
                fid = self._cam.frame_id()
            except Exception:
                # Handle mid-swap or already dead; treat as no advance.
                fid = None
            now = time.monotonic()
            with self._lock:
                if fid is not None:
                    delta = fid - self._last_frame_id
                    if delta > 0:
                        self._last_frame_id = fid
                        self._fps_window_count += delta
                        self._last_advance = now
                    elif delta < 0:
                        # Fresh handle after a reopen restarts ids from zero.
                        self._last_frame_id = fid
                elapsed = now - self._fps_window_start
                if elapsed >= 1.0:
                    self._current_fps = self._fps_window_count / elapsed
                    self._fps_window_count = 0
                    self._fps_window_start = now
                stalled = (now - self._last_advance) > STALL_REOPEN_SECONDS
            if stalled:
                self._reopen()

    def _reopen(self) -> None:
        """Close the dead SDK handle and open a fresh one.

        Runs on the watchdog thread. Consumers keep calling read() on this
        wrapper throughout — read() tolerates the swap and simply returns
        None until the new handle delivers.
        """
        self._reopen_count += 1
        logger.warning(
            "camera: no frames for %.0fs — reopening (attempt #%d)",
            STALL_REOPEN_SECONDS, self._reopen_count,
        )
        old = self._cam
        target_mean = None
        try:
            target_mean = old.target_mean
        except Exception:
            pass
        try:
            old.close()
        except Exception as e:
            logger.warning("camera: close of stale handle failed: %s", e)
        try:
            new_cam = _AscamCamera()
            if target_mean is not None:
                new_cam.target_mean = target_mean
        except Exception as e:
            logger.error("camera: reopen failed (%s); retrying in %.0fs",
                         e, REOPEN_RETRY_SECONDS)
            with self._lock:
                # Push the stall deadline out so the loop retries on the
                # configured cadence instead of every 100 ms tick.
                self._last_advance = time.monotonic() + REOPEN_RETRY_SECONDS - STALL_REOPEN_SECONDS
            return
        self._cam = new_cam
        with self._lock:
            self._last_frame_id = 0
            self._last_advance = time.monotonic()
        logger.info("camera: reopened successfully")

    def read(self) -> Optional[np.ndarray]:
        try:
            return self._cam.read()
        except Exception:
            # Mid-reopen or dead handle — a None frame is the contract for
            # "nothing available", and every consumer already handles it.
            return None

    def fps(self) -> float:
        with self._lock:
            return self._current_fps

    def reopens(self) -> int:
        """How many times the stall watchdog has recycled the SDK handle."""
        return self._reopen_count

    def gain(self) -> Optional[float]:
        """Current AE-computed software gain multiplier."""
        try:
            return self._cam.gain
        except Exception:
            return None

    def target_mean(self) -> Optional[float]:
        """Target BGR mean the AE loop converges toward."""
        try:
            return self._cam.target_mean
        except Exception:
            return None

    def set_target_mean(self, value: float) -> None:
        self._cam.target_mean = value

    def get_jpeg(
        self,
        target_width: Optional[int] = None,
        target_height: Optional[int] = None,
        quality: int = 70,
    ) -> Optional[bytes]:
        """Return a JPEG-encoded frame. Resizes only if target dims are given."""
        frame = self.read()
        if frame is None:
            return None
        if (
            target_width is not None
            and target_height is not None
            and (frame.shape[1] != target_width or frame.shape[0] != target_height)
        ):
            frame = cv2.resize(
                frame, (target_width, target_height), interpolation=cv2.INTER_AREA
            )
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return jpeg.tobytes() if ok else None

    def close(self) -> None:
        self._stop.set()
        self._fps_thread.join(timeout=2.0)
        self._cam.close()

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
