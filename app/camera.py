"""Threaded camera wrapper for the FastAPI app.

Backed by the vendor-SDK binding at `app.ascam.Camera`. That underlying
binding already runs an SDK-owned thread delivering frames into a buffer;
this wrapper layers on:

- a small background thread that samples frame_id() so fps() stays current
  even if no consumer is actively polling,
- get_jpeg() — downscale + JPEG encode for the MJPEG endpoint,
- a context-manager surface.

Direct V4L2 capture via OpenCV was tried first and was empirically shown to
deliver banded garbage on the Nuwa60C (verified across four independent
test runs). The vendor C++ SDK is the only working path.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

from app.ascam import Camera as _AscamCamera


class Camera:
    def __init__(self) -> None:
        self._cam = _AscamCamera()
        self._lock = threading.Lock()
        self._last_frame_id = 0
        self._fps_window_start = time.monotonic()
        self._fps_window_count = 0
        self._current_fps = 0.0

        self._stop = threading.Event()
        self._fps_thread = threading.Thread(target=self._fps_loop, daemon=True)
        self._fps_thread.start()

    def _fps_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.1)
            fid = self._cam.frame_id()
            with self._lock:
                delta = fid - self._last_frame_id
                if delta > 0:
                    self._last_frame_id = fid
                    self._fps_window_count += delta
                now = time.monotonic()
                elapsed = now - self._fps_window_start
                if elapsed >= 1.0:
                    self._current_fps = self._fps_window_count / elapsed
                    self._fps_window_count = 0
                    self._fps_window_start = now

    def read(self) -> Optional[np.ndarray]:
        return self._cam.read()

    def fps(self) -> float:
        with self._lock:
            return self._current_fps

    def gain(self) -> float:
        return self._cam.gain

    def set_gain(self, value: float) -> None:
        self._cam.gain = value

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
