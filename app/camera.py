"""Threaded camera capture from /dev/video0 via cv2/v4l2.

A single background thread continuously reads frames; consumers (MJPEG
endpoint, dog detector) pull the latest BGR frame without blocking each
other.

The Nuwa60C only delivers YUYV @ 1280×1040 ~10 fps reliably on this Pi —
MJPG modes are advertised but stall (confirmed during prelim testing). We
capture at native resolution and downscale at JPEG-encode time so phone
bandwidth stays low.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np


class Camera:
    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 1280,
        height: int = 1040,
    ) -> None:
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera at {device}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None

        self._fps_window_start = time.monotonic()
        self._fps_window_frames = 0
        self._current_fps = 0.0

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            with self._lock:
                self._latest = frame
                self._fps_window_frames += 1
                now = time.monotonic()
                elapsed = now - self._fps_window_start
                if elapsed >= 1.0:
                    self._current_fps = self._fps_window_frames / elapsed
                    self._fps_window_frames = 0
                    self._fps_window_start = now

    def read(self) -> Optional[np.ndarray]:
        """Return a copy of the latest BGR frame, or None if no frame yet."""
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def fps(self) -> float:
        with self._lock:
            return self._current_fps

    def get_jpeg(
        self,
        target_width: int = 640,
        target_height: int = 520,
        quality: int = 70,
    ) -> Optional[bytes]:
        """Return a downscaled, JPEG-encoded frame ready for an MJPEG stream."""
        frame = self.read()
        if frame is None:
            return None
        if frame.shape[1] != target_width or frame.shape[0] != target_height:
            frame = cv2.resize(
                frame, (target_width, target_height), interpolation=cv2.INTER_AREA
            )
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return jpeg.tobytes() if ok else None

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._cap.release()

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
