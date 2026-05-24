"""Camera backed by the Angstrong vendor SDK (Nuwa60C / HP60C model).

The actual lifecycle (SDK init, USB enumeration, RGB stream callback) lives
in the compiled C++ extension at app/_ascam_ext.<...>.so. This module is a
thin Python facade with software auto-exposure (AE) on top.

Why software AE
---------------
The HP60C SDK has no auto-exposure and no public hardware-gain API. With
a fixed software multiplier, dim scenes were too dark and bright scenes
blew out highlights (a 3× multiplier saturates anything raw ≥ 85).

This Camera measures each raw frame's mean BGR and adjusts a single gain
multiplier toward a target mean (default 100). An exponential moving
average smooths the gain so flicker / scene transitions don't cause
brightness oscillation. Converges in about 1 second at 10 fps.

Tunable via env vars:
    ROVER_CAM_TARGET_MEAN  desired BGR mean of the output frame (default 100)
                           lower = darker output, higher = brighter

TODO(hw-gain): hardware gain via the SDK's XuCmdCameraHp60c::setGain C++
symbol would be cleaner than software multiplication (no noise
amplification, no highlight clipping). Deferred — needs reverse
engineering of the SDK's internal handle layout. See git history.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np

from app._ascam_ext import Camera as _Camera

_REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_DIR = str(
    _REPO_ROOT / "Camera_Specs" / "ascam_ros2_ws" / "src" / "ascamera" / "configurationfiles"
)
DEFAULT_TARGET_MEAN = float(os.environ.get("ROVER_CAM_TARGET_MEAN", "100"))


class Camera:
    """Open the first attached vendor camera with software auto-exposure."""

    _GAIN_MIN = 0.3
    _GAIN_MAX = 12.0
    _GAIN_ALPHA = 0.2   # EMA factor: converges to a new scene in ~10 frames

    def __init__(
        self,
        config_dir: Optional[str] = None,
        timeout: float = 5.0,
        target_mean: float = DEFAULT_TARGET_MEAN,
    ) -> None:
        self._cam = _Camera(config_dir or DEFAULT_CONFIG_DIR, timeout)
        self._target_mean = max(1.0, min(254.0, float(target_mean)))
        self._gain = 3.0   # initial guess; AE will converge from here

    def read(self) -> Optional[np.ndarray]:
        """Latest BGR frame with AE-adjusted gain applied.

        Reads raw, updates the EMA gain toward (target / raw_mean), then
        applies it. Result is uint8 clipped to [0, 255].
        """
        raw = self._cam.read()
        if raw is None:
            return None

        raw_mean = float(raw.mean())
        if raw_mean > 1.0:
            desired = self._target_mean / raw_mean
            desired = max(self._GAIN_MIN, min(self._GAIN_MAX, desired))
            self._gain = (1.0 - self._GAIN_ALPHA) * self._gain + self._GAIN_ALPHA * desired

        if abs(self._gain - 1.0) < 1e-3:
            return raw
        return np.clip(raw.astype(np.float32) * self._gain, 0, 255).astype(np.uint8)

    def read_raw(self) -> Optional[np.ndarray]:
        """Latest BGR frame without AE — for diagnostics."""
        return self._cam.read()

    @property
    def gain(self) -> float:
        """Current AE-computed gain multiplier."""
        return self._gain

    @property
    def target_mean(self) -> float:
        return self._target_mean

    @target_mean.setter
    def target_mean(self, value: float) -> None:
        self._target_mean = max(1.0, min(254.0, float(value)))

    def width(self) -> int:
        return self._cam.width()

    def height(self) -> int:
        return self._cam.height()

    def frame_id(self) -> int:
        return self._cam.frame_id()

    def close(self) -> None:
        self._cam.close()

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
