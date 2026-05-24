"""Camera backed by the Angstrong vendor SDK (Nuwa60C / HP60C model).

The actual lifecycle (SDK init, USB enumeration, RGB stream callback) lives
in the compiled C++ extension at app/_ascam_ext.<...>.so. This module is a
thin Python facade for ergonomics: default config-dir path, context-manager
support, and software gain.

Why software gain
-----------------
The HP60C SDK ships with no auto-exposure and no C-style API to set hardware
gain. A 60-second dark-room warmup showed zero AE convergence (mean BGR
stuck at 0.001). The encrypted JSON config bakes gain at a low value
suitable for outdoor light only; indoor rooms render near-black.

Hardware gain methods do exist as exported C++ class symbols
(`XuCmdCameraHp60c::setGain(int)`, etc.) but reaching them requires
reverse-engineering the SDK's internal handle layout. Deferred — see TODO
below if/when YOLOv8 noise tolerance becomes a problem.

Until then, software gain (multiply BGR in numpy after read) gives a usable
image for both phone-feed and detection. Tune with the `gain` constructor
arg or the ROVER_CAM_GAIN env var.

TODO(hw-gain): call XuCmdCameraHp60c::setGain directly. Symbol exists at
_ZN16XuCmdCameraHp60c7setGainEi in libAngstrongCameraSdk.so but the
instance pointer is created inside AS_SDK_OpenCamera and not exposed by any
public API. Would require declaring a stub class and finding the storage
keyed off AS_CAM_PTR.
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
DEFAULT_GAIN = float(os.environ.get("ROVER_CAM_GAIN", "6.0"))


class Camera:
    """Open the first attached vendor camera and stream RGB frames."""

    def __init__(
        self,
        config_dir: Optional[str] = None,
        timeout: float = 5.0,
        gain: float = DEFAULT_GAIN,
    ) -> None:
        self._cam = _Camera(config_dir or DEFAULT_CONFIG_DIR, timeout)
        self._gain = float(gain)

    def read(self) -> Optional[np.ndarray]:
        """Latest BGR frame as a NumPy uint8 array of shape (H, W, 3), gain applied."""
        frame = self._cam.read()
        if frame is None:
            return None
        if self._gain == 1.0:
            return frame
        return np.clip(frame.astype(np.float32) * self._gain, 0, 255).astype(np.uint8)

    def read_raw(self) -> Optional[np.ndarray]:
        """Latest BGR frame WITHOUT software gain applied."""
        return self._cam.read()

    @property
    def gain(self) -> float:
        return self._gain

    @gain.setter
    def gain(self, value: float) -> None:
        self._gain = max(0.1, float(value))

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
