"""Camera backed by the Angstrong vendor SDK (Nuwa60C / HP60C model).

The actual lifecycle (SDK init, USB enumeration, RGB stream callback) lives
in the compiled C++ extension at app/_ascam_ext.<...>.so. This module is a
thin Python facade for ergonomics: default config-dir path and context-manager
support.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from app._ascam_ext import Camera as _Camera

_REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_DIR = str(
    _REPO_ROOT / "Camera_Specs" / "ascam_ros2_ws" / "src" / "ascamera" / "configurationfiles"
)


class Camera:
    """Open the first attached vendor camera and stream RGB frames."""

    def __init__(self, config_dir: Optional[str] = None, timeout: float = 5.0) -> None:
        self._cam = _Camera(config_dir or DEFAULT_CONFIG_DIR, timeout)

    def read(self) -> Optional[np.ndarray]:
        """Latest BGR frame as a NumPy uint8 array of shape (H, W, 3), or None."""
        return self._cam.read()

    def width(self) -> int:
        return self._cam.width()

    def height(self) -> int:
        return self._cam.height()

    def frame_id(self) -> int:
        """Monotonic counter from the SDK callback — increments on every new frame."""
        return self._cam.frame_id()

    def close(self) -> None:
        self._cam.close()

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
