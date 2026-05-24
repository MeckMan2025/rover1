#!/usr/bin/env python3
"""Vendor-SDK camera bench test.

Opens the camera through the pybind11 wrapper, captures 30 frames, and saves
the 15th to /tmp/ascam_test.jpg. The success criterion is rover-Claude
RENDERING that JPEG (via the Read tool) and confirming it depicts a real
scene — channel-mean math passes on banded garbage, so it is NOT enough.

Run from the repo root with the venv active:
    python tests/bench_camera.py
"""

from __future__ import annotations

import os
import sys
import time

# Make `app` importable when run directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from app.ascam import Camera  # noqa: E402

NUM_FRAMES = 30
KEEP_INDEX = 15
OUTPUT = "/tmp/ascam_test.jpg"


def main() -> None:
    print("Opening camera...")
    with Camera() as cam:
        print(f"Camera open. Reported dimensions: {cam.width()} x {cam.height()}")

        # Wait for the first frame to arrive (callback-driven, so it may be late).
        t0 = time.monotonic()
        while cam.frame_id() == 0:
            if time.monotonic() - t0 > 3.0:
                print("Timed out waiting for first frame.")
                return
            time.sleep(0.05)

        captured = []
        last_id = -1
        while len(captured) < NUM_FRAMES:
            fid = cam.frame_id()
            if fid != last_id:
                frame = cam.read()
                if frame is not None:
                    captured.append(frame)
                    last_id = fid
            else:
                time.sleep(0.005)

        print(f"Captured {len(captured)} frames.")
        for i, f in enumerate(captured[:5] + captured[-2:]):
            print(f"  frame {i:2d}: shape={f.shape} dtype={f.dtype} "
                  f"mean_bgr=({f[:,:,0].mean():.0f}, {f[:,:,1].mean():.0f}, {f[:,:,2].mean():.0f})")

        # Save the KEEP_INDEX-th frame as JPEG for visual verification.
        target = captured[KEEP_INDEX]
        ok = cv2.imwrite(OUTPUT, target)
        if not ok:
            print(f"cv2.imwrite returned False for {OUTPUT}")
            return
        size = os.path.getsize(OUTPUT)
        print(f"Wrote {OUTPUT} ({size} bytes).")
        print(f"  This frame: shape={target.shape}  dtype={target.dtype}")
        print(f"  mean_bgr=({target[:,:,0].mean():.1f}, "
              f"{target[:,:,1].mean():.1f}, {target[:,:,2].mean():.1f})")
        print()
        print("VERIFICATION REQUIRED: render the JPEG before claiming success.")


if __name__ == "__main__":
    main()
