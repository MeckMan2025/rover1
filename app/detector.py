"""COCO box follower for the simplify rover.

Hailo-8 YOLOv8s inference (COCO 80-class), filtered to a single chosen class
(dog or person), feeding a forward-only P-control loop that writes to the same
MotorTarget the joystick uses. The control law is class-agnostic: pick the
largest box of the target class, yaw to center it, and walk forward when it's
beyond the framing goal. No strafe. The one reverse case is edge-safety: if
the bbox top or bottom is clipped against the frame edge, the subject is too
tall to fit at the current distance and we back off gently until the full
body is in view. Bidirectional + strafe-aware (added briefly in May) overshot
in field test and was reverted to this simpler tune.

Architecture
------------
HailoYolo loads the .hef and holds persistent VStreams. detect(bgr_frame)
returns a list of (x1, y1, x2, y2, conf, class_id) in pixel coordinates.

BoxFollower owns a background thread that ticks at INFERENCE_HZ. Each tick:
  read latest frame → infer → pick largest box of target class → compute
  (vx, omega) → write (vx, 0, omega) to MotorTarget. When no target and
  DETECTION_TIMEOUT elapses, writes zeros. The target class is swappable
  at runtime via set_target_class() — used by the UI's DOG / PERSON follow
  buttons.

Power-scale + MAX_LINEAR/MAX_ANGULAR happen downstream (MotorTarget.get and the
motor thread), so the follower outputs normalized values in [-1, 1] just like
the joystick does.

Lazy init
---------
The Hailo SDK takes ~3 seconds and ~96 MB to load. We do that on the first
set_enabled(True) call, not at app startup, so the rover boots fast.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

import cv2
import numpy as np


# Default model path on the rover. Override with ROVER_HEF_PATH if you want
# the H8-native build (yolov8s_h8.hef) or a different file.
DEFAULT_HEF_PATH = os.environ.get(
    "ROVER_HEF_PATH",
    os.path.expanduser("~/ros2_ws/src/rover1/models/yolov8s.hef"),
)

DOG_CLASS_ID    = 16     # COCO class index for "dog"
PERSON_CLASS_ID = 0      # COCO class index for "person"
YOLO_INPUT      = 640    # YOLOv8 input side length

# Map of supported follow-mode labels → COCO class id. Anything else from
# the UI is rejected. Add new entries here to expose more classes.
TARGET_CLASSES: dict[str, int] = {
    "dog":    DOG_CLASS_ID,
    "person": PERSON_CLASS_ID,
}

# Control law constants. Forward-only P-control — no strafe, no reverse,
# no edge-safety. Yaw centers the target horizontally; forward speed scales
# with how far the target is from the framing goal (area). At or past
# TOO_CLOSE_RATIO, forward stops; the rover keeps tracking via yaw but
# doesn't approach. The more aggressive strafe + reverse + edge-safety
# extensions overshot and felt twitchy in field test, so we reverted to
# this responsiveness profile from rover1_vision/config/dog_follower.yaml —
# the first dog-follower tune that worked well.
CONFIDENCE_THRESHOLD = 0.5
TARGET_AREA_RATIO    = 0.15    # target fills ~15 % of frame at follow distance
TOO_CLOSE_RATIO      = 0.40    # target filling 40 %+ → stop forward, just track
AREA_DEADZONE        = 0.03    # don't twitch on small area changes
CENTER_TOLERANCE     = 0.05    # don't yaw inside ±5 % of center
YAW_GAIN             = 4.0     # multiplies normalized horizontal error
LINEAR_GAIN          = 10.0    # multiplies normalized distance error

# Edge-safety reverse — the one exception to "forward only". If the bbox
# top or bottom is at the frame edge, the subject is too tall to fit at
# this distance (head or feet would be clipped). Left/right edge clipping is
# a centering problem, not a distance problem — yaw handles that — so we don't
# react to it here.
#
# SAFETY (R4): the rover has NO rear camera or proximity sensor, so backing up
# is blind. Reverse is therefore OPT-IN (EDGE_REVERSE_ENABLED) — the default is
# stop-and-track. When enabled it is triply bounded: only while the subject is
# roughly centered (never while yawing toward someone off to the side), only up
# to EDGE_REVERSE_MAX_TIME of continuous motion per clip event, and only at a
# gentle fixed speed. Backing into a wall/curb/person behind is the failure it
# guards against.
EDGE_MARGIN_RATIO       = 0.05    # bbox edge within 5 % of frame top/bottom → clipped
EDGE_REVERSE_SPEED      = 0.20    # gentle constant reverse while clipped
EDGE_REVERSE_ENABLED    = False   # opt-in; default is stop-and-track (no rear sensing)
EDGE_REVERSE_CENTER_TOL = 0.10    # only reverse within ±10 % of center
EDGE_REVERSE_MAX_TIME   = 1.5     # max seconds of continuous reverse per clip event

DETECTION_TIMEOUT    = 1.0     # seconds; after this, declare "lost"
INFERENCE_HZ         = 10      # camera caps near this; no benefit going higher


class HailoYolo:
    """YOLOv8s on Hailo-8. Persistent VStreams, single-batch synchronous inference."""

    def __init__(self, hef_path: str = DEFAULT_HEF_PATH) -> None:
        # Lazy import — saves ~96 MB and doesn't fail at process boot if Hailo
        # isn't installed yet (which it now is, via pre-flight).
        from hailo_platform import (
            HEF,
            VDevice,
            HailoStreamInterface,
            InferVStreams,
            ConfigureParams,
            InputVStreamParams,
            OutputVStreamParams,
            FormatType,
        )

        if not os.path.exists(hef_path):
            raise FileNotFoundError(f"HEF model not found at {hef_path}")

        self._closed = False
        self.hef = HEF(hef_path)
        self.device = VDevice()

        configure_params = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = self.device.configure(self.hef, configure_params)[0]

        # Activate and keep the context alive for the lifetime of this object.
        self._activated = self.network_group.activate()
        self._activated.__enter__()

        self.input_info = self.hef.get_input_vstream_infos()[0]
        self.output_info = self.hef.get_output_vstream_infos()

        # Quantized uint8 in, float32 out (per the legacy code — Hailo expects
        # uint8 not normalized float for this .hef).
        input_params = InputVStreamParams.make_from_network_group(
            self.network_group, quantized=True, format_type=FormatType.UINT8
        )
        output_params = OutputVStreamParams.make_from_network_group(
            self.network_group, quantized=False, format_type=FormatType.FLOAT32
        )

        self._pipeline = InferVStreams(self.network_group, input_params, output_params)
        self._pipeline.__enter__()

    def detect(self, bgr_frame: np.ndarray) -> list[tuple]:
        """Run inference on a BGR frame, return [(x1, y1, x2, y2, conf, class_id), …]
        in original-frame pixel coordinates.
        """
        h, w = bgr_frame.shape[:2]

        # Preprocess: BGR→RGB resize to 640×640 uint8.
        resized = cv2.resize(bgr_frame, (YOLO_INPUT, YOLO_INPUT))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.uint8)
        input_data = np.expand_dims(rgb, axis=0)

        output_dict = self._pipeline.infer({self.input_info.name: input_data})
        return self._postprocess(output_dict, w, h)

    @staticmethod
    def _postprocess(outputs: dict, orig_w: int, orig_h: int) -> list[tuple]:
        """Parse Hailo's on-chip NMS output (nested-by-class list of detections).

        Output structure: outputs[name][batch][class_id] = [[ymin, xmin, ymax, xmax, score], …]
        Each class has a variable number of detections (or none), coords are normalized 0–1.
        Reused verbatim from rover1_vision/dog_follower.py.
        """
        detections: list[tuple] = []
        output_name = list(outputs.keys())[0]
        output = outputs[output_name]
        batch_output = output[0] if isinstance(output, list) else output[0]

        for class_id, class_detections in enumerate(batch_output):
            if class_detections is None or len(class_detections) == 0:
                continue
            if isinstance(class_detections, list):
                class_detections = np.array(class_detections)
            if len(class_detections.shape) == 1:
                class_detections = class_detections.reshape(1, -1)

            for det in class_detections:
                if len(det) < 5:
                    continue
                ymin, xmin, ymax, xmax, score = det[:5]
                if score < CONFIDENCE_THRESHOLD:
                    continue
                x1 = max(0, min(orig_w, int(xmin * orig_w)))
                y1 = max(0, min(orig_h, int(ymin * orig_h)))
                x2 = max(0, min(orig_w, int(xmax * orig_w)))
                y2 = max(0, min(orig_h, int(ymax * orig_h)))
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append((x1, y1, x2, y2, float(score), int(class_id)))

        return detections

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        # Walk back through the contexts we entered, in reverse. Each is wrapped
        # in try/except so cleanup keeps going even if a sub-step throws.
        for ctx_attr in ("_pipeline", "_activated"):
            ctx = getattr(self, ctx_attr, None)
            if ctx is None:
                continue
            try:
                ctx.__exit__(None, None, None)
            except Exception:
                pass
        try:
            self.device.release()
        except Exception:
            pass


class BoxFollower:
    """Closed-loop largest-box tracker for one COCO class at a time.

    Background thread reads frames from a Camera, filters detections to the
    active target class (dog or person), picks the biggest bbox, and writes
    drive commands to a MotorTarget. Mode is mutually exclusive — flipping
    from dog→person doesn't stop the loop, it just changes which class is
    matched on the next tick.
    """

    def __init__(self, camera, motor_target, *,
                 target_class_id: int = DOG_CLASS_ID,
                 target_label: str = "dog") -> None:
        self.camera = camera
        self.motor_target = motor_target

        self._target_class_id = target_class_id
        self._target_label = target_label

        self._enabled = False
        self._yolo: Optional[HailoYolo] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._reverse_since: Optional[float] = None  # start of current edge-reverse (R4 cap)

        # State exposed via status() — main.py merges this into /telemetry.
        self._lock = threading.Lock()
        self._status = "idle"
        self._target_seen = False
        self._target_bbox: Optional[tuple[int, int, int, int]] = None
        self._target_confidence: Optional[float] = None
        self._target_area_ratio: Optional[float] = None
        self._detector_fps = 0.0
        self._last_detection_time: Optional[float] = None

        # FPS rolling window
        self._fps_window_start = time.monotonic()
        self._fps_window_count = 0

    # --- Public API ---------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def target_label(self) -> str:
        return self._target_label

    def status(self) -> dict:
        """Class-neutral status payload. UI keys off `follow_target` to know
        which button to light up."""
        with self._lock:
            return {
                "follow_enabled": self._enabled,
                "follow_status": self._status,
                "follow_target": self._target_label if self._enabled else None,
                "target_label": self._target_label,
                "target_seen": self._target_seen,
                "target_bbox": list(self._target_bbox) if self._target_bbox else None,
                "target_confidence": self._target_confidence,
                "target_area_ratio": self._target_area_ratio,
                "detector_fps": round(self._detector_fps, 1),
            }

    def set_target_class(self, class_id: int, label: str) -> None:
        """Swap which COCO class the loop chases. Safe to call while running —
        the next inference tick reads the new id. Resets the visual state so
        the old class's bbox doesn't linger in the UI for a moment."""
        with self._lock:
            self._target_class_id = class_id
            self._target_label = label
            self._target_seen = False
            self._target_bbox = None
            self._target_confidence = None
            self._target_area_ratio = None
            self._last_detection_time = None
            if self._enabled:
                self._status = "searching"

    def set_enabled(self, enabled: bool) -> None:
        if enabled == self._enabled:
            return
        self._enabled = enabled
        if enabled:
            self._start()
        else:
            self._stop_drive()
            with self._lock:
                self._status = "idle"
                self._target_seen = False
                self._target_bbox = None

    def close(self) -> None:
        self._enabled = False
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._yolo is not None:
            self._yolo.close()
            self._yolo = None

    # --- Internal -----------------------------------------------------------

    def _start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        if self._yolo is None:
            with self._lock:
                self._status = "initializing"
            try:
                self._yolo = HailoYolo()
            except Exception as e:
                with self._lock:
                    self._status = f"init_failed: {e}"
                self._enabled = False
                return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        with self._lock:
            self._status = "searching"

        period = 1.0 / INFERENCE_HZ
        next_tick = time.monotonic()

        while not self._stop_event.is_set() and self._enabled:
            frame = self.camera.read()
            if frame is None:
                time.sleep(0.02)
                continue

            try:
                detections = self._yolo.detect(frame) if self._yolo else []
            except Exception:
                # Transient inference errors must not kill the loop.
                detections = []

            target = self._best_target(detections, frame.shape)
            self._update_state(target, frame.shape)

            if target is not None:
                vx, omega = self._compute_drive(target, frame.shape)
                vx = self._bounded_reverse(vx)
                # Forward-only: vy (strafe) is always zero. Normalized output —
                # MotorTarget.get applies power scale, motor thread applies
                # MAX_LINEAR/MAX_ANGULAR. Same path the joystick takes. source=
                # "follow" so MotorTarget yields to a recent manual command (R2).
                self.motor_target.set_drive(vx, 0.0, omega, source="follow")
            else:
                # No target this frame. Keep writing zero so the motor watchdog
                # stays fed (vs. going silent and triggering its 150 ms
                # zero-on-stale path mid-tick).
                self._reverse_since = None
                self.motor_target.set_drive(0.0, 0.0, 0.0, source="follow")

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()

        # Loop exited — make sure we leave motors stopped.
        self._stop_drive()

    def _best_target(self, detections: list, frame_shape: tuple) -> Optional[dict]:
        h, w = frame_shape[:2]
        target_id = self._target_class_id
        edge_margin_px = h * EDGE_MARGIN_RATIO
        candidates = []
        for d in detections:
            x1, y1, x2, y2, conf, class_id = d
            if class_id != target_id:
                continue
            area = (x2 - x1) * (y2 - y1)
            # Top or bottom of the bbox flush with the frame edge means the
            # subject is being clipped — head or feet not visible. Computed
            # once here so _compute_drive and _update_state agree on it.
            edge_clipped = (y1 < edge_margin_px) or (y2 > h - edge_margin_px)
            candidates.append({
                "bbox": (int(x1), int(y1), int(x2), int(y2)),
                "confidence": conf,
                "area": area,
                "center_x": (x1 + x2) / 2,
                "center_y": (y1 + y2) / 2,
                "area_ratio": area / (w * h),
                "edge_clipped": edge_clipped,
            })
        if not candidates:
            return None
        # Largest box = closest subject.
        return max(candidates, key=lambda d: d["area"])

    @staticmethod
    def _compute_drive(target: dict, frame_shape: tuple) -> tuple[float, float]:
        """Returns normalized (vx, omega) in [-1, 1]. No strafe.

        Yaw is P-control on horizontal centering with a deadzone. Negative
        because +center_err means the target is on the right and we need to
        yaw right (negative omega in ROS convention).

        Forward / stop / reverse:
          - bbox top or bottom clipped at frame edge → reverse ONLY if opt-in
            and the subject is centered (see R4 safety note); otherwise stop
            and keep tracking. The time budget is applied by the caller.
          - target too close by area (≥ TOO_CLOSE_RATIO) → stop forward
          - target far → forward proportional to area shortfall
        """
        h, w = frame_shape[:2]
        center_err = (target["center_x"] - w / 2) / w   # -0.5 .. +0.5
        area_ratio = target["area_ratio"]

        omega = 0.0
        if abs(center_err) > CENTER_TOLERANCE:
            omega = max(-1.0, min(1.0, -center_err * YAW_GAIN))

        vx = 0.0
        if target["edge_clipped"]:
            # Subject is taller than the frame at this distance — head or feet
            # are off-screen. Reverse is blind (no rear sensor), so only back up
            # when explicitly enabled AND the subject is centered; never while
            # yawing toward someone off to the side. Otherwise stop and track —
            # do NOT drive forward into an already-clipping subject.
            if EDGE_REVERSE_ENABLED and abs(center_err) <= EDGE_REVERSE_CENTER_TOL:
                vx = -EDGE_REVERSE_SPEED
        elif area_ratio <= TOO_CLOSE_RATIO:
            distance_err = TARGET_AREA_RATIO - area_ratio
            if distance_err > AREA_DEADZONE:
                vx = min(1.0, distance_err * LINEAR_GAIN)

        return vx, omega

    def _bounded_reverse(self, vx: float) -> float:
        """Cap continuous edge-reverse to EDGE_REVERSE_MAX_TIME per clip event.
        The rover backs up blind (no rear sensing), so it must not reverse
        indefinitely: if the subject stays clipped past the budget, stop and wait
        for them to step back — which clears edge_clipped, zeros vx, and resets
        the timer for the next event."""
        if vx >= 0.0:
            self._reverse_since = None
            return vx
        now = time.monotonic()
        if self._reverse_since is None:
            self._reverse_since = now
        if now - self._reverse_since > EDGE_REVERSE_MAX_TIME:
            return 0.0   # budget exhausted — hold position, keep tracking
        return vx

    def _update_state(self, target: Optional[dict], frame_shape: tuple) -> None:
        now = time.monotonic()
        self._fps_window_count += 1
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._detector_fps = self._fps_window_count / elapsed
            self._fps_window_count = 0
            self._fps_window_start = now

        with self._lock:
            if target is not None:
                self._target_seen = True
                self._target_bbox = target["bbox"]
                self._target_confidence = target["confidence"]
                self._target_area_ratio = target["area_ratio"]
                self._last_detection_time = now
                # "too_close" fires for both area overshoot AND edge clipping,
                # so the UI bbox turns red whenever the rover is backing off
                # or refusing to advance.
                if target["area_ratio"] > TOO_CLOSE_RATIO or target["edge_clipped"]:
                    self._status = "too_close"
                else:
                    self._status = "tracking"
            else:
                self._target_seen = False
                if self._last_detection_time is None or (now - self._last_detection_time) > DETECTION_TIMEOUT:
                    self._status = "searching"
                    self._target_bbox = None
                    self._target_confidence = None
                    self._target_area_ratio = None

    def _stop_drive(self) -> None:
        try:
            self.motor_target.set_drive(0.0, 0.0, 0.0)
        except Exception:
            pass
