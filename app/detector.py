"""Dog detection + follower for the simplify rover.

Hailo-8 YOLOv8s inference (COCO 80-class, filtered to class 16 = dog), feeding a
forward-only P-control loop that writes to the same MotorTarget the joystick
uses. Lifted from rover1_vision/dog_follower.py (lines 738–937 are the core);
stripped of all ROS scaffolding — no Node, services, QoS, cv_bridge, or recovery
scan. Same Hailo init dance, same on-chip-NMS postprocess, same tuned constants.

Architecture
------------
HailoYolo loads the .hef and holds persistent VStreams. detect(bgr_frame)
returns a list of (x1, y1, x2, y2, conf, class_id) in pixel coordinates.

DogFollower owns a background thread that ticks at INFERENCE_HZ. Each tick:
  read latest frame → infer → pick largest dog → compute (vx, omega) → write
  to MotorTarget. When no dog and DETECTION_TIMEOUT elapses, writes zeros.

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

DOG_CLASS_ID = 16        # COCO class index for "dog"
YOLO_INPUT = 640         # YOLOv8 input side length

# Control law constants. Copied from rover1_vision/config/dog_follower.yaml —
# these are the values that worked in legacy production.
CONFIDENCE_THRESHOLD = 0.5
TARGET_AREA_RATIO    = 0.15    # dog should fill ~15 % of frame at follow distance
TOO_CLOSE_RATIO      = 0.40    # dog filling 40 %+ of frame → stop forward, just track
AREA_DEADZONE        = 0.03    # don't twitch on small area changes
CENTER_TOLERANCE     = 0.05    # don't yaw inside ±5 % of center
YAW_GAIN             = 4.0     # multiplies normalized horizontal error
LINEAR_GAIN          = 10.0    # multiplies normalized distance error
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


class DogFollower:
    """Closed-loop dog tracker. Background thread reads frames from a Camera
    and writes drive commands to a MotorTarget. Forward-only — no reverse,
    no strafe in v1 (legacy didn't strafe either).
    """

    def __init__(self, camera, motor_target) -> None:
        self.camera = camera
        self.motor_target = motor_target

        self._enabled = False
        self._yolo: Optional[HailoYolo] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # State exposed via status() — main.py merges this into /telemetry.
        self._lock = threading.Lock()
        self._status = "idle"
        self._dog_seen = False
        self._dog_bbox: Optional[tuple[int, int, int, int]] = None
        self._dog_confidence: Optional[float] = None
        self._dog_area_ratio: Optional[float] = None
        self._detector_fps = 0.0
        self._last_detection_time: Optional[float] = None

        # FPS rolling window
        self._fps_window_start = time.monotonic()
        self._fps_window_count = 0

    # --- Public API ---------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    def status(self) -> dict:
        with self._lock:
            return {
                "follow_enabled": self._enabled,
                "follow_status": self._status,
                "dog_seen": self._dog_seen,
                "dog_bbox": list(self._dog_bbox) if self._dog_bbox else None,
                "dog_confidence": self._dog_confidence,
                "dog_area_ratio": self._dog_area_ratio,
                "detector_fps": round(self._detector_fps, 1),
            }

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
                self._dog_seen = False
                self._dog_bbox = None

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

            dog = self._best_dog(detections, frame.shape)
            self._update_state(dog, frame.shape)

            if dog is not None:
                vx, omega = self._compute_drive(dog, frame.shape)
                # Normalized output — MotorTarget.get applies power scale,
                # motor thread applies MAX_LINEAR/MAX_ANGULAR. Same path the
                # joystick takes.
                self.motor_target.set_drive(vx, 0.0, omega)
            else:
                # No dog this frame. Keep writing zero so the motor watchdog
                # stays fed (vs. going silent and triggering its 150 ms
                # zero-on-stale path mid-tick).
                self.motor_target.set_drive(0.0, 0.0, 0.0)

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()

        # Loop exited — make sure we leave motors stopped.
        self._stop_drive()

    @staticmethod
    def _best_dog(detections: list, frame_shape: tuple) -> Optional[dict]:
        h, w = frame_shape[:2]
        dogs = []
        for d in detections:
            x1, y1, x2, y2, conf, class_id = d
            if class_id != DOG_CLASS_ID:
                continue
            area = (x2 - x1) * (y2 - y1)
            dogs.append({
                "bbox": (int(x1), int(y1), int(x2), int(y2)),
                "confidence": conf,
                "area": area,
                "center_x": (x1 + x2) / 2,
                "area_ratio": area / (w * h),
            })
        if not dogs:
            return None
        # Largest box = closest dog
        return max(dogs, key=lambda d: d["area"])

    @staticmethod
    def _compute_drive(dog: dict, frame_shape: tuple) -> tuple[float, float]:
        """Returns normalized (vx, omega) in [-1, 1].

        Yaw is P-control on horizontal centering with a deadzone. Negative
        because +center_err means the dog is on the right and we need to yaw
        right (negative omega in ROS convention).

        Forward is forward-only and gated by bbox area: dog too close → stop
        forward; dog far → advance proportionally to area shortfall.
        """
        h, w = frame_shape[:2]
        center_err = (dog["center_x"] - w / 2) / w   # -0.5 .. +0.5
        area_ratio = dog["area_ratio"]

        omega = 0.0
        if abs(center_err) > CENTER_TOLERANCE:
            omega = max(-1.0, min(1.0, -center_err * YAW_GAIN))

        vx = 0.0
        if area_ratio <= TOO_CLOSE_RATIO:
            distance_err = TARGET_AREA_RATIO - area_ratio
            if distance_err > AREA_DEADZONE:
                vx = min(1.0, distance_err * LINEAR_GAIN)

        return vx, omega

    def _update_state(self, dog: Optional[dict], frame_shape: tuple) -> None:
        now = time.monotonic()
        self._fps_window_count += 1
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._detector_fps = self._fps_window_count / elapsed
            self._fps_window_count = 0
            self._fps_window_start = now

        with self._lock:
            if dog is not None:
                self._dog_seen = True
                self._dog_bbox = dog["bbox"]
                self._dog_confidence = dog["confidence"]
                self._dog_area_ratio = dog["area_ratio"]
                self._last_detection_time = now
                self._status = "too_close" if dog["area_ratio"] > TOO_CLOSE_RATIO else "tracking"
            else:
                self._dog_seen = False
                if self._last_detection_time is None or (now - self._last_detection_time) > DETECTION_TIMEOUT:
                    self._status = "searching"
                    self._dog_bbox = None
                    self._dog_confidence = None
                    self._dog_area_ratio = None

    def _stop_drive(self) -> None:
        try:
            self.motor_target.set_drive(0.0, 0.0, 0.0)
        except Exception:
            pass
