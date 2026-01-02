#!/usr/bin/env python3
"""
Dog Follower Node - Uses Hailo-8L for YOLOv8 dog detection and follows detected dogs.

This node uses the Hailo-8L AI accelerator to run real-time YOLOv8 inference,
detecting dogs (COCO class 16) and generating velocity commands to follow them.

TWO-STATE DESIGN:
  - Detection: Controls whether Hailo inference runs and annotated images are published
  - Following: Controls whether the rover moves to follow detected dogs (requires detection on)

SAFETY FEATURES:
  - Both detection and following disabled by default
  - Auto-disables following if teleop sends /cmd_vel (teleop override)
  - Stops if no dog detected for configurable timeout
  - Velocity limits enforced in hardware

Subscribes:
  - /ascamera_hp60c/camera_publisher/rgb0/image (sensor_msgs/Image)
  - /cmd_vel (geometry_msgs/Twist) - for teleop override detection

Publishes:
  - /cmd_vel (geometry_msgs/Twist) - when following enabled
  - /dog_follower/status (std_msgs/String) - status updates
  - /dog_follower/annotated_image (sensor_msgs/Image) - when detection enabled

Services:
  - /dog_follower/detection_enable (std_srvs/Trigger) - turn on detection/visualization
  - /dog_follower/detection_disable (std_srvs/Trigger) - turn off detection (fails if following)
  - /dog_follower/enable (std_srvs/Trigger) - start following (auto-enables detection)
  - /dog_follower/disable (std_srvs/Trigger) - stop following (keeps detection on)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading

# Hailo SDK imports - gracefully handle if not installed
try:
    from hailo_platform import (
        HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams,
        InputVStreamParams, OutputVStreamParams, FormatType
    )
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False


class DogFollower(Node):
    """ROS 2 node for dog detection and following using Hailo-8L."""

    # COCO class IDs
    DOG_CLASS_ID = 16

    # YOLOv8 input size
    YOLO_INPUT_SIZE = 640

    # Colors for bounding box visualization (BGR format)
    COLOR_DOG_TARGET = (0, 255, 0)      # Bright green - active target being followed
    COLOR_DOG_DETECTED = (0, 200, 0)    # Darker green - detected but not target
    COLOR_STATUS_ACTIVE = (0, 255, 0)   # Green - detection/following active
    COLOR_STATUS_IDLE = (128, 128, 128) # Gray - idle

    def __init__(self):
        super().__init__('dog_follower')

        # Callback groups for concurrent handling
        self.service_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.bridge = CvBridge()
        self.detection_enabled = False  # Controls Hailo inference and annotated image
        self.following_enabled = False  # Controls whether rover follows (requires detection)
        self.last_detection_time = None
        self.current_status = 'idle'
        self.current_target_dog = None  # Track which dog we're following for visualization

        # Bounding box persistence (prevents flicker during brief detection drops)
        self.last_dog_bbox = None
        self.last_dog_bbox_time = None
        self.bbox_persist_duration = 1.0  # Match the 1s following timeout

        # Tracking for predictive recovery when dog walks past
        self.last_dog_frame_position = None  # 'left', 'center', 'right'
        self.last_dog_was_close = False  # Was dog in "too close" zone before losing detection
        self.recovery_rotation_direction = 0.0  # Which way to rotate when scanning
        self.recovery_scan_start_time = None  # When we started recovery scan

        # Thread safety for cmd_vel detection
        self.cmd_vel_lock = threading.Lock()
        self.last_external_cmd_time = None
        self.our_last_publish_time = None

        # Declare parameters
        self.declare_parameter('model_path', os.path.expanduser(
            '~/ros2_ws/src/rover1/models/yolov8s.hef'))
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('target_box_area_ratio', 0.15)  # Target: dog fills 15% of frame
        self.declare_parameter('linear_speed', 1.0)  # m/s - aggressive for fast dogs
        self.declare_parameter('angular_speed', 2.0)  # rad/s - aggressive for fast dogs
        self.declare_parameter('center_tolerance', 0.1)  # 10% of frame width deadzone
        self.declare_parameter('detection_timeout', 1.0)  # Stop if no dog for 1s
        self.declare_parameter('teleop_override_timeout', 0.5)  # Disable if teleop active
        self.declare_parameter('too_close_area_ratio', 0.40)  # Stop approaching when dog fills 40% of frame
        self.declare_parameter('recovery_scan_timeout', 3.0)  # Give up recovery scan after 3s

        # Load parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.target_area_ratio = self.get_parameter('target_box_area_ratio').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.teleop_override_timeout = self.get_parameter('teleop_override_timeout').value
        self.too_close_area_ratio = self.get_parameter('too_close_area_ratio').value
        self.recovery_scan_timeout = self.get_parameter('recovery_scan_timeout').value

        # Initialize Hailo inference engine
        self.hailo_device = None
        self.hailo_hef = None
        self.hailo_network_group = None
        self.activated_network_group = None
        self.input_vstream_info = None
        self.output_vstream_info = None
        self.input_vstream_params = None
        self.output_vstream_params = None
        self.infer_pipeline = None  # Persistent inference pipeline
        self._init_hailo()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/dog_follower/status', 10)
        self.annotated_image_pub = self.create_publisher(
            Image, '/dog_follower/annotated_image', qos_profile_sensor_data
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Subscribe to cmd_vel to detect teleop override
        # Use BEST_EFFORT to not miss any messages
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            cmd_vel_qos
        )

        # Services - Detection control
        self.detection_enable_srv = self.create_service(
            Trigger, '/dog_follower/detection_enable',
            self.detection_enable_callback,
            callback_group=self.service_cb_group
        )
        self.detection_disable_srv = self.create_service(
            Trigger, '/dog_follower/detection_disable',
            self.detection_disable_callback,
            callback_group=self.service_cb_group
        )

        # Services - Following control
        self.enable_srv = self.create_service(
            Trigger, '/dog_follower/enable',
            self.enable_callback,
            callback_group=self.service_cb_group
        )
        self.disable_srv = self.create_service(
            Trigger, '/dog_follower/disable',
            self.disable_callback,
            callback_group=self.service_cb_group
        )

        # Safety timer - checks for lost target and teleop override
        self.safety_timer = self.create_timer(
            0.1, self.safety_check,
            callback_group=self.timer_cb_group
        )

        # Status publishing timer
        self.status_timer = self.create_timer(
            0.5, self.publish_status_timer,
            callback_group=self.timer_cb_group
        )

        self.get_logger().info('Dog Follower node initialized')
        self.get_logger().info(f'Model path: {self.model_path}')
        self.get_logger().info(f'Hailo SDK available: {HAILO_AVAILABLE}')
        if not HAILO_AVAILABLE:
            self.get_logger().warn('Running in MOCK MODE - no real inference')
        self.publish_status('idle')

    def _init_hailo(self):
        """Initialize Hailo-8L inference engine with persistent vstreams."""
        if not HAILO_AVAILABLE:
            self.get_logger().warn('Hailo SDK not available - running in mock mode')
            return

        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model file not found: {self.model_path}')
            self.get_logger().warn('Running in mock mode until model is available')
            return

        try:
            # Load HEF (Hailo Executable Format)
            self.hailo_hef = HEF(self.model_path)

            # Create virtual device (auto-detects Hailo-8L at /dev/hailo0)
            self.hailo_device = VDevice()

            # Configure network group
            configure_params = ConfigureParams.create_from_hef(
                self.hailo_hef, interface=HailoStreamInterface.PCIe
            )
            self.hailo_network_group = self.hailo_device.configure(
                self.hailo_hef, configure_params
            )[0]

            # Enter activation context and keep it open for node lifetime
            self.activated_network_group = self.hailo_network_group.activate()
            self.activated_network_group.__enter__()

            # Get input/output stream info
            self.input_vstream_info = self.hailo_hef.get_input_vstream_infos()[0]
            self.output_vstream_info = self.hailo_hef.get_output_vstream_infos()

            # Create persistent vstream params
            self.input_vstream_params = InputVStreamParams.make_from_network_group(
                self.hailo_network_group, quantized=True, format_type=FormatType.UINT8
            )
            self.output_vstream_params = OutputVStreamParams.make_from_network_group(
                self.hailo_network_group, quantized=False, format_type=FormatType.FLOAT32
            )

            # Create persistent inference pipeline and enter its context
            self.infer_pipeline = InferVStreams(
                self.hailo_network_group,
                self.input_vstream_params,
                self.output_vstream_params
            )
            self.infer_pipeline.__enter__()

            self.get_logger().info('Hailo model loaded and vstreams activated successfully')
            self.get_logger().info(f'Input shape: {self.input_vstream_info.shape}')
            self.get_logger().info(f'Output layers: {len(self.output_vstream_info)}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Hailo: {e}')
            self.get_logger().warn('Running in mock mode')
            self.hailo_device = None

    def detection_enable_callback(self, request, response):
        """Enable detection (Hailo inference + annotated image publishing)."""
        was_already_enabled = self.detection_enabled

        # Always reset state - this fixes stuck 'teleop_override' status
        self.detection_enabled = True
        self.current_status = 'detecting'
        self.current_target_dog = None
        self.publish_status('detecting')

        if was_already_enabled:
            response.success = True
            response.message = 'Detection re-activated'
            self.get_logger().info('Detection re-activated (was already enabled)')
        else:
            response.success = True
            response.message = 'Detection enabled - Hailo inference active'
            self.get_logger().info('Detection ENABLED')

        return response

    def detection_disable_callback(self, request, response):
        """Disable detection (stops Hailo inference)."""
        if self.following_enabled:
            response.success = False
            response.message = 'Cannot disable detection while following is active. Stop following first.'
            return response

        self.detection_enabled = False
        self.current_status = 'idle'
        self.current_target_dog = None

        response.success = True
        response.message = 'Detection disabled'
        self.get_logger().info('Detection DISABLED')
        self.publish_status('idle')
        return response

    def enable_callback(self, request, response):
        """Enable dog following (auto-enables detection if needed)."""
        if self.following_enabled:
            response.success = False
            response.message = 'Dog follower already enabled'
            return response

        # Auto-enable detection if not already enabled
        if not self.detection_enabled:
            self.detection_enabled = True
            self.get_logger().info('Detection auto-enabled for following')

        self.following_enabled = True
        self.last_detection_time = self.get_clock().now()
        self.current_status = 'searching'

        response.success = True
        response.message = 'Dog follower enabled - searching for dog'
        self.get_logger().info('Dog follower ENABLED')
        self.publish_status('searching')
        return response

    def disable_callback(self, request, response):
        """Disable dog following (keeps detection on for visualization)."""
        was_enabled = self.following_enabled
        self.following_enabled = False
        self.stop_robot()
        self.current_target_dog = None

        # Keep detection on so user can still see bounding boxes
        if self.detection_enabled:
            self.current_status = 'detecting'
            self.publish_status('detecting')
        else:
            self.current_status = 'idle'
            self.publish_status('idle')

        response.success = True
        if was_enabled:
            response.message = 'Dog follower disabled (detection still active)'
            self.get_logger().info('Dog follower DISABLED')
        else:
            response.message = 'Dog follower was not enabled'

        return response

    def cmd_vel_callback(self, msg: Twist):
        """
        Detect external cmd_vel messages (teleop override).

        If we receive a cmd_vel that we didn't publish, and dog follower is enabled,
        this indicates teleop is trying to take control - we should auto-disable.
        """
        now = self.get_clock().now()

        with self.cmd_vel_lock:
            # Check if this is our own message (published within last 50ms)
            if self.our_last_publish_time is not None:
                elapsed = (now - self.our_last_publish_time).nanoseconds / 1e9
                if elapsed < 0.05:  # 50ms window
                    return  # This is our message, ignore

            # This is an external message
            is_nonzero = (
                abs(msg.linear.x) > 0.01 or
                abs(msg.linear.y) > 0.01 or
                abs(msg.angular.z) > 0.01
            )

            if is_nonzero:
                self.last_external_cmd_time = now

    def image_callback(self, msg: Image):
        """Process camera images - run detection when enabled, follow when enabled."""
        # If detection is off, do nothing (dashboard uses raw camera feed)
        if not self.detection_enabled:
            return

        try:
            # Convert ROS image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Run inference (only when detection is enabled)
        detections = self.run_inference(cv_image)

        # Filter to dogs only for visualization
        dog_detections = [d for d in detections if int(d[5]) == self.DOG_CLASS_ID]

        # Find best dog (for following and target highlight)
        dog = self.find_best_dog(detections, cv_image.shape)
        self.current_target_dog = dog  # Store for visualization

        # Update cached bbox if we have a detection
        if dog is not None:
            self.last_dog_bbox = dog['bbox']
            self.last_dog_bbox_time = self.get_clock().now()

            # Track dog's position in frame for recovery prediction
            frame_third = cv_image.shape[1] / 3
            if dog['center_x'] < frame_third:
                self.last_dog_frame_position = 'left'
            elif dog['center_x'] > frame_third * 2:
                self.last_dog_frame_position = 'right'
            else:
                self.last_dog_frame_position = 'center'

            # Check if dog is in "too close" zone
            frame_area = cv_image.shape[0] * cv_image.shape[1]
            area_ratio = dog['area'] / frame_area
            self.last_dog_was_close = area_ratio > self.too_close_area_ratio

        # Use cached bbox for visualization if detection dropped briefly
        display_dog = dog
        if dog is None and self.last_dog_bbox_time is not None:
            elapsed = (self.get_clock().now() - self.last_dog_bbox_time).nanoseconds / 1e9
            if elapsed < self.bbox_persist_duration:
                # Create a "ghost" dog dict for visualization only (not for following)
                x1, y1, x2, y2 = self.last_dog_bbox
                display_dog = {
                    'bbox': self.last_dog_bbox,
                    'confidence': 0.0,  # Mark as cached
                    'area': (x2 - x1) * (y2 - y1),
                    'center_x': (x1 + x2) / 2,
                    'center_y': (y1 + y2) / 2
                }

        # Debug logging - only log dog detections
        if len(dog_detections) > 0:
            top = max(dog_detections, key=lambda d: d[4])
            self.get_logger().info(
                f'Detected {len(dog_detections)} dog(s). '
                f'Best: conf={top[4]:.2f}'
            )

        # Draw bounding boxes on dogs only and publish annotated image
        annotated = self.draw_detections(cv_image, dog_detections, target_dog=display_dog)
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_msg.header = msg.header  # Preserve timestamp
            self.annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

        # Only do following logic if following is enabled
        if not self.following_enabled:
            return

        if dog is not None:
            self.last_detection_time = self.get_clock().now()
            # Reset recovery state - we found the dog again
            self.last_dog_was_close = False
            self.recovery_scan_start_time = None
            self.follow_dog(dog, cv_image.shape)
            if self.current_status != 'following':
                self.current_status = 'following'
                self.publish_status('following')
        else:
            if self.current_status == 'following':
                self.current_status = 'searching'
                self.publish_status('searching')

    def draw_detections(self, image: np.ndarray, detections: list,
                        target_dog: dict = None) -> np.ndarray:
        """
        Draw bounding boxes and labels on detected dogs.

        Args:
            image: BGR image to annotate
            detections: List of dog detections [x1, y1, x2, y2, conf, class_id]
            target_dog: The dog being followed (highlighted differently), or None

        Returns:
            Annotated image copy
        """
        annotated = image.copy()

        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Check if this is the target dog being followed
            is_target = False
            if target_dog is not None and self.following_enabled:
                is_target = self._is_same_detection(det, target_dog)

            # Color and thickness based on whether this is the active target
            if is_target:
                color = self.COLOR_DOG_TARGET
                thickness = 3
                label = f'DOG {conf:.2f} [TARGET]'
            else:
                color = self.COLOR_DOG_DETECTED
                thickness = 2
                label = f'dog {conf:.2f}'

            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

            # Draw label background
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1
            (label_w, label_h), baseline = cv2.getTextSize(
                label, font, font_scale, font_thickness
            )

            # Label background rectangle (above the box)
            cv2.rectangle(
                annotated,
                (x1, y1 - label_h - 8),
                (x1 + label_w + 4, y1),
                color,
                -1  # Filled
            )

            # Label text (white on colored background)
            cv2.putText(
                annotated, label,
                (x1 + 2, y1 - 4),
                font, font_scale,
                (255, 255, 255),
                font_thickness
            )

        # Draw status overlay in top-left corner
        if self.following_enabled:
            status_text = f'Dog Follower: {self.current_status.upper()}'
            status_color = self.COLOR_STATUS_ACTIVE
        else:
            status_text = 'Detection: ON'
            status_color = self.COLOR_DOG_DETECTED

        cv2.putText(
            annotated, status_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            status_color, 2
        )

        return annotated

    def _is_same_detection(self, det: list, dog: dict) -> bool:
        """Check if detection matches the target dog (by bbox center proximity)."""
        x1, y1, x2, y2 = det[:4]
        tx1, ty1, tx2, ty2 = dog['bbox']
        # Simple center proximity check
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        tcx, tcy = (tx1 + tx2) / 2, (ty1 + ty2) / 2
        return abs(cx - tcx) < 20 and abs(cy - tcy) < 20

    def run_inference(self, image: np.ndarray) -> list:
        """
        Run YOLOv8 inference on Hailo-8L using persistent pipeline.

        Args:
            image: BGR image from camera (640x480)

        Returns:
            List of detections: [[x1, y1, x2, y2, confidence, class_id], ...]
        """
        if self.hailo_device is None or self.infer_pipeline is None:
            # Mock mode - return empty detections
            return self._mock_inference(image)

        try:
            # Preprocess: resize for YOLOv8
            # IMPORTANT: Hailo model expects UINT8 input (0-255), NOT normalized float
            h, w = image.shape[:2]
            input_image = cv2.resize(image, (self.YOLO_INPUT_SIZE, self.YOLO_INPUT_SIZE))
            input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
            input_image = input_image.astype(np.uint8)

            # Reshape for batch processing
            input_data = np.expand_dims(input_image, axis=0)

            # Run inference using persistent pipeline (no context manager needed)
            input_dict = {self.input_vstream_info.name: input_data}
            output_dict = self.infer_pipeline.infer(input_dict)

            # Post-process YOLOv8 output
            detections = self._postprocess_yolov8(output_dict, w, h)
            return detections

        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            return []

    def _mock_inference(self, image: np.ndarray) -> list:
        """
        Mock inference for testing without Hailo SDK.
        Returns empty list - no detections in mock mode.
        """
        return []

    def _postprocess_yolov8(self, outputs: dict, orig_w: int, orig_h: int) -> list:
        """
        Post-process Hailo NMS output format.

        Output structure (nested lists, NOT uniform array):
          outputs[name][batch][class_id] = array of detections for that class
          Each detection: [ymin, xmin, ymax, xmax, score] in normalized coords (0-1)

        Each class can have a variable number of detections (or none),
        so we can't convert the whole output to a numpy array directly.
        """
        detections = []

        try:
            output_name = list(outputs.keys())[0]
            output = outputs[output_name]

            # output is a nested list: [batch][class_id][detection]
            # Access batch 0
            batch_output = output[0] if isinstance(output, list) else output[0]

            num_classes = len(batch_output)

            for class_id in range(num_classes):
                class_detections = batch_output[class_id]

                # Skip if no detections for this class
                if class_detections is None or len(class_detections) == 0:
                    continue

                # Convert to numpy if needed (each class array is homogeneous)
                if isinstance(class_detections, list):
                    class_detections = np.array(class_detections)

                # Handle both (N, 5) and (5,) shapes
                if len(class_detections.shape) == 1:
                    class_detections = class_detections.reshape(1, -1)

                for det in class_detections:
                    if len(det) < 5:
                        continue

                    ymin, xmin, ymax, xmax, score = det[:5]

                    if score < self.confidence_threshold:
                        continue

                    # Convert normalized coords to pixels
                    x1 = int(xmin * orig_w)
                    y1 = int(ymin * orig_h)
                    x2 = int(xmax * orig_w)
                    y2 = int(ymax * orig_h)

                    # Clip to image bounds
                    x1 = max(0, min(orig_w, x1))
                    y1 = max(0, min(orig_h, y1))
                    x2 = max(0, min(orig_w, x2))
                    y2 = max(0, min(orig_h, y2))

                    if x2 <= x1 or y2 <= y1:
                        continue

                    detections.append([x1, y1, x2, y2, float(score), class_id])

        except Exception as e:
            self.get_logger().error(f'Postprocess error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

        return detections

    def find_best_dog(self, detections: list, image_shape: tuple) -> dict:
        """
        Find the largest/most confident dog detection.

        Args:
            detections: List of [x1, y1, x2, y2, conf, class_id]
            image_shape: (height, width, channels)

        Returns:
            Dict with dog info or None if no dog found
        """
        dogs = []
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            if int(class_id) == self.DOG_CLASS_ID and conf >= self.confidence_threshold:
                area = (x2 - x1) * (y2 - y1)
                dogs.append({
                    'bbox': (x1, y1, x2, y2),
                    'confidence': conf,
                    'area': area,
                    'center_x': (x1 + x2) / 2,
                    'center_y': (y1 + y2) / 2
                })

        if not dogs:
            return None

        # Return largest dog (closest)
        return max(dogs, key=lambda d: d['area'])

    def follow_dog(self, dog: dict, image_shape: tuple):
        """
        Generate cmd_vel to follow the detected dog.

        Control strategy:
        - Angular: Turn to keep dog centered in frame (proportional control)
        - Linear: FORWARD ONLY - move forward when dog is far, stay still when close

        When dog approaches (large bounding box), the rover rotates to track but
        does not back up. This avoids collisions with furniture since we have no
        rear obstacle detection. As the dog passes by, we keep rotating to track.
        Once the dog moves away (small bounding box), we follow forward.
        """
        h, w = image_shape[:2]
        frame_center_x = w / 2
        frame_area = w * h

        # Calculate errors
        # Positive center_error = dog is right of center, need to turn right (negative angular.z)
        center_error = (dog['center_x'] - frame_center_x) / w  # -0.5 to 0.5

        # Positive distance_error = dog is too far (small box), need to move forward
        area_ratio = dog['area'] / frame_area
        distance_error = self.target_area_ratio - area_ratio

        twist = Twist()

        # Angular control (centering) - proportional with deadzone
        if abs(center_error) > self.center_tolerance:
            # Negative because positive error (dog on right) needs negative angular.z (turn right)
            twist.angular.z = -center_error * self.angular_speed * 4.0
            # Clamp to max speed
            twist.angular.z = np.clip(twist.angular.z, -self.angular_speed, self.angular_speed)

        # Linear control (distance) - FORWARD ONLY, STOP if too close
        # When dog is very close (>40% of frame), stop to avoid losing detection
        # When dog is at target distance, stay still
        # When dog moves away, follow forward
        area_deadzone = 0.03  # 3% area variation is acceptable
        if area_ratio > self.too_close_area_ratio:
            # Dog is very close - stop forward motion, rotate only to track
            twist.linear.x = 0.0
        elif distance_error > area_deadzone:
            # Dog is too far (small in frame) - move forward to follow
            twist.linear.x = distance_error * self.linear_speed * 10.0
            # Clamp to max forward speed
            twist.linear.x = min(twist.linear.x, self.linear_speed)
        # When dog is close but not too close, linear.x stays 0 - rotate only

        # Mark this as our publish time (for teleop detection)
        with self.cmd_vel_lock:
            self.our_last_publish_time = self.get_clock().now()

        self.cmd_vel_pub.publish(twist)

    def safety_check(self):
        """
        Periodic safety checks (only when following is enabled):
        1. Stop if no dog detected for too long
        2. Auto-disable following if teleop is active
        """
        if not self.following_enabled:
            return

        now = self.get_clock().now()

        # Check for teleop override
        with self.cmd_vel_lock:
            if self.last_external_cmd_time is not None:
                elapsed = (now - self.last_external_cmd_time).nanoseconds / 1e9
                if elapsed < self.teleop_override_timeout:
                    self.get_logger().warn('Teleop override detected - disabling following')
                    self.following_enabled = False
                    self.stop_robot()
                    self.current_status = 'teleop_override'
                    self.publish_status('teleop_override')
                    # Reset external cmd time
                    self.last_external_cmd_time = None
                    return

        # Check for lost target
        if self.last_detection_time is not None:
            elapsed = (now - self.last_detection_time).nanoseconds / 1e9
            if elapsed > self.detection_timeout:
                # If we lost dog after it was very close, try predictive rotation recovery
                if self.last_dog_was_close and self.current_status not in ['recovery_scan', 'lost_target']:
                    self.current_status = 'recovery_scan'
                    self.recovery_scan_start_time = now
                    # Rotate toward last known position
                    if self.last_dog_frame_position == 'left':
                        self.recovery_rotation_direction = self.angular_speed  # Rotate left (positive)
                    elif self.last_dog_frame_position == 'right':
                        self.recovery_rotation_direction = -self.angular_speed  # Rotate right (negative)
                    else:
                        # Was centered - default to rotating right
                        self.recovery_rotation_direction = -self.angular_speed
                    self.publish_status('recovery_scan')
                    self.get_logger().info(f'Dog lost after close contact - scanning {self.last_dog_frame_position}')

                # Handle recovery scan state
                if self.current_status == 'recovery_scan':
                    # Check if recovery scan timed out
                    if self.recovery_scan_start_time is not None:
                        scan_elapsed = (now - self.recovery_scan_start_time).nanoseconds / 1e9
                        if scan_elapsed > self.recovery_scan_timeout:
                            # Give up recovery - go to lost_target
                            self.stop_robot()
                            self.current_status = 'lost_target'
                            self.publish_status('lost_target')
                            self.last_dog_was_close = False  # Reset for next time
                            self.get_logger().info('Recovery scan timeout - target lost')
                            return

                    # Continue recovery rotation
                    twist = Twist()
                    twist.angular.z = self.recovery_rotation_direction
                    with self.cmd_vel_lock:
                        self.our_last_publish_time = self.get_clock().now()
                    self.cmd_vel_pub.publish(twist)
                else:
                    # Normal lost target - stop
                    self.stop_robot()
                    if self.current_status != 'lost_target':
                        self.current_status = 'lost_target'
                        self.publish_status('lost_target')
                        self.get_logger().info('Target lost - stopping')

    def stop_robot(self):
        """Publish zero velocity to stop the robot."""
        # Mark as our publish
        with self.cmd_vel_lock:
            self.our_last_publish_time = self.get_clock().now()

        self.cmd_vel_pub.publish(Twist())

    def publish_status(self, status: str):
        """Publish current status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.current_status = status

    def publish_status_timer(self):
        """Periodic status publishing for dashboard."""
        self.publish_status(self.current_status)

    def destroy_node(self):
        """Cleanup on shutdown."""
        try:
            self.stop_robot()
        except Exception:
            pass  # ROS context might already be invalid during shutdown

        # Close persistent inference pipeline context
        if self.infer_pipeline is not None:
            try:
                self.infer_pipeline.__exit__(None, None, None)
            except Exception:
                pass

        # Exit activation context
        if self.activated_network_group is not None:
            try:
                self.activated_network_group.__exit__(None, None, None)
            except Exception:
                pass

        # Release device
        if self.hailo_device is not None:
            try:
                self.hailo_device.release()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DogFollower()

    # MultiThreadedExecutor required for ReentrantCallbackGroup
    # Allows service calls to execute while image callbacks are running
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
