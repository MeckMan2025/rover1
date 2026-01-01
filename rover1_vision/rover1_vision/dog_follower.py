#!/usr/bin/env python3
"""
Dog Follower Node - Uses Hailo-8L for YOLOv8 dog detection and follows detected dogs.

This node uses the Hailo-8L AI accelerator to run real-time YOLOv8 inference,
detecting dogs (COCO class 16) and generating velocity commands to follow them.

SAFETY FEATURES:
  - Disabled by default, must be explicitly enabled via service
  - Auto-disables if teleop sends /cmd_vel (teleop override)
  - Stops if no dog detected for configurable timeout
  - Velocity limits enforced in hardware

Subscribes:
  - /ascamera_hp60c/camera_publisher/rgb0/image (sensor_msgs/Image)
  - /cmd_vel (geometry_msgs/Twist) - for teleop override detection

Publishes:
  - /cmd_vel (geometry_msgs/Twist) - when enabled and following
  - /dog_follower/status (std_msgs/String) - status updates

Services:
  - /dog_follower/enable (std_srvs/Trigger) - start following
  - /dog_follower/disable (std_srvs/Trigger) - stop following
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

    def __init__(self):
        super().__init__('dog_follower')

        # Callback groups for concurrent handling
        self.service_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.bridge = CvBridge()
        self.enabled = False
        self.last_detection_time = None
        self.current_status = 'idle'

        # Thread safety for cmd_vel detection
        self.cmd_vel_lock = threading.Lock()
        self.last_external_cmd_time = None
        self.our_last_publish_time = None

        # Declare parameters
        self.declare_parameter('model_path', os.path.expanduser(
            '~/ros2_ws/src/rover1/models/yolov8s.hef'))
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_box_area_ratio', 0.15)  # Target: dog fills 15% of frame
        self.declare_parameter('linear_speed', 0.2)  # m/s - conservative for safety
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('center_tolerance', 0.1)  # 10% of frame width deadzone
        self.declare_parameter('detection_timeout', 1.0)  # Stop if no dog for 1s
        self.declare_parameter('teleop_override_timeout', 0.5)  # Disable if teleop active

        # Load parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.target_area_ratio = self.get_parameter('target_box_area_ratio').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.teleop_override_timeout = self.get_parameter('teleop_override_timeout').value

        # Initialize Hailo inference engine
        self.hailo_device = None
        self.hailo_hef = None
        self.hailo_network_group = None
        self.input_vstream_info = None
        self.output_vstream_info = None
        self._init_hailo()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/dog_follower/status', 10)

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

        # Services
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
        """Initialize Hailo-8L inference engine."""
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

            # Get input/output stream info
            self.input_vstream_info = self.hailo_hef.get_input_vstream_infos()[0]
            self.output_vstream_info = self.hailo_hef.get_output_vstream_infos()

            self.get_logger().info(f'Hailo model loaded successfully')
            self.get_logger().info(f'Input shape: {self.input_vstream_info.shape}')
            self.get_logger().info(f'Output layers: {len(self.output_vstream_info)}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Hailo: {e}')
            self.get_logger().warn('Running in mock mode')
            self.hailo_device = None

    def enable_callback(self, request, response):
        """Enable dog following."""
        if self.enabled:
            response.success = False
            response.message = 'Dog follower already enabled'
            return response

        self.enabled = True
        self.last_detection_time = self.get_clock().now()
        self.current_status = 'searching'

        response.success = True
        response.message = 'Dog follower enabled - searching for dog'
        self.get_logger().info('Dog follower ENABLED')
        self.publish_status('searching')
        return response

    def disable_callback(self, request, response):
        """Disable dog following."""
        was_enabled = self.enabled
        self.enabled = False
        self.stop_robot()
        self.current_status = 'idle'

        response.success = True
        if was_enabled:
            response.message = 'Dog follower disabled'
            self.get_logger().info('Dog follower DISABLED')
        else:
            response.message = 'Dog follower was not enabled'

        self.publish_status('idle')
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
        """Process camera images and detect dogs."""
        if not self.enabled:
            return

        try:
            # Convert ROS image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Run inference
        detections = self.run_inference(cv_image)

        # Find best dog detection
        dog = self.find_best_dog(detections, cv_image.shape)

        if dog is not None:
            self.last_detection_time = self.get_clock().now()
            self.follow_dog(dog, cv_image.shape)
            if self.current_status != 'following':
                self.current_status = 'following'
                self.publish_status('following')
        else:
            if self.current_status == 'following':
                self.current_status = 'searching'
                self.publish_status('searching')

    def run_inference(self, image: np.ndarray) -> list:
        """
        Run YOLOv8 inference on Hailo-8L.

        Args:
            image: BGR image from camera (640x480)

        Returns:
            List of detections: [[x1, y1, x2, y2, confidence, class_id], ...]
        """
        if self.hailo_device is None or self.hailo_network_group is None:
            # Mock mode - return empty detections
            # In real deployment, this would be replaced with actual inference
            return self._mock_inference(image)

        try:
            # Preprocess: resize and normalize for YOLOv8
            h, w = image.shape[:2]
            input_image = cv2.resize(image, (self.YOLO_INPUT_SIZE, self.YOLO_INPUT_SIZE))
            input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
            input_image = input_image.astype(np.float32) / 255.0

            # Reshape for batch processing
            input_data = np.expand_dims(input_image, axis=0)

            # Create VStream params
            input_params = InputVStreamParams.make_from_network_group(
                self.hailo_network_group, quantized=False,
                format_type=FormatType.FLOAT32
            )
            output_params = OutputVStreamParams.make_from_network_group(
                self.hailo_network_group, quantized=False,
                format_type=FormatType.FLOAT32
            )

            # Run inference
            with InferVStreams(self.hailo_network_group, input_params, output_params) as infer_pipeline:
                input_dict = {self.input_vstream_info.name: input_data}
                output_dict = infer_pipeline.infer(input_dict)

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
        Post-process YOLOv8 outputs to get detection boxes.

        YOLOv8 output format varies by model variant. This handles the common
        format of [batch, num_detections, 85] where 85 = 4 (box) + 1 (obj) + 80 (classes)
        """
        detections = []

        try:
            # Get the output tensor (name varies by model)
            output_name = list(outputs.keys())[0]
            output = outputs[output_name]

            # Handle different output formats
            if len(output.shape) == 3:
                # Shape: [batch, num_boxes, 85]
                output = output[0]  # Remove batch dimension

            # Scale factors for original image
            scale_x = orig_w / self.YOLO_INPUT_SIZE
            scale_y = orig_h / self.YOLO_INPUT_SIZE

            for detection in output:
                if len(detection) < 6:
                    continue

                # YOLOv8 format: [x_center, y_center, width, height, conf, class_scores...]
                x_center, y_center, width, height = detection[:4]
                confidence = detection[4]
                class_scores = detection[5:]

                if confidence < self.confidence_threshold:
                    continue

                class_id = int(np.argmax(class_scores))
                class_conf = class_scores[class_id]

                # Combine objectness and class confidence
                final_conf = confidence * class_conf
                if final_conf < self.confidence_threshold:
                    continue

                # Convert to corner format and scale to original image
                x1 = int((x_center - width / 2) * scale_x)
                y1 = int((y_center - height / 2) * scale_y)
                x2 = int((x_center + width / 2) * scale_x)
                y2 = int((y_center + height / 2) * scale_y)

                # Clip to image bounds
                x1 = max(0, min(orig_w, x1))
                y1 = max(0, min(orig_h, y1))
                x2 = max(0, min(orig_w, x2))
                y2 = max(0, min(orig_h, y2))

                detections.append([x1, y1, x2, y2, final_conf, class_id])

        except Exception as e:
            self.get_logger().debug(f'Postprocess error: {e}')

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
        - Linear: Move forward/backward to maintain target distance (based on box area)
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
            twist.angular.z = -center_error * self.angular_speed * 2.0
            # Clamp to max speed
            twist.angular.z = np.clip(twist.angular.z, -self.angular_speed, self.angular_speed)

        # Linear control (distance) - proportional with deadzone
        area_deadzone = 0.03  # 3% area variation is acceptable
        if abs(distance_error) > area_deadzone:
            # Positive error (too far) = positive linear.x (forward)
            twist.linear.x = distance_error * self.linear_speed * 5.0
            # Clamp to max speed
            twist.linear.x = np.clip(twist.linear.x, -self.linear_speed, self.linear_speed)

        # Mark this as our publish time (for teleop detection)
        with self.cmd_vel_lock:
            self.our_last_publish_time = self.get_clock().now()

        self.cmd_vel_pub.publish(twist)

    def safety_check(self):
        """
        Periodic safety checks:
        1. Stop if no dog detected for too long
        2. Auto-disable if teleop is active
        """
        if not self.enabled:
            return

        now = self.get_clock().now()

        # Check for teleop override
        with self.cmd_vel_lock:
            if self.last_external_cmd_time is not None:
                elapsed = (now - self.last_external_cmd_time).nanoseconds / 1e9
                if elapsed < self.teleop_override_timeout:
                    self.get_logger().warn('Teleop override detected - disabling dog follower')
                    self.enabled = False
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
        self.stop_robot()
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
