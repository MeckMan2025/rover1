#!/usr/bin/env python3
"""
Person Follower Node - Uses Hailo-8L for YOLOv8 person detection and follows detected persons.

This is the core Pack Robot functionality: following humans using computer vision.

This node uses the Hailo-8L AI accelerator to run real-time YOLOv8 inference,
detecting persons (COCO class 0) and generating velocity commands to follow them.

APPROACH:
  - Uses person bounding box tracking with foot position (bottom-center of bbox)
  - Camera is mounted low (33cm) and angled down, so feet position in frame indicates distance
  - Lower in frame = closer person, higher in frame = farther person

TWO-STATE DESIGN:
  - Detection: Controls whether Hailo inference runs and annotated images are published
  - Following: Controls whether the rover moves to follow detected persons (requires detection on)

SAFETY FEATURES:
  - Both detection and following disabled by default
  - Auto-disables following if teleop sends /cmd_vel (teleop override)
  - Stops if no person detected for configurable timeout
  - Velocity limits enforced in hardware

Subscribes:
  - /ascamera_hp60c/camera_publisher/rgb0/image (sensor_msgs/Image)
  - /cmd_vel (geometry_msgs/Twist) - for teleop override detection

Publishes:
  - /cmd_vel (geometry_msgs/Twist) - when following enabled
  - /person_follower/status (std_msgs/String) - status updates
  - /person_follower/annotated_image (sensor_msgs/Image) - when detection enabled

Services:
  - ~/enable_detection (std_srvs/Trigger) - Start/stop person detection (visual only)
  - ~/disable_detection (std_srvs/Trigger) - Stop detection completely  
  - ~/enable_following (std_srvs/Trigger) - Start following detected persons
  - ~/disable_following (std_srvs/Trigger) - Stop following (keeps detection on)

Author: Rover2 Pack Robot Project
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import cv2
import numpy as np
import threading
import time
from typing import List, Dict, Optional, Tuple

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

# Hailo imports
try:
    from hailo_platform import HailoRT, VDevice, FormatType, HailoStreamInterface
    import hailo_platform.pyhailort as hailort
    HAILO_AVAILABLE = True
except ImportError as e:
    print(f"Hailo import failed: {e}")
    print("Running in simulation mode (no actual inference)")
    HAILO_AVAILABLE = False


class PersonFollower(Node):
    """ROS 2 node for person detection and following using Hailo-8L."""
    
    # Detection colors (BGR format for OpenCV)
    COLOR_PERSON_DETECTED = (0, 255, 0)     # Green - detected person
    COLOR_PERSON_TARGET = (0, 255, 255)     # Yellow - active target being followed
    COLOR_BBOX_LINE_THICKNESS = 2
    COLOR_FOOT_MARKER = (255, 0, 0)         # Blue - foot position marker
    COLOR_TARGET_LINE = (0, 255, 255)       # Yellow - target line
    
    # Status constants
    STATUS_IDLE = "idle"
    STATUS_DETECTING = "detecting"
    STATUS_FOLLOWING = "following"
    STATUS_TELEOP_OVERRIDE = "teleop_override"
    STATUS_NO_PERSON = "no_person"
    STATUS_RECOVERY_SCAN = "recovery_scan"
    
    def __init__(self):
        super().__init__('person_follower')
        
        # Callback groups for concurrent operation
        self.service_cb_group = ReentrantCallbackGroup()
        self.image_cb_group = MutuallyExclusiveCallbackGroup()
        
        # State management
        self.detection_enabled = False
        self.following_enabled = False
        self.current_target_person = None  # Track which person we're following for visualization
        self.last_detection_time = 0.0
        self.last_teleop_time = 0.0
        self.recovery_scan_start_time = 0.0
        self.angular_recovery_direction = 1  # 1 for right, -1 for left
        
        # Parameters
        self.declare_parameters()
        
        # Load AI model (lazy loading for better startup performance)
        self.model = None
        self.model_input_shape = None
        self.model_loaded = False
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # QoS profiles
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            qos_profile=self.image_qos,
            callback_group=self.image_cb_group
        )
        
        # Monitor /cmd_vel for teleop override detection
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.image_cb_group
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/person_follower/status', 10)
        self.annotated_image_pub = self.create_publisher(
            Image, 
            '/person_follower/annotated_image', 
            qos_profile=self.image_qos
        )
        
        # Services
        self.enable_detection_service = self.create_service(
            Trigger, 
            '~/enable_detection', 
            self.enable_detection_callback,
            callback_group=self.service_cb_group
        )
        self.disable_detection_service = self.create_service(
            Trigger, 
            '~/disable_detection', 
            self.disable_detection_callback,
            callback_group=self.service_cb_group
        )
        self.enable_following_service = self.create_service(
            Trigger, 
            '~/enable_following', 
            self.enable_following_callback,
            callback_group=self.service_cb_group
        )
        self.disable_following_service = self.create_service(
            Trigger, 
            '~/disable_following', 
            self.disable_following_callback,
            callback_group=self.service_cb_group
        )
        
        # Status monitoring timer
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10Hz
        
        self.get_logger().info("Person Follower initialized - Pack Robot ready for human following")
    
    def declare_parameters(self):
        """Declare ROS parameters with defaults."""
        self.declare_parameter('model_path', '/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('target_foot_y_ratio', 0.70)
        self.declare_parameter('too_close_foot_y_ratio', 0.85)
        self.declare_parameter('center_tolerance', 0.12)
        self.declare_parameter('detection_timeout', 2.0)
        self.declare_parameter('teleop_override_timeout', 0.5)
        self.declare_parameter('recovery_scan_timeout', 4.0)
        
        # Cache parameter values
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.target_foot_y_ratio = self.get_parameter('target_foot_y_ratio').value
        self.too_close_foot_y_ratio = self.get_parameter('too_close_foot_y_ratio').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.teleop_override_timeout = self.get_parameter('teleop_override_timeout').value
        self.recovery_scan_timeout = self.get_parameter('recovery_scan_timeout').value
        
    def load_model(self):
        """Lazy-load the Hailo AI model."""
        if self.model_loaded or not HAILO_AVAILABLE:
            return True
            
        try:
            self.get_logger().info(f"Loading YOLOv8 model from: {self.model_path}")
            
            # Initialize Hailo device
            self.vdevice = VDevice()
            self.hef = hailort.HEF(self.model_path)
            
            # Configure model
            self.infer_model = self.vdevice.create_infer_model(self.hef)
            self.model_input_shape = self.hef.get_input_stream_infos()[0].shape
            
            self.get_logger().info(f"Model loaded successfully. Input shape: {self.model_input_shape}")
            self.model_loaded = True
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            return False
    
    def enable_detection_callback(self, request, response):
        """Enable person detection (visual only, no following)."""
        if not self.load_model():
            response.success = False
            response.message = 'Failed to load AI model'
            return response
            
        self.detection_enabled = True
        response.success = True
        response.message = 'Person detection enabled - visual tracking only'
        self.get_logger().info("Person detection enabled")
        return response
    
    def disable_detection_callback(self, request, response):
        """Disable person detection completely."""
        self.detection_enabled = False
        self.following_enabled = False  # Also disable following
        self.stop_rover()
        response.success = True
        response.message = 'Person detection disabled'
        self.get_logger().info("Person detection disabled")
        return response
    
    def enable_following_callback(self, request, response):
        """Enable person following (auto-enables detection if needed)."""
        if not self.load_model():
            response.success = False
            response.message = 'Failed to load AI model'
            return response
            
        self.detection_enabled = True  # Auto-enable detection
        self.following_enabled = True
        response.success = True
        response.message = 'Person follower enabled - searching for person'
        self.get_logger().info("Person following enabled")
        return response
    
    def disable_following_callback(self, request, response):
        """Disable person following (keeps detection on for visualization)."""
        self.following_enabled = False
        self.stop_rover()
        response.success = True
        response.message = 'Person following disabled (detection still active for visualization)'
        self.get_logger().info("Person following disabled")
        return response
    
    def cmd_vel_callback(self, msg: Twist):
        """Monitor /cmd_vel for teleop override detection."""
        # Check if this is a non-zero command (indicating active teleop)
        if (abs(msg.linear.x) > 0.01 or 
            abs(msg.linear.y) > 0.01 or 
            abs(msg.angular.z) > 0.01):
            self.last_teleop_time = time.time()
    
    def is_teleop_active(self) -> bool:
        """Check if teleop override is currently active."""
        return (time.time() - self.last_teleop_time) < self.teleop_override_timeout
    
    def get_current_status(self) -> str:
        """Get current operational status."""
        if self.is_teleop_active():
            return self.STATUS_TELEOP_OVERRIDE
        elif not self.detection_enabled:
            return self.STATUS_IDLE
        elif self.following_enabled:
            time_since_detection = time.time() - self.last_detection_time
            if time_since_detection > self.detection_timeout:
                if (time.time() - self.recovery_scan_start_time) < self.recovery_scan_timeout:
                    return self.STATUS_RECOVERY_SCAN
                else:
                    return self.STATUS_NO_PERSON
            else:
                return self.STATUS_FOLLOWING
        else:
            return self.STATUS_DETECTING
    
    def publish_status(self):
        """Publish current status."""
        status_msg = String()
        status_msg.data = self.get_current_status()
        self.status_pub.publish(status_msg)
    
    def image_callback(self, msg: Image):
        """Main image processing callback."""
        if not self.detection_enabled:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run inference if model is loaded
            detections = []
            if self.model_loaded and HAILO_AVAILABLE:
                detections = self.run_inference(cv_image)
            
            # Find best person (for following and target highlight)
            best_person = self.find_best_person(detections, cv_image.shape)
            
            # Update detection timing
            if best_person:
                self.last_detection_time = time.time()
                self.current_target_person = best_person
                # Reset recovery scan
                self.recovery_scan_start_time = 0.0
            else:
                # Start recovery scan if we were following and lost the person
                if (self.following_enabled and 
                    self.recovery_scan_start_time == 0.0 and 
                    (time.time() - self.last_detection_time) > self.detection_timeout):
                    self.recovery_scan_start_time = time.time()
                    self.get_logger().info("Person lost - starting recovery scan")
            
            # Generate annotated image for visualization
            annotated_image = self.create_annotated_image(
                cv_image.copy(), 
                detections, 
                best_person if self.following_enabled else None
            )
            
            # Publish annotated image
            self.publish_annotated_image(annotated_image, msg.header.stamp)
            
            # Handle movement
            current_status = self.get_current_status()
            
            if current_status == self.STATUS_TELEOP_OVERRIDE:
                # Don't send commands during teleop override
                pass
            elif current_status == self.STATUS_FOLLOWING and best_person:
                # Follow the detected person
                self.follow_person(best_person, cv_image.shape)
            elif current_status == self.STATUS_RECOVERY_SCAN:
                # Perform recovery scan
                self.perform_recovery_scan()
            else:
                # Stop movement for all other states
                if self.following_enabled:  # Only stop if we were supposed to be following
                    self.stop_rover()
                
        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")
    
    def create_annotated_image(self, cv_image: np.ndarray, detections: List[Dict], 
                             target_person: Optional[Dict]) -> np.ndarray:
        """
        Create annotated image with bounding boxes and status info.
        
        Args:
            cv_image: Original camera image
            detections: List of detected persons
            target_person: The person being followed (highlighted differently), or None
        """
        height, width = cv_image.shape[:2]
        
        # Draw all person detections
        for person in detections:
            x1, y1, x2, y2 = person['bbox']
            confidence = person['confidence']
            
            # Check if this is the target person being followed
            is_target = False
            if target_person is not None and self.following_enabled:
                # Simple comparison - in practice you might want a more robust matching
                if (abs(x1 - target_person['bbox'][0]) < 10 and 
                    abs(y1 - target_person['bbox'][1]) < 10):
                    is_target = True
            
            # Choose color based on target status
            color = self.COLOR_PERSON_TARGET if is_target else self.COLOR_PERSON_DETECTED
            
            # Draw bounding box
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), 
                         color, self.COLOR_BBOX_LINE_THICKNESS)
            
            # Draw confidence
            label = f"Person: {confidence:.2f}"
            if is_target:
                label += " [FOLLOWING]"
            
            # Calculate text size and position
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            
            # Draw text background
            cv2.rectangle(cv_image, 
                         (int(x1), int(y1) - text_height - baseline - 10),
                         (int(x1) + text_width, int(y1)),
                         color, -1)
            
            # Draw text
            cv2.putText(cv_image, label, (int(x1), int(y1) - baseline - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # Draw foot position marker (bottom-center of bbox)
            foot_x = int((x1 + x2) / 2)
            foot_y = int(y2)
            cv2.circle(cv_image, (foot_x, foot_y), 5, self.COLOR_FOOT_MARKER, -1)
        
        # Draw target line and distance info for following mode
        if target_person and self.following_enabled:
            self.draw_following_info(cv_image, target_person)
        
        # Draw status info
        self.draw_status_overlay(cv_image)
        
        return cv_image
    
    def draw_following_info(self, cv_image: np.ndarray, person: Dict):
        """Draw following-specific information on the image."""
        height, width = cv_image.shape[:2]
        x1, y1, x2, y2 = person['bbox']
        
        # Calculate foot position
        foot_x = int((x1 + x2) / 2)
        foot_y = int(y2)
        foot_y_ratio = foot_y / height
        
        # Draw target line (where we want the feet to be)
        target_y = int(height * self.target_foot_y_ratio)
        cv2.line(cv_image, (0, target_y), (width, target_y), 
                self.COLOR_TARGET_LINE, 2)
        cv2.putText(cv_image, "TARGET", (10, target_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLOR_TARGET_LINE, 1)
        
        # Draw too-close line
        too_close_y = int(height * self.too_close_foot_y_ratio)
        cv2.line(cv_image, (0, too_close_y), (width, too_close_y), 
                (0, 0, 255), 2)
        cv2.putText(cv_image, "TOO CLOSE", (10, too_close_y + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Show distance info
        distance_info = f"Foot Y: {foot_y_ratio:.2f} (Target: {self.target_foot_y_ratio:.2f})"
        cv2.putText(cv_image, distance_info, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    def draw_status_overlay(self, cv_image: np.ndarray):
        """Draw status information overlay."""
        status = self.get_current_status()
        height, width = cv_image.shape[:2]
        
        # Status text
        status_text = f"Status: {status.upper()}"
        if self.detection_enabled:
            status_text += " | Detection: ON"
        else:
            status_text += " | Detection: OFF"
            
        if self.following_enabled:
            status_text += " | Following: ON"
        else:
            status_text += " | Following: OFF"
        
        # Draw status background
        (text_width, text_height), baseline = cv2.getTextSize(
            status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
        )
        cv2.rectangle(cv_image, (0, height - text_height - baseline - 10),
                     (text_width + 20, height), (0, 0, 0), -1)
        
        # Draw status text
        cv2.putText(cv_image, status_text, (10, height - baseline - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    def publish_annotated_image(self, cv_image: np.ndarray, timestamp):
        """Publish annotated image."""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = 'camera_link'
            self.annotated_image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")
    
    def run_inference(self, cv_image: np.ndarray) -> List[Dict]:
        """Run YOLOv8 inference on the image."""
        if not HAILO_AVAILABLE or not self.model_loaded:
            return []
            
        try:
            # Preprocess image
            input_data = self.preprocess_image(cv_image)
            
            # Run inference
            results = self.infer_model.run([input_data])
            
            # Post-process results
            detections = self.postprocess_results(results, cv_image.shape)
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return []
    
    def preprocess_image(self, cv_image: np.ndarray) -> np.ndarray:
        """Preprocess image for YOLOv8 inference."""
        # Resize to model input size
        model_height, model_width = self.model_input_shape[1:3]
        resized = cv2.resize(cv_image, (model_width, model_height))
        
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # Normalize and reshape
        input_data = rgb_image.astype(np.float32) / 255.0
        input_data = np.transpose(input_data, (2, 0, 1))  # HWC to CHW
        input_data = np.expand_dims(input_data, axis=0)  # Add batch dimension
        
        return input_data
    
    def postprocess_results(self, results, original_shape) -> List[Dict]:
        """Post-process YOLOv8 results to extract person detections."""
        if not results:
            return []
            
        # This is a simplified post-processing - you may need to adjust
        # based on your specific YOLOv8 model output format
        detections = []
        
        try:
            # Extract detection data (format may vary by model)
            output = results[0]
            
            # Scale factors for converting model coordinates to image coordinates
            orig_height, orig_width = original_shape[:2]
            model_height, model_width = self.model_input_shape[1:3]
            scale_x = orig_width / model_width
            scale_y = orig_height / model_height
            
            # Process detections (this is model-specific)
            # You'll need to adapt this based on your YOLOv8 output format
            for detection in output:
                confidence = detection[4]  # Assuming confidence is at index 4
                class_id = int(detection[5])  # Assuming class ID is at index 5
                
                # Only process person detections (COCO class 0)
                if class_id == 0 and confidence > self.confidence_threshold:
                    x1 = detection[0] * scale_x
                    y1 = detection[1] * scale_y
                    x2 = detection[2] * scale_x
                    y2 = detection[3] * scale_y
                    
                    detections.append({
                        'bbox': [x1, y1, x2, y2],
                        'confidence': confidence,
                        'class_id': class_id
                    })
                    
        except Exception as e:
            self.get_logger().error(f"Post-processing error: {e}")
        
        return detections
    
    def find_best_person(self, detections: List[Dict], image_shape: tuple) -> Optional[Dict]:
        """Find the best person to follow based on position and size."""
        if not detections:
            return None
        
        height, width = image_shape[:2]
        best_person = None
        best_score = -1
        
        for person in detections:
            x1, y1, x2, y2 = person['bbox']
            
            # Calculate foot position (bottom center of bounding box)
            foot_x = (x1 + x2) / 2
            foot_y = y2
            
            # Prefer persons closer to center horizontally
            center_x = width / 2
            horizontal_distance = abs(foot_x - center_x) / width
            
            # Prefer persons at reasonable distance (not too close, not too far)
            foot_y_ratio = foot_y / height
            distance_score = 1.0 - abs(foot_y_ratio - self.target_foot_y_ratio)
            
            # Combined score (higher is better)
            score = person['confidence'] * distance_score * (1.0 - horizontal_distance)
            
            if score > best_score:
                best_score = score
                best_person = person
                
                # Add foot position for following calculations
                best_person['foot_x'] = foot_x
                best_person['foot_y'] = foot_y
                best_person['foot_y_ratio'] = foot_y_ratio
        
        return best_person
    
    def follow_person(self, person: dict, image_shape: tuple):
        """
        Generate cmd_vel to follow the detected person.
        
        Args:
            person: Dict containing person detection info including foot position
            image_shape: Tuple of (height, width, channels)
        """
        height, width = image_shape[:2]
        
        foot_x = person['foot_x']
        foot_y_ratio = person['foot_y_ratio']
        
        # Calculate center offset (normalized to [-1, 1])
        center_x = width / 2
        center_offset = (foot_x - center_x) / (width / 2)
        
        # Initialize command
        cmd = Twist()
        
        # Distance control based on foot Y position
        if foot_y_ratio > self.too_close_foot_y_ratio:
            # Person is too close (feet at bottom of frame) - don't move forward, maybe back up
            cmd.linear.x = -0.1 * self.linear_speed  # Slow backup
        elif foot_y_ratio < self.target_foot_y_ratio:
            # When person moves away (feet high in frame), follow forward
            distance_error = self.target_foot_y_ratio - foot_y_ratio
            # Scale forward speed based on distance
            forward_speed = min(distance_error * 2.0, 1.0)  # Cap at 100%
            cmd.linear.x = forward_speed * self.linear_speed
        else:
            # Person is too far (feet too high in frame) - move forward to follow
            cmd.linear.x = 0.3 * self.linear_speed
        
        # Rotation control - turn toward person
        if abs(center_offset) > self.center_tolerance:
            # Turn toward the person
            rotation_speed = min(abs(center_offset) * 1.5, 1.0)  # Scale with offset, cap at 100%
            cmd.angular.z = -rotation_speed * self.angular_speed * np.sign(center_offset)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log following info (debug)
        if self.get_logger().get_effective_level() <= 10:  # DEBUG level
            self.get_logger().debug(
                f"Following: foot_y_ratio={foot_y_ratio:.2f}, "
                f"center_offset={center_offset:.2f}, "
                f"cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})"
            )
    
    def perform_recovery_scan(self):
        """Perform a slow rotation to search for lost person."""
        cmd = Twist()
        cmd.angular.z = self.angular_recovery_direction * 0.3 * self.angular_speed
        self.cmd_vel_pub.publish(cmd)
        
        # Check if recovery scan timeout exceeded
        if (time.time() - self.recovery_scan_start_time) > self.recovery_scan_timeout:
            self.get_logger().info("Recovery scan timeout - stopping search")
            self.stop_rover()
            # Switch direction for next time
            self.angular_recovery_direction *= -1
    
    def stop_rover(self):
        """Send stop command to rover."""
        cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PersonFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()