#!/usr/bin/env python3
"""
Waypoint Recorder Node for Teach & Repeat Patrol System

Records robot poses at regular intervals while driving manually.
Saves waypoints to YAML files for later patrol playback.

HYBRID APPROACH (Dec 30, 2025):
  - Uses wheel odometry to detect WHEN the rover has moved (immune to SLAM drift)
  - Uses SLAM TF (map→base_link) for WHERE to record (globally consistent position)
  - This prevents false waypoints when stationary due to SLAM localization noise

Services:
  /patrol/start_recording - Begin recording waypoints
  /patrol/stop_recording - Stop recording (keeps in memory)
  /patrol/save_path - Save recorded path with name
  /patrol/discard_recording - Discard current recording

Publishers:
  /patrol/status - PatrolStatus message with recording state
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformListener, TransformException
import yaml
import os
import math
from datetime import datetime
from pathlib import Path

from rover1_patrol.msg import PatrolStatus
from rover1_patrol.srv import SavePath, SaveMapSnapshot, SavePointCloud


class WaypointRecorder(Node):
    """Records waypoints during manual driving for patrol playback."""

    def __init__(self):
        super().__init__('waypoint_recorder')

        # Parameters
        self.declare_parameter('min_distance', 0.30)  # 30cm = rover length
        self.declare_parameter('paths_directory', os.path.expanduser('~/paths'))
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('odom_topic', '/odom/wheel_odom')

        self.min_distance = self.get_parameter('min_distance').value
        self.paths_dir = Path(self.get_parameter('paths_directory').value)
        self.tf_timeout = self.get_parameter('tf_timeout').value
        odom_topic = self.get_parameter('odom_topic').value

        # Ensure paths directory exists
        self.paths_dir.mkdir(parents=True, exist_ok=True)

        # Recording state
        self.recording = False
        self.waypoints = []
        self.gps_waypoints = []  # GPS coordinates (lat/lon) for each waypoint

        # GPS state - store latest fix for capture when recording waypoints
        self.latest_gps = None  # (lat, lon, status) tuple
        self.gps_fix_valid = False

        # Subscribe to GPS fix
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        # Odometry-based distance tracking (hybrid approach)
        # Accumulate distance from wheel encoders, record position from SLAM
        self.accumulated_distance = 0.0
        self.last_odom_time = None

        # Subscribe to wheel odometry for motion detection
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # TF2 setup (for getting SLAM position when recording)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Callback group for concurrent service handling
        self.cb_group = ReentrantCallbackGroup()

        # Services
        self.start_srv = self.create_service(
            Trigger, '/patrol/start_recording',
            self.start_recording_callback,
            callback_group=self.cb_group
        )
        self.stop_srv = self.create_service(
            Trigger, '/patrol/stop_recording',
            self.stop_recording_callback,
            callback_group=self.cb_group
        )
        self.save_srv = self.create_service(
            SavePath, '/patrol/save_path',
            self.save_path_callback,
            callback_group=self.cb_group
        )
        self.discard_srv = self.create_service(
            Trigger, '/patrol/discard_recording',
            self.discard_recording_callback,
            callback_group=self.cb_group
        )

        # Map snapshot client (will be called when saving)
        self.map_snapshot_client = self.create_client(
            SaveMapSnapshot, '/patrol/save_map_snapshot'
        )

        # Point cloud export client (will be called when saving)
        self.pointcloud_client = self.create_client(
            SavePointCloud, '/patrol/save_pointcloud'
        )

        # Status publisher
        self.status_pub = self.create_publisher(PatrolStatus, '/patrol/status', 10)

        # Timer for TF lookups and status publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.get_logger().info(
            f'Waypoint Recorder ready. Min distance: {self.min_distance}m, '
            f'Paths dir: {self.paths_dir}, Odom topic: {odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        """
        Process wheel odometry to detect actual robot motion.

        This is the TRIGGER for recording - we only record when wheels actually turn.
        This prevents false waypoints from SLAM drift when stationary.
        """
        if not self.recording:
            return

        current_time = self.get_clock().now()

        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        # Calculate time delta
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        if dt <= 0 or dt > 1.0:  # Skip invalid or stale data
            return

        # Get velocity from odometry (twist is in base_link frame)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        # Calculate distance traveled in this time step
        # For Mecanum, we have both forward (vx) and strafe (vy) motion
        distance_step = math.sqrt(vx * vx + vy * vy) * dt

        # Accumulate distance
        self.accumulated_distance += distance_step

        # Check if we've traveled far enough to record a waypoint
        if self.accumulated_distance >= self.min_distance:
            self.record_waypoint_from_slam()
            self.accumulated_distance = 0.0  # Reset accumulator

    def gps_callback(self, msg: NavSatFix):
        """
        Process GPS fix messages and store latest coordinates.

        GPS status codes (NavSatStatus):
          -1 = STATUS_NO_FIX
           0 = STATUS_FIX (standard GPS)
           1 = STATUS_SBAS_FIX (SBAS augmented)
           2 = STATUS_GBAS_FIX (ground-based augmentation / RTK)
        """
        # Check for valid fix (status >= 0 means we have a fix)
        if msg.status.status >= 0:
            self.latest_gps = (msg.latitude, msg.longitude, msg.status.status)
            self.gps_fix_valid = True
        else:
            self.gps_fix_valid = False

    def record_waypoint_from_slam(self):
        """Record current SLAM position as a waypoint, with GPS coordinates if available."""
        transform = self.get_robot_pose()
        if transform is None:
            self.get_logger().warn('Cannot record waypoint: TF lookup failed')
            return

        pose = self.transform_to_pose(transform)
        self.waypoints.append(pose)

        # Capture GPS coordinates if available
        gps_point = None
        if self.gps_fix_valid and self.latest_gps is not None:
            lat, lon, status = self.latest_gps
            gps_point = {
                'lat': float(lat),
                'lon': float(lon),
                'status': int(status)  # 0=GPS, 1=SBAS, 2=RTK
            }
            self.gps_waypoints.append(gps_point)
            gps_str = f', GPS: ({lat:.7f}, {lon:.7f})'
        else:
            # Append None placeholder to keep indices aligned
            self.gps_waypoints.append(None)
            gps_str = ' [no GPS fix]'

        self.get_logger().info(
            f'Recorded waypoint {len(self.waypoints)}: '
            f'({pose["x"]:.2f}, {pose["y"]:.2f}){gps_str}'
        )

    def get_robot_pose(self):
        """Get current robot pose in map frame from SLAM."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )
            return transform
        except TransformException as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return None

    def transform_to_pose(self, transform: TransformStamped) -> dict:
        """Convert TF transform to pose dict."""
        t = transform.transform.translation
        r = transform.transform.rotation

        # Convert quaternion to yaw (theta)
        siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
        cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        return {
            'x': float(t.x),
            'y': float(t.y),
            'theta': float(theta)
        }

    def timer_callback(self):
        """Periodic callback for status publishing."""
        # Publish status (recording happens in odom_callback now)
        self.publish_status()

    def publish_status(self):
        """Publish current status."""
        # Only publish when active (recording or pending_save)
        # When idle, don't publish - avoids conflict with trace_recorder
        if not self.recording and len(self.waypoints) == 0:
            return

        msg = PatrolStatus()

        if self.recording:
            msg.state = 'recording'
        elif len(self.waypoints) > 0:
            msg.state = 'pending_save'
        else:
            msg.state = 'idle'

        msg.recorded_waypoint_count = len(self.waypoints)
        msg.path_name = ''
        msg.current_waypoint = 0
        msg.total_waypoints = len(self.waypoints)
        msg.current_loop = 0
        msg.total_loops = 0
        msg.speed_percent = 1.0
        msg.error_message = ''

        # GPS status for live feedback during recording
        msg.gps_fix_available = self.gps_fix_valid
        msg.gps_waypoint_count = len([gp for gp in self.gps_waypoints if gp is not None])

        self.status_pub.publish(msg)

    def start_recording_callback(self, request, response):
        """Start recording waypoints."""
        if self.recording:
            response.success = False
            response.message = 'Already recording'
            return response

        if len(self.waypoints) > 0:
            response.success = False
            response.message = 'Unsaved waypoints exist. Save or discard first.'
            return response

        self.recording = True
        self.waypoints = []
        self.gps_waypoints = []
        self.accumulated_distance = 0.0
        self.last_odom_time = None

        self.get_logger().info('Started recording waypoints (odom-triggered, GPS enabled)')
        response.success = True
        response.message = 'Recording started (odometry-triggered)'
        return response

    def stop_recording_callback(self, request, response):
        """Stop recording waypoints."""
        if not self.recording:
            response.success = False
            response.message = 'Not currently recording'
            return response

        self.recording = False
        self.get_logger().info(f'Stopped recording. {len(self.waypoints)} waypoints captured.')

        response.success = True
        response.message = f'Recording stopped. {len(self.waypoints)} waypoints ready to save.'
        return response

    def save_path_callback(self, request, response):
        """Save recorded path to YAML file."""
        if self.recording:
            response.success = False
            response.message = 'Stop recording first'
            response.saved_file_path = ''
            return response

        if len(self.waypoints) == 0:
            response.success = False
            response.message = 'No waypoints to save'
            response.saved_file_path = ''
            return response

        path_name = request.path_name.strip()
        if not path_name:
            response.success = False
            response.message = 'Path name cannot be empty'
            response.saved_file_path = ''
            return response

        # Sanitize filename
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        if not safe_name:
            response.success = False
            response.message = 'Invalid path name'
            response.saved_file_path = ''
            return response

        # Filter out None values from GPS waypoints (keep only valid fixes)
        valid_gps = [gp for gp in self.gps_waypoints if gp is not None]
        gps_coverage = len(valid_gps) / len(self.waypoints) * 100 if self.waypoints else 0

        # Create YAML data
        path_data = {
            'name': path_name,
            'recorded': datetime.now().isoformat(),
            'waypoint_spacing_m': self.min_distance,
            'waypoint_count': len(self.waypoints),
            'gps_waypoint_count': len(valid_gps),
            'gps_coverage_percent': round(gps_coverage, 1),
            'waypoints': self.waypoints,
            'gps_waypoints': valid_gps  # Only save waypoints with valid GPS
        }

        # Save YAML file
        yaml_path = self.paths_dir / f'{safe_name}.yaml'
        try:
            with open(yaml_path, 'w') as f:
                yaml.dump(path_data, f, default_flow_style=False)
            self.get_logger().info(f'Saved path to {yaml_path}')
        except Exception as e:
            response.success = False
            response.message = f'Failed to save YAML: {e}'
            response.saved_file_path = ''
            return response

        # Request map snapshot
        if self.map_snapshot_client.wait_for_service(timeout_sec=2.0):
            snapshot_req = SaveMapSnapshot.Request()
            snapshot_req.path_name = safe_name
            snapshot_req.waypoints_x = [wp['x'] for wp in self.waypoints]
            snapshot_req.waypoints_y = [wp['y'] for wp in self.waypoints]

            future = self.map_snapshot_client.call_async(snapshot_req)
            # Don't wait for response - let it complete async
            self.get_logger().info('Map snapshot requested')
        else:
            self.get_logger().warn('Map snapshot service not available')

        # Request point cloud export (for 3D visualization)
        if self.pointcloud_client.wait_for_service(timeout_sec=2.0):
            cloud_req = SavePointCloud.Request()
            cloud_req.path_name = safe_name

            future = self.pointcloud_client.call_async(cloud_req)
            # Don't wait for response - let it complete async
            self.get_logger().info('Point cloud export requested')
        else:
            self.get_logger().warn('Point cloud export service not available')

        # Clear waypoints and reset state
        self.waypoints = []
        self.gps_waypoints = []
        self.accumulated_distance = 0.0
        self.last_odom_time = None

        gps_msg = f' ({len(valid_gps)} with GPS)' if valid_gps else ' (no GPS)'
        response.success = True
        response.message = f'Path "{path_name}" saved with {path_data["waypoint_count"]} waypoints{gps_msg}'
        response.saved_file_path = str(yaml_path)
        return response

    def discard_recording_callback(self, request, response):
        """Discard current recording."""
        was_recording = self.recording
        had_waypoints = len(self.waypoints) > 0

        self.recording = False
        self.waypoints = []
        self.gps_waypoints = []
        self.accumulated_distance = 0.0
        self.last_odom_time = None

        if was_recording or had_waypoints:
            response.success = True
            response.message = 'Recording discarded'
            self.get_logger().info('Recording discarded')
        else:
            response.success = True
            response.message = 'Nothing to discard'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
