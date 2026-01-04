#!/usr/bin/env python3
"""
Trace Recorder - Simple motion recording for Trace & Retrace

Records robot motion from wheel odometry at regular intervals.
Saves relative positions for dead-reckoning playback.

This is the simplest form of teach & repeat:
- No SLAM, no GPS, no map frame
- Just wheel odometry (odom frame)
- Records where the robot went relative to start position

Services:
  /trace/start_recording - Begin recording
  /trace/stop_recording - Stop recording (keeps in memory)
  /trace/save_path - Save recorded path with name
  /trace/discard_recording - Discard current recording

Publishers:
  /patrol/status - PatrolStatus message with recording state
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import yaml
import os
import math
from datetime import datetime
from pathlib import Path

from rover1_patrol.msg import PatrolStatus
from rover1_patrol.srv import SavePath


class TraceRecorder(Node):
    """Records motion from wheel odometry for dead-reckoning playback."""

    def __init__(self):
        super().__init__('trace_recorder')

        # Parameters
        self.declare_parameter('min_distance', 0.20)  # 20cm between waypoints
        self.declare_parameter('min_rotation', 0.15)  # ~8.5 degrees
        self.declare_parameter('paths_directory', os.path.expanduser('~/paths'))
        self.declare_parameter('odom_topic', '/odom/wheel_odom')

        self.min_distance = self.get_parameter('min_distance').value
        self.min_rotation = self.get_parameter('min_rotation').value
        self.paths_dir = Path(self.get_parameter('paths_directory').value)
        odom_topic = self.get_parameter('odom_topic').value

        # Ensure paths directory exists
        self.paths_dir.mkdir(parents=True, exist_ok=True)

        # Recording state
        self.recording = False
        self.waypoints = []  # List of (x, y, theta) relative to start
        self.start_pose = None  # (x, y, theta) at recording start
        self.last_recorded_pose = None  # Last recorded (x, y, theta)

        # Subscribe to wheel odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Callback group for concurrent service handling
        self.cb_group = ReentrantCallbackGroup()

        # Services
        self.start_srv = self.create_service(
            Trigger, '/trace/start_recording',
            self.start_recording_callback,
            callback_group=self.cb_group
        )
        self.stop_srv = self.create_service(
            Trigger, '/trace/stop_recording',
            self.stop_recording_callback,
            callback_group=self.cb_group
        )
        self.save_srv = self.create_service(
            SavePath, '/trace/save_path',
            self.save_path_callback,
            callback_group=self.cb_group
        )
        self.discard_srv = self.create_service(
            Trigger, '/trace/discard_recording',
            self.discard_recording_callback,
            callback_group=self.cb_group
        )

        # Status publisher
        self.status_pub = self.create_publisher(PatrolStatus, '/patrol/status', 10)

        # Timer for status publishing
        self.timer = self.create_timer(0.1, self.publish_status)

        self.get_logger().info(
            f'Trace Recorder ready. Min distance: {self.min_distance}m, '
            f'Min rotation: {math.degrees(self.min_rotation):.1f}°'
        )

    def odom_callback(self, msg: Odometry):
        """Process wheel odometry and record waypoints when moved enough."""
        if not self.recording:
            return

        # Extract current pose from odometry
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        # Convert quaternion to yaw
        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        current_pose = (pos.x, pos.y, theta)

        # Initialize start pose on first odom message while recording
        if self.start_pose is None:
            self.start_pose = current_pose
            self.last_recorded_pose = current_pose
            # Record the starting point (0, 0, 0 relative to itself)
            self.waypoints.append({'x': 0.0, 'y': 0.0, 'theta': 0.0})
            self.get_logger().info('Recording started at current position')
            return

        # Calculate distance and rotation from last recorded point
        dx = current_pose[0] - self.last_recorded_pose[0]
        dy = current_pose[1] - self.last_recorded_pose[1]
        distance = math.sqrt(dx * dx + dy * dy)

        dtheta = current_pose[2] - self.last_recorded_pose[2]
        # Normalize to [-pi, pi]
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi

        # Record if moved enough (distance OR rotation)
        if distance >= self.min_distance or abs(dtheta) >= self.min_rotation:
            self.record_waypoint(current_pose)

    def record_waypoint(self, current_pose):
        """Record a waypoint relative to start position."""
        # Calculate position relative to start
        rel_x = current_pose[0] - self.start_pose[0]
        rel_y = current_pose[1] - self.start_pose[1]
        rel_theta = current_pose[2] - self.start_pose[2]

        # Normalize theta
        while rel_theta > math.pi:
            rel_theta -= 2 * math.pi
        while rel_theta < -math.pi:
            rel_theta += 2 * math.pi

        waypoint = {
            'x': float(rel_x),
            'y': float(rel_y),
            'theta': float(rel_theta)
        }

        self.waypoints.append(waypoint)
        self.last_recorded_pose = current_pose

        self.get_logger().info(
            f'Recorded waypoint {len(self.waypoints)}: '
            f'({rel_x:.2f}, {rel_y:.2f}, {math.degrees(rel_theta):.1f}°)'
        )

    def publish_status(self):
        """Publish current status."""
        # Only publish when active (recording or pending_save)
        # When idle, let waypoint_recorder own /patrol/status
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
        msg.gps_fix_available = False
        msg.gps_waypoint_count = 0

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
        self.start_pose = None
        self.last_recorded_pose = None

        self.get_logger().info('Trace recording started')
        response.success = True
        response.message = 'Recording started (odom-based trace)'
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

        # Create YAML data
        path_data = {
            'name': path_name,
            'type': 'trace',  # Marks this as a trace recording
            'recorded': datetime.now().isoformat(),
            'waypoint_spacing_m': self.min_distance,
            'waypoint_count': len(self.waypoints),
            'waypoints': self.waypoints,
        }

        # Save YAML file
        yaml_path = self.paths_dir / f'{safe_name}.yaml'
        try:
            with open(yaml_path, 'w') as f:
                yaml.dump(path_data, f, default_flow_style=False)
            self.get_logger().info(f'Saved trace path to {yaml_path}')
        except Exception as e:
            response.success = False
            response.message = f'Failed to save YAML: {e}'
            response.saved_file_path = ''
            return response

        # Clear waypoints and reset state
        self.waypoints = []
        self.start_pose = None
        self.last_recorded_pose = None

        response.success = True
        response.message = f'Trace path "{path_name}" saved with {path_data["waypoint_count"]} waypoints'
        response.saved_file_path = str(yaml_path)
        return response

    def discard_recording_callback(self, request, response):
        """Discard current recording."""
        was_recording = self.recording
        had_waypoints = len(self.waypoints) > 0

        self.recording = False
        self.waypoints = []
        self.start_pose = None
        self.last_recorded_pose = None

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
    node = TraceRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
