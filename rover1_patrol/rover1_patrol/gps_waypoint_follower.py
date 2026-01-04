#!/usr/bin/env python3
"""
GPS Waypoint Follower - RTK GPS-Based Autonomous Navigation

Waypoint following using RTK GPS for outdoor operation.
Uses lat/lon coordinates and haversine calculations.

Use Case: Outdoor demos, long-range paths, open areas.

Subscribes:
  - /fix: NavSatFix GPS position (BEST_EFFORT QoS)
  - /imu/data: IMU for heading (optional, fallback to course-over-ground)

Publishes:
  - /cmd_vel: Twist commands for navigation

Services:
  - /patrol/gps_start: Start following GPS waypoints
  - /patrol/gps_stop: Stop following
  - /patrol/gps_pause: Pause at current position
  - /patrol/gps_resume: Resume from paused state
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
import yaml
import math
import os
from pathlib import Path
from enum import Enum
from datetime import datetime

from rover1_patrol.msg import PatrolStatus
from rover1_patrol.srv import StartPatrol


class FollowerState(Enum):
    IDLE = "idle"
    FOLLOWING = "following"
    PAUSED = "paused"
    COMPLETE = "complete"
    NO_GPS = "no_gps"


class GPSWaypointFollower(Node):
    """GPS-based waypoint following for outdoor navigation."""

    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # Parameters
        self.declare_parameter('paths_directory', os.path.expanduser('~/paths'))
        self.declare_parameter('waypoint_tolerance', 1.0)   # 1m for GPS (RTK ~2cm, but allow margin)
        self.declare_parameter('angular_tolerance', 0.2)    # ~11 degrees
        self.declare_parameter('max_linear_speed', 0.4)     # m/s
        self.declare_parameter('max_angular_speed', 0.8)    # rad/s
        self.declare_parameter('linear_p_gain', 0.5)        # Proportional gain
        self.declare_parameter('angular_p_gain', 1.2)       # Proportional gain
        self.declare_parameter('control_rate', 10.0)        # Hz (GPS is slower than odom)
        self.declare_parameter('gps_timeout', 2.0)          # Seconds without GPS = stop

        self.paths_dir = Path(self.get_parameter('paths_directory').value)
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.linear_p = self.get_parameter('linear_p_gain').value
        self.angular_p = self.get_parameter('angular_p_gain').value
        self.gps_timeout = self.get_parameter('gps_timeout').value

        # State
        self.state = FollowerState.IDLE
        self.waypoints = []  # List of {lat, lon} dicts
        self.current_waypoint_idx = 0
        self.current_loop = 0
        self.total_loops = 0
        self.reverse_mode = False
        self.going_reverse = False
        self.speed_scale = 1.0
        self.path_name = ""
        self.error_message = ""

        # GPS state
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None  # From IMU or course-over-ground
        self.last_gps_time = None
        self.last_lat = None
        self.last_lon = None

        # QoS for GPS (BEST_EFFORT to match ublox driver)
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fix',
            self.gps_callback,
            gps_qos
        )

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(PatrolStatus, '/patrol/status', 10)

        # Callback group
        self.cb_group = ReentrantCallbackGroup()

        # Services
        self.start_srv = self.create_service(
            StartPatrol, '/patrol/gps_start',
            self.start_callback,
            callback_group=self.cb_group
        )
        self.stop_srv = self.create_service(
            Trigger, '/patrol/gps_stop',
            self.stop_callback,
            callback_group=self.cb_group
        )
        self.pause_srv = self.create_service(
            Trigger, '/patrol/gps_pause',
            self.pause_callback,
            callback_group=self.cb_group
        )
        self.resume_srv = self.create_service(
            Trigger, '/patrol/gps_resume',
            self.resume_callback,
            callback_group=self.cb_group
        )

        # Speed scale subscriber (from dashboard)
        self.speed_sub = self.create_subscription(
            Float32, '/patrol/speed_scale',
            self.speed_callback, 10
        )

        # Control loop timer
        rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)

        # Status publishing timer
        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f'GPS Waypoint Follower ready. '
            f'Tolerance: {self.waypoint_tolerance}m'
        )

    def gps_callback(self, msg: NavSatFix):
        """Process GPS position updates."""
        if msg.status.status < 0:  # No fix
            return

        # Store previous position for heading calculation
        if self.current_lat is not None:
            self.last_lat = self.current_lat
            self.last_lon = self.current_lon

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_time = datetime.now()

        # Calculate course-over-ground heading if no IMU
        if self.last_lat is not None and self.current_heading is None:
            distance = self.haversine_distance(
                self.last_lat, self.last_lon,
                self.current_lat, self.current_lon
            )
            if distance > 0.1:  # Only update if moved >10cm
                self.current_heading = self.bearing(
                    self.last_lat, self.last_lon,
                    self.current_lat, self.current_lon
                )

    def imu_callback(self, msg: Imu):
        """Process IMU for heading."""
        # Extract yaw from quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points in meters."""
        R = 6371000  # Earth radius in meters

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = (math.sin(delta_phi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    @staticmethod
    def bearing(lat1, lon1, lat2, lon2):
        """Calculate bearing from point 1 to point 2 in radians."""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)

        x = math.sin(delta_lambda) * math.cos(phi2)
        y = (math.cos(phi1) * math.sin(phi2) -
             math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda))

        return math.atan2(x, y)

    def load_path(self, path_name: str) -> bool:
        """Load GPS waypoints from YAML file."""
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        yaml_path = self.paths_dir / f'{safe_name}_gps.yaml'

        # Fallback to regular path file
        if not yaml_path.exists():
            yaml_path = self.paths_dir / f'{safe_name}.yaml'

        if not yaml_path.exists():
            self.error_message = f'Path file not found: {yaml_path}'
            self.get_logger().error(self.error_message)
            return False

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            # Check for GPS waypoints
            if 'gps_waypoints' in data:
                self.waypoints = data['gps_waypoints']
            elif 'waypoints' in data:
                # Check if waypoints have lat/lon
                wps = data['waypoints']
                if wps and 'lat' in wps[0]:
                    self.waypoints = wps
                else:
                    self.error_message = 'Path does not contain GPS waypoints (lat/lon)'
                    self.get_logger().error(self.error_message)
                    return False
            else:
                self.error_message = 'No waypoints found in path file'
                self.get_logger().error(self.error_message)
                return False

            self.path_name = data.get('name', path_name)

            if len(self.waypoints) == 0:
                self.error_message = 'Path has no waypoints'
                self.get_logger().error(self.error_message)
                return False

            self.get_logger().info(
                f'Loaded GPS path "{self.path_name}" with {len(self.waypoints)} waypoints'
            )
            return True

        except Exception as e:
            self.error_message = f'Failed to load path: {e}'
            self.get_logger().error(self.error_message)
            return False

    def control_loop(self):
        """Main control loop - compute and publish velocity commands."""
        if self.state != FollowerState.FOLLOWING:
            return

        # Check GPS timeout
        if self.last_gps_time is None:
            self.state = FollowerState.NO_GPS
            self.error_message = 'Waiting for GPS fix'
            return

        elapsed = (datetime.now() - self.last_gps_time).total_seconds()
        if elapsed > self.gps_timeout:
            self.state = FollowerState.NO_GPS
            self.error_message = f'GPS timeout ({elapsed:.1f}s)'
            self.stop_robot()
            return

        # Restore from NO_GPS state
        if self.state == FollowerState.NO_GPS:
            self.state = FollowerState.FOLLOWING
            self.error_message = ""

        # Check we have position
        if self.current_lat is None or self.current_lon is None:
            return

        # Get target waypoint
        if self.current_waypoint_idx >= len(self.waypoints):
            self.handle_path_complete()
            return

        # Handle reverse traversal
        if self.going_reverse:
            idx = len(self.waypoints) - 1 - self.current_waypoint_idx
        else:
            idx = self.current_waypoint_idx

        target = self.waypoints[idx]
        target_lat = target['lat']
        target_lon = target['lon']

        # Compute distance and bearing to target
        distance = self.haversine_distance(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )
        target_bearing = self.bearing(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )

        # Compute heading error
        if self.current_heading is None:
            # No heading available, just drive forward slowly
            heading_error = 0.0
        else:
            heading_error = target_bearing - self.current_heading
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi

        # Check if we've reached the waypoint
        if distance < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'Reached GPS waypoint {self.current_waypoint_idx}/{len(self.waypoints)} '
                f'({target_lat:.6f}, {target_lon:.6f})'
            )
            return

        # Compute velocities using P-controller
        cmd = Twist()

        # Only move forward if roughly pointing at target
        if abs(heading_error) < self.angular_tolerance * 3:
            linear_vel = self.linear_p * distance
            linear_vel = min(linear_vel, self.max_linear * self.speed_scale)

            turn_factor = 1.0 - min(abs(heading_error) / math.pi, 0.8)
            cmd.linear.x = linear_vel * turn_factor
        else:
            cmd.linear.x = 0.0

        # Angular velocity proportional to heading error
        angular_vel = self.angular_p * heading_error
        angular_vel = max(-self.max_angular, min(angular_vel, self.max_angular))
        cmd.angular.z = angular_vel * self.speed_scale

        self.cmd_pub.publish(cmd)

    def handle_path_complete(self):
        """Handle completion of waypoint sequence."""
        self.get_logger().info('GPS waypoint sequence complete')

        if self.reverse_mode and not self.going_reverse:
            self.going_reverse = True
            self.current_waypoint_idx = 0
            self.get_logger().info('Starting reverse pass')
            return

        self.current_loop += 1
        self.going_reverse = False
        self.current_waypoint_idx = 0

        self.get_logger().info(f'Completed loop {self.current_loop}/{self.total_loops}')

        if self.total_loops > 0 and self.current_loop >= self.total_loops:
            self.state = FollowerState.COMPLETE
            self.stop_robot()
            self.get_logger().info('All loops complete!')
        else:
            self.get_logger().info('Starting next loop')

    def stop_robot(self):
        """Send zero velocity command."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def publish_status(self):
        """Publish patrol status for dashboard."""
        # Let waypoint_recorder own /patrol/status when we're idle
        if self.state == FollowerState.IDLE:
            return

        msg = PatrolStatus()
        msg.state = self.state.value
        msg.path_name = self.path_name
        msg.current_waypoint = self.current_waypoint_idx
        msg.total_waypoints = len(self.waypoints)
        msg.current_loop = self.current_loop
        msg.total_loops = self.total_loops
        msg.speed_percent = self.speed_scale
        msg.error_message = self.error_message
        msg.recorded_waypoint_count = 0

        self.status_pub.publish(msg)

    # Service callbacks

    def start_callback(self, request, response):
        """Start following GPS waypoints."""
        if self.state == FollowerState.FOLLOWING:
            response.success = False
            response.message = 'Already following. Stop first.'
            return response

        if not self.load_path(request.path_name):
            response.success = False
            response.message = self.error_message
            return response

        self.total_loops = max(0, request.loop_count)
        self.reverse_mode = request.reverse_mode
        self.speed_scale = max(0.1, min(1.0, request.speed_percent))
        self.current_loop = 0
        self.current_waypoint_idx = 0
        self.going_reverse = False
        self.error_message = ""
        self.state = FollowerState.FOLLOWING

        self.get_logger().info(
            f'Starting GPS patrol: path="{self.path_name}", '
            f'loops={self.total_loops if self.total_loops > 0 else "infinite"}'
        )

        response.success = True
        response.message = f'GPS patrol started: {self.path_name}'
        return response

    def stop_callback(self, request, response):
        """Stop following."""
        if self.state == FollowerState.IDLE:
            response.success = False
            response.message = 'Not currently following'
            return response

        self.state = FollowerState.IDLE
        self.stop_robot()
        self.waypoints = []
        self.path_name = ""
        self.error_message = ""

        self.get_logger().info('GPS patrol stopped')

        response.success = True
        response.message = 'Patrol stopped'
        return response

    def pause_callback(self, request, response):
        """Pause following."""
        if self.state != FollowerState.FOLLOWING:
            response.success = False
            response.message = 'Not currently following'
            return response

        self.state = FollowerState.PAUSED
        self.stop_robot()

        self.get_logger().info(f'Paused at waypoint {self.current_waypoint_idx}')

        response.success = True
        response.message = f'Paused at waypoint {self.current_waypoint_idx}'
        return response

    def resume_callback(self, request, response):
        """Resume following."""
        if self.state != FollowerState.PAUSED:
            response.success = False
            response.message = 'Not currently paused'
            return response

        self.state = FollowerState.FOLLOWING

        self.get_logger().info('Resumed GPS patrol')

        response.success = True
        response.message = 'Patrol resumed'
        return response

    def speed_callback(self, msg: Float32):
        """Handle speed scale updates."""
        self.speed_scale = max(0.1, min(1.0, msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
