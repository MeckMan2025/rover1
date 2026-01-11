#!/usr/bin/env python3
"""
Trace Follower - Dead-reckoning waypoint following for Trace & Retrace

Replays recorded motion using wheel odometry only.
No SLAM, no GPS - just follows the same path relative to start position.

This is the simplest form of teach & repeat:
- Starts at current position (which becomes the new origin)
- Navigates to each recorded relative position using P-controller
- Uses only wheel odometry for position feedback

Services:
  /trace/start - Start following waypoints
  /trace/stop - Stop following
  /trace/pause - Pause at current position
  /trace/resume - Resume from paused state

Publishes:
  /cmd_vel - Velocity commands
  /patrol/status - Status updates
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
import yaml
import math
import os
from pathlib import Path
from enum import Enum

from rover1_patrol.msg import PatrolStatus
from rover1_patrol.srv import StartPatrol


class FollowerState(Enum):
    IDLE = "idle"
    FOLLOWING = "following"
    PAUSED = "paused"
    COMPLETE = "complete"


class TraceFollower(Node):
    """Dead-reckoning waypoint follower using wheel odometry."""

    def __init__(self):
        super().__init__('trace_follower')

        # Parameters
        self.declare_parameter('paths_directory', os.path.expanduser('~/paths'))
        self.declare_parameter('waypoint_tolerance', 0.15)  # 15cm arrival threshold
        self.declare_parameter('angular_tolerance', 0.10)   # ~6 degrees
        self.declare_parameter('max_linear_speed', 0.25)    # m/s (conservative)
        self.declare_parameter('max_angular_speed', 0.6)    # rad/s
        self.declare_parameter('linear_p_gain', 0.8)
        self.declare_parameter('angular_p_gain', 1.2)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('odom_topic', '/odom/wheel_odom')

        self.paths_dir = Path(self.get_parameter('paths_directory').value)
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.linear_p = self.get_parameter('linear_p_gain').value
        self.angular_p = self.get_parameter('angular_p_gain').value
        odom_topic = self.get_parameter('odom_topic').value

        # State
        self.state = FollowerState.IDLE
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.current_loop = 0
        self.total_loops = 0
        self.reverse_mode = False
        self.going_reverse = False
        self.speed_scale = 1.0
        self.path_name = ""
        self.error_message = ""

        # Odometry tracking
        self.start_pose = None  # (x, y, theta) at patrol start
        self.current_pose = None  # Latest (x, y, theta) from odom

        # Teleop preemption
        self.teleop_active = False
        self.last_teleop_time = None
        self.teleop_timeout = 0.5

        # Subscribe to wheel odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(PatrolStatus, '/patrol/status', 10)

        # Callback group
        self.cb_group = ReentrantCallbackGroup()

        # Services
        self.start_srv = self.create_service(
            StartPatrol, '/trace/start',
            self.start_callback,
            callback_group=self.cb_group
        )
        self.stop_srv = self.create_service(
            Trigger, '/trace/stop',
            self.stop_callback,
            callback_group=self.cb_group
        )
        self.pause_srv = self.create_service(
            Trigger, '/trace/pause',
            self.pause_callback,
            callback_group=self.cb_group
        )
        self.resume_srv = self.create_service(
            Trigger, '/trace/resume',
            self.resume_callback,
            callback_group=self.cb_group
        )

        # Speed scale subscriber
        self.speed_sub = self.create_subscription(
            Float32, '/patrol/speed_scale',
            self.speed_callback, 10
        )

        # Teleop preemption - yield to gamepad
        self.teleop_sub = self.create_subscription(
            Twist, '/teleop/cmd_vel',
            self.teleop_callback, 10
        )

        # Control loop timer
        rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)

        # Status publishing timer
        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f'Trace Follower ready. Tolerance: {self.waypoint_tolerance}m'
        )

    def odom_callback(self, msg: Odometry):
        """Process wheel odometry to track position."""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation

        # Convert quaternion to yaw
        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.current_pose = (pos.x, pos.y, theta)

    def get_relative_pose(self):
        """Get current pose relative to start position."""
        if self.current_pose is None or self.start_pose is None:
            return None

        # Calculate position relative to start
        rel_x = self.current_pose[0] - self.start_pose[0]
        rel_y = self.current_pose[1] - self.start_pose[1]
        rel_theta = self.current_pose[2] - self.start_pose[2]

        # Normalize theta
        while rel_theta > math.pi:
            rel_theta -= 2 * math.pi
        while rel_theta < -math.pi:
            rel_theta += 2 * math.pi

        return (rel_x, rel_y, rel_theta)

    def load_path(self, path_name: str) -> bool:
        """Load waypoints from YAML file."""
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        yaml_path = self.paths_dir / f'{safe_name}.yaml'

        if not yaml_path.exists():
            self.error_message = f'Path file not found: {yaml_path}'
            self.get_logger().error(self.error_message)
            return False

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            self.waypoints = data.get('waypoints', [])
            self.path_name = data.get('name', path_name)

            if len(self.waypoints) == 0:
                self.error_message = 'Path has no waypoints'
                self.get_logger().error(self.error_message)
                return False

            self.get_logger().info(
                f'Loaded trace path "{self.path_name}" with {len(self.waypoints)} waypoints'
            )
            return True

        except Exception as e:
            self.error_message = f'Failed to load path: {e}'
            self.get_logger().error(self.error_message)
            return False

    def control_loop(self):
        """Main control loop - navigate to waypoints."""
        if self.state != FollowerState.FOLLOWING:
            return

        # Yield to teleop
        if self.is_teleop_active():
            return

        # Get current relative pose
        pose = self.get_relative_pose()
        if pose is None:
            return

        x, y, yaw = pose

        # Check if we've finished all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.handle_path_complete()
            return

        # Get target waypoint (handle reverse traversal)
        if self.going_reverse:
            idx = len(self.waypoints) - 1 - self.current_waypoint_idx
        else:
            idx = self.current_waypoint_idx

        target = self.waypoints[idx]
        tx, ty = target['x'], target['y']

        # Compute distance and heading to target
        dx = tx - x
        dy = ty - y
        distance = math.sqrt(dx * dx + dy * dy)
        target_heading = math.atan2(dy, dx)

        # Compute heading error
        heading_error = target_heading - yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Check if we've reached the waypoint
        if distance < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx}/{len(self.waypoints)}'
            )
            return

        # Compute velocities using P-controller
        cmd = Twist()

        # Only move forward if roughly pointing at target
        if abs(heading_error) < self.angular_tolerance * 3:
            linear_vel = self.linear_p * distance
            linear_vel = min(linear_vel, self.max_linear * self.speed_scale)

            # Slow down when turning
            turn_factor = 1.0 - min(abs(heading_error) / math.pi, 0.8)
            cmd.linear.x = linear_vel * turn_factor
        else:
            # Turn in place if heading error is large
            cmd.linear.x = 0.0

        # Angular velocity
        angular_vel = self.angular_p * heading_error
        angular_vel = max(-self.max_angular, min(angular_vel, self.max_angular))
        cmd.angular.z = angular_vel * self.speed_scale

        self.cmd_pub.publish(cmd)

    def handle_path_complete(self):
        """Handle completion of waypoint sequence."""
        self.get_logger().info('Trace sequence complete')

        # Handle reverse mode (retrace back to start)
        if self.reverse_mode and not self.going_reverse:
            self.going_reverse = True
            self.current_waypoint_idx = 0
            self.get_logger().info('Retracing back to start')
            return

        # Complete one loop
        self.current_loop += 1
        self.going_reverse = False
        self.current_waypoint_idx = 0

        self.get_logger().info(f'Completed loop {self.current_loop}/{self.total_loops}')

        # Check if done
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
        """Publish patrol status."""
        # Don't publish when idle (let recorder own the topic)
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
        msg.gps_fix_available = False
        msg.gps_waypoint_count = 0

        self.status_pub.publish(msg)

    # Service callbacks

    def start_callback(self, request, response):
        """Start following a trace path."""
        if self.state == FollowerState.FOLLOWING:
            response.success = False
            response.message = 'Already following. Stop first.'
            return response

        if self.current_pose is None:
            response.success = False
            response.message = 'No odometry data. Is wheel odometry running?'
            return response

        if not self.load_path(request.path_name):
            response.success = False
            response.message = self.error_message
            return response

        # Set start pose to current position
        self.start_pose = self.current_pose

        self.total_loops = max(0, request.loop_count)
        self.reverse_mode = request.reverse_mode
        self.speed_scale = max(0.1, min(1.0, request.speed_percent))
        self.current_loop = 0
        self.current_waypoint_idx = 0
        self.going_reverse = False
        self.error_message = ""
        self.state = FollowerState.FOLLOWING

        self.get_logger().info(
            f'Starting trace: path="{self.path_name}", '
            f'loops={self.total_loops if self.total_loops > 0 else "infinite"}, '
            f'reverse={self.reverse_mode}'
        )

        response.success = True
        response.message = f'Trace started: {self.path_name}'
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
        self.start_pose = None

        self.get_logger().info('Trace stopped')

        response.success = True
        response.message = 'Trace stopped'
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

        self.get_logger().info('Trace resumed')

        response.success = True
        response.message = 'Trace resumed'
        return response

    def speed_callback(self, msg: Float32):
        """Handle speed scale updates."""
        self.speed_scale = max(0.1, min(1.0, msg.data))

    def teleop_callback(self, msg: Twist):
        """Handle teleop commands - yield control to operator."""
        has_motion = (abs(msg.linear.x) > 0.01 or
                      abs(msg.linear.y) > 0.01 or
                      abs(msg.angular.z) > 0.01)

        if has_motion:
            self.last_teleop_time = self.get_clock().now()
            if not self.teleop_active:
                self.teleop_active = True
                self.get_logger().info('Teleop active - yielding control')

    def is_teleop_active(self) -> bool:
        """Check if teleop is currently active."""
        if self.last_teleop_time is None:
            return False

        elapsed = (self.get_clock().now() - self.last_teleop_time).nanoseconds / 1e9
        if elapsed > self.teleop_timeout:
            if self.teleop_active:
                self.teleop_active = False
                self.get_logger().info('Teleop inactive - resuming trace')
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TraceFollower()

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
