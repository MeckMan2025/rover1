#!/usr/bin/env python3
"""
Patrol Manager Node for Teach & Repeat Patrol System

Manages patrol execution by sending waypoints to Nav2's waypoint_follower.
Supports looping, reverse mode, pause/resume, and speed control.

Services:
  /patrol/list_paths - List all saved patrol paths
  /patrol/start_patrol - Start patrolling a saved path
  /patrol/stop_patrol - Stop patrol immediately
  /patrol/pause_patrol - Pause at current position
  /patrol/resume_patrol - Resume from paused state

Publishers:
  /patrol/status - PatrolStatus message
  /patrol/speed_scale - Float32 speed multiplier for velocity_scaler
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus
import yaml
import os
import math
from pathlib import Path
from datetime import datetime

from rover1_patrol.msg import PatrolStatus
from rover1_patrol.srv import ListPaths, StartPatrol


class PatrolManager(Node):
    """Manages patrol execution using Nav2 waypoint_follower."""

    def __init__(self):
        super().__init__('patrol_manager')

        # Parameters
        self.declare_parameter('paths_directory', os.path.expanduser('~/paths'))
        self.declare_parameter('stuck_timeout', 30.0)  # Seconds before declaring stuck

        self.paths_dir = Path(self.get_parameter('paths_directory').value)
        self.stuck_timeout = self.get_parameter('stuck_timeout').value

        # State
        self.state = 'idle'  # idle, patrolling, paused, error
        self.current_path_name = ''
        self.waypoints = []
        self.current_waypoint = 0
        self.current_loop = 0
        self.total_loops = 0  # 0 = infinite
        self.reverse_mode = False
        self.going_reverse = False  # True when traversing in reverse direction
        self.speed_percent = 1.0
        self.error_message = ''

        # For stuck detection
        self.last_waypoint_change_time = None
        self.last_reported_waypoint = -1

        # Goal handle
        self.goal_handle = None
        self.paused_waypoint_index = 0

        # Callback group
        self.cb_group = ReentrantCallbackGroup()

        # Action client for waypoint follower
        self.waypoint_client = ActionClient(
            self, FollowWaypoints, '/follow_waypoints'
        )

        # Services
        self.list_srv = self.create_service(
            ListPaths, '/patrol/list_paths',
            self.list_paths_callback,
            callback_group=self.cb_group
        )
        self.start_srv = self.create_service(
            StartPatrol, '/patrol/start_patrol',
            self.start_patrol_callback,
            callback_group=self.cb_group
        )
        self.stop_srv = self.create_service(
            Trigger, '/patrol/stop_patrol',
            self.stop_patrol_callback,
            callback_group=self.cb_group
        )
        self.pause_srv = self.create_service(
            Trigger, '/patrol/pause_patrol',
            self.pause_patrol_callback,
            callback_group=self.cb_group
        )
        self.resume_srv = self.create_service(
            Trigger, '/patrol/resume_patrol',
            self.resume_patrol_callback,
            callback_group=self.cb_group
        )

        # Publishers
        self.status_pub = self.create_publisher(PatrolStatus, '/patrol/status', 10)
        self.speed_pub = self.create_publisher(Float32, '/patrol/speed_scale', 10)

        # Timer for status publishing
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'Patrol Manager ready. Paths dir: {self.paths_dir}')

    def timer_callback(self):
        """Periodic status publishing and stuck detection."""
        self.publish_status()
        self.publish_speed()

        # Stuck detection
        if self.state == 'patrolling' and self.last_waypoint_change_time:
            elapsed = (datetime.now() - self.last_waypoint_change_time).total_seconds()
            if elapsed > self.stuck_timeout:
                self.get_logger().error(
                    f'Stuck on waypoint {self.current_waypoint} for {elapsed:.1f}s'
                )
                self.cancel_navigation()
                self.state = 'error'
                self.error_message = f'Navigation stuck at waypoint {self.current_waypoint}'

    def publish_status(self):
        """Publish current patrol status."""
        msg = PatrolStatus()
        msg.state = self.state
        msg.path_name = self.current_path_name
        msg.current_waypoint = self.current_waypoint
        msg.total_waypoints = len(self.waypoints)
        msg.current_loop = self.current_loop
        msg.total_loops = self.total_loops
        msg.speed_percent = self.speed_percent
        msg.error_message = self.error_message
        msg.recorded_waypoint_count = 0  # Not used by patrol_manager

        self.status_pub.publish(msg)

    def publish_speed(self):
        """Publish speed scale for velocity_scaler node."""
        msg = Float32()
        msg.data = self.speed_percent if self.state == 'patrolling' else 0.0
        self.speed_pub.publish(msg)

    def load_path(self, path_name: str) -> bool:
        """Load a saved path from YAML file."""
        # Sanitize and find file
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        yaml_path = self.paths_dir / f'{safe_name}.yaml'

        if not yaml_path.exists():
            self.get_logger().error(f'Path file not found: {yaml_path}')
            return False

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            self.waypoints = data.get('waypoints', [])
            self.current_path_name = data.get('name', path_name)

            if len(self.waypoints) == 0:
                self.get_logger().error('Path has no waypoints')
                return False

            self.get_logger().info(
                f'Loaded path "{self.current_path_name}" with {len(self.waypoints)} waypoints'
            )
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to load path: {e}')
            return False

    def waypoints_to_poses(self, start_idx: int = 0, reverse: bool = False) -> list:
        """Convert waypoints to PoseStamped messages for Nav2."""
        poses = []
        waypoints = self.waypoints[start_idx:]

        if reverse:
            waypoints = list(reversed(waypoints))

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0

            # Convert theta to quaternion
            theta = wp.get('theta', 0.0)
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)

            poses.append(pose)

        return poses

    def send_waypoints(self, start_idx: int = 0, reverse: bool = False):
        """Send waypoints to Nav2 waypoint_follower."""
        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Waypoint follower action server not available')
            self.state = 'error'
            self.error_message = 'Nav2 waypoint_follower not available'
            return

        poses = self.waypoints_to_poses(start_idx, reverse)

        if len(poses) == 0:
            self.get_logger().warn('No waypoints to send')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info(
            f'Sending {len(poses)} waypoints (start_idx={start_idx}, reverse={reverse})'
        )

        self.last_waypoint_change_time = datetime.now()
        self.last_reported_waypoint = -1

        self.goal_handle = None
        send_future = self.waypoint_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Waypoint goal rejected')
            self.state = 'error'
            self.error_message = 'Navigation goal rejected'
            return

        self.get_logger().info('Waypoint goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        new_waypoint = feedback.current_waypoint

        if new_waypoint != self.last_reported_waypoint:
            self.current_waypoint = new_waypoint
            self.last_reported_waypoint = new_waypoint
            self.last_waypoint_change_time = datetime.now()
            self.get_logger().info(
                f'Reached waypoint {new_waypoint + 1}/{len(self.waypoints)}'
            )

    def goal_result_callback(self, future):
        """Handle navigation completion."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint sequence completed')
            self.handle_sequence_complete()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation canceled')
            # State already set by stop/pause callback
        else:
            self.get_logger().error(f'Navigation failed with status {status}')
            self.state = 'error'
            self.error_message = f'Navigation failed (status {status})'

    def handle_sequence_complete(self):
        """Handle completion of a waypoint sequence."""
        if self.state != 'patrolling':
            return

        # Check if in reverse mode and need to go back
        if self.reverse_mode:
            if not self.going_reverse:
                # Just finished forward pass, now go reverse
                self.going_reverse = True
                self.get_logger().info('Starting reverse pass')
                self.send_waypoints(start_idx=0, reverse=True)
                return
            else:
                # Finished reverse pass, that's one complete loop
                self.going_reverse = False
                self.current_loop += 1
        else:
            # Not reverse mode, one forward pass = one loop
            self.current_loop += 1

        self.get_logger().info(f'Completed loop {self.current_loop}')

        # Check if we should continue
        if self.total_loops > 0 and self.current_loop >= self.total_loops:
            self.get_logger().info('All loops completed')
            self.state = 'idle'
            self.current_path_name = ''
            return

        # Continue looping
        self.current_waypoint = 0
        self.get_logger().info('Starting next loop')
        self.send_waypoints(start_idx=0, reverse=False)

    def cancel_navigation(self):
        """Cancel current navigation goal."""
        if self.goal_handle is not None:
            self.get_logger().info('Canceling navigation goal')
            cancel_future = self.goal_handle.cancel_goal_async()
            # Don't wait for cancellation to complete
            self.goal_handle = None

    # Service callbacks

    def list_paths_callback(self, request, response):
        """List all saved paths."""
        response.path_names = []
        response.waypoint_counts = []
        response.recorded_dates = []

        try:
            for yaml_file in sorted(self.paths_dir.glob('*.yaml')):
                try:
                    with open(yaml_file, 'r') as f:
                        data = yaml.safe_load(f)

                    name = data.get('name', yaml_file.stem)
                    count = len(data.get('waypoints', []))
                    recorded = data.get('recorded', '')

                    # Parse date for display
                    if recorded:
                        try:
                            dt = datetime.fromisoformat(recorded)
                            date_str = dt.strftime('%Y-%m-%d')
                        except ValueError:
                            date_str = recorded
                    else:
                        date_str = 'Unknown'

                    response.path_names.append(name)
                    response.waypoint_counts.append(count)
                    response.recorded_dates.append(date_str)

                except Exception as e:
                    self.get_logger().warn(f'Failed to read {yaml_file}: {e}')

        except Exception as e:
            self.get_logger().error(f'Failed to list paths: {e}')

        return response

    def start_patrol_callback(self, request, response):
        """Start patrolling a saved path."""
        if self.state == 'patrolling':
            response.success = False
            response.message = 'Already patrolling. Stop first.'
            return response

        # Load path
        if not self.load_path(request.path_name):
            response.success = False
            response.message = f'Failed to load path "{request.path_name}"'
            return response

        # Set patrol parameters
        self.total_loops = max(0, request.loop_count)
        self.reverse_mode = request.reverse_mode
        self.speed_percent = max(0.1, min(1.0, request.speed_percent))
        self.current_loop = 0
        self.current_waypoint = 0
        self.going_reverse = False
        self.error_message = ''
        self.state = 'patrolling'

        self.get_logger().info(
            f'Starting patrol: path="{self.current_path_name}", '
            f'loops={self.total_loops if self.total_loops > 0 else "infinite"}, '
            f'reverse={self.reverse_mode}, speed={self.speed_percent:.0%}'
        )

        # Start navigation
        self.send_waypoints(start_idx=0, reverse=False)

        response.success = True
        response.message = f'Patrol started: {self.current_path_name}'
        return response

    def stop_patrol_callback(self, request, response):
        """Stop patrol immediately."""
        if self.state not in ['patrolling', 'paused']:
            response.success = False
            response.message = 'Not currently patrolling'
            return response

        self.cancel_navigation()
        self.state = 'idle'
        self.current_path_name = ''
        self.waypoints = []
        self.error_message = ''

        self.get_logger().info('Patrol stopped')

        response.success = True
        response.message = 'Patrol stopped'
        return response

    def pause_patrol_callback(self, request, response):
        """Pause patrol at current position."""
        if self.state != 'patrolling':
            response.success = False
            response.message = 'Not currently patrolling'
            return response

        self.paused_waypoint_index = self.current_waypoint
        self.cancel_navigation()
        self.state = 'paused'

        self.get_logger().info(f'Patrol paused at waypoint {self.current_waypoint}')

        response.success = True
        response.message = f'Patrol paused at waypoint {self.current_waypoint}'
        return response

    def resume_patrol_callback(self, request, response):
        """Resume patrol from paused state."""
        if self.state != 'paused':
            response.success = False
            response.message = 'Not currently paused'
            return response

        self.state = 'patrolling'

        self.get_logger().info(f'Resuming patrol from waypoint {self.paused_waypoint_index}')

        # Resume from paused waypoint
        self.send_waypoints(
            start_idx=self.paused_waypoint_index,
            reverse=self.going_reverse
        )

        response.success = True
        response.message = f'Patrol resumed from waypoint {self.paused_waypoint_index}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
