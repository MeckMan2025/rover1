#!/usr/bin/env python3
"""
Rover1 Web Dashboard Server

Web server that provides real-time monitoring and control:
- GNSS/RTK status from /gnss/health
- Camera feed
- Patrol system controls (Teach & Repeat)

Accessible from any device on the network.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import asyncio
import websockets
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os
from pathlib import Path
import subprocess

from gnss_health_monitor.msg import GnssHealth
from rover1_patrol.msg import PatrolStatus
from rover1_patrol.srv import ListPaths, StartPatrol, SavePath
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import base64
import time
from ament_index_python.packages import get_package_share_directory


class Rover1WebDashboard(Node):
    """Web dashboard server for Rover1 monitoring and patrol control"""

    def __init__(self):
        super().__init__('rover1_web_dashboard')

        # Callback group for service calls
        self.cb_group = ReentrantCallbackGroup()

        # Latest data
        self.latest_health = None
        self.latest_patrol_status = None
        self.health_lock = threading.Lock()

        # Broadcast throttling (reduce WebSocket traffic for smoother teleop)
        self.last_health_broadcast = 0
        self.health_broadcast_interval = 0.5  # 2Hz for GNSS updates (was every callback ~10Hz)
        self.last_patrol_broadcast = 0
        self.patrol_broadcast_interval = 0.2  # 5Hz for patrol status

        # Paths directory for map images
        self.paths_dir = Path(os.path.expanduser('~/paths'))

        # Connected WebSocket clients and event loop
        self.ws_clients = set()
        self.ws_loop = None

        # Subscribe to GNSS health with reliable QoS
        self.health_sub = self.create_subscription(
            GnssHealth,
            '/gnss/health',
            self.health_callback,
            10
        )

        # Subscribe to patrol status
        self.patrol_sub = self.create_subscription(
            PatrolStatus,
            '/patrol/status',
            self.patrol_status_callback,
            10
        )

        # Create patrol service clients
        self.patrol_clients = {
            'list_paths': self.create_client(ListPaths, '/patrol/list_paths'),
            'start_recording': self.create_client(Trigger, '/patrol/start_recording'),
            'stop_recording': self.create_client(Trigger, '/patrol/stop_recording'),
            'save_path': self.create_client(SavePath, '/patrol/save_path'),
            'discard_recording': self.create_client(Trigger, '/patrol/discard_recording'),
            # Nav2 mode (original patrol_manager - heavy CPU)
            'nav2_start': self.create_client(StartPatrol, '/patrol/start_patrol'),
            'nav2_stop': self.create_client(Trigger, '/patrol/stop_patrol'),
            'nav2_pause': self.create_client(Trigger, '/patrol/pause_patrol'),
            'nav2_resume': self.create_client(Trigger, '/patrol/resume_patrol'),
            # Odometry mode (simple_waypoint_follower - lightweight)
            'odometry_start': self.create_client(StartPatrol, '/patrol/simple_start'),
            'odometry_stop': self.create_client(Trigger, '/patrol/simple_stop'),
            'odometry_pause': self.create_client(Trigger, '/patrol/simple_pause'),
            'odometry_resume': self.create_client(Trigger, '/patrol/simple_resume'),
            # GPS mode (gps_waypoint_follower - outdoor)
            'gps_start': self.create_client(StartPatrol, '/patrol/gps_start'),
            'gps_stop': self.create_client(Trigger, '/patrol/gps_stop'),
            'gps_pause': self.create_client(Trigger, '/patrol/gps_pause'),
            'gps_resume': self.create_client(Trigger, '/patrol/gps_resume'),
        }

        # Track current auto mode for patrol operations
        self.current_auto_mode = 'odometry'  # Default to lightweight mode

        # Dog Follower service clients
        self.dog_follower_clients = {
            'detection_enable': self.create_client(Trigger, '/dog_follower/detection_enable'),
            'detection_disable': self.create_client(Trigger, '/dog_follower/detection_disable'),
            'enable': self.create_client(Trigger, '/dog_follower/enable'),
            'disable': self.create_client(Trigger, '/dog_follower/disable'),
            'reset': self.create_client(Trigger, '/dog_follower/reset'),
        }

        # Dog Follower status tracking and process management
        self.latest_dog_follower_status = 'idle'
        self.detection_enabled = False  # Track if we should use annotated feed
        self.dog_follower_process = None  # Subprocess for dynamic spawning
        self.dog_follower_sub = self.create_subscription(
            String,
            '/dog_follower/status',
            self.dog_follower_status_callback,
            10
        )

        # Shoe Follower service clients
        self.shoe_follower_clients = {
            'detection_enable': self.create_client(Trigger, '/shoe_follower/detection_enable'),
            'detection_disable': self.create_client(Trigger, '/shoe_follower/detection_disable'),
            'enable': self.create_client(Trigger, '/shoe_follower/enable'),
            'disable': self.create_client(Trigger, '/shoe_follower/disable'),
            'reset': self.create_client(Trigger, '/shoe_follower/reset'),
        }

        # Shoe Follower status tracking and process management
        self.latest_shoe_follower_status = 'idle'
        self.shoe_detection_enabled = False
        self.shoe_follower_process = None  # Subprocess for dynamic spawning
        self.shoe_follower_sub = self.create_subscription(
            String,
            '/shoe_follower/status',
            self.shoe_follower_status_callback,
            10
        )

        # Camera integration (optimized for low-latency teleop)
        self.bridge = CvBridge()
        self.latest_image_base64 = None
        self.last_img_time = 0
        self.fps_limit = 5.0  # Reduced from 10 to minimize bandwidth contention
        self.interval = 1.0 / self.fps_limit
        self.stream_width = 320  # Downsample for faster transmission
        self.stream_height = 240

        # Track which feed we're using
        self.using_annotated_feed = False
        self.last_annotated_time = 0

        # Subscribe to annotated camera feed (with bounding boxes) - preferred when detection is on
        self.annotated_image_sub = self.create_subscription(
            Image,
            '/dog_follower/annotated_image',
            self.annotated_image_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Subscribe to raw camera RGB feed - fallback when detection is off
        self.raw_image_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.raw_image_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Power status monitoring (Pi 5 USB-C PD)
        self.power_status = None
        self.power_timer = self.create_timer(3.0, self.check_power_status)
        self.check_power_status()  # Initial check

        # WiFi signal strength monitoring
        self.wifi_signal = None
        self.wifi_timer = self.create_timer(3.0, self.check_wifi_signal)
        self.check_wifi_signal()  # Initial check

        # Teleop speed control publisher
        self.teleop_speed_pub = self.create_publisher(Float32, '/teleop/speed_scale', 10)
        self.teleop_speed = 1.0  # Track current speed for dashboard sync
        self.pending_teleop_speed = None  # Thread-safe queue for cross-thread publishing

        # Teleop velocity command publisher (for web-based driving)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pending_cmd_vel = None  # Thread-safe queue for cross-thread publishing
        self.teleop_active = False  # Track if web teleop is actively sending commands

        # Timer to publish pending teleop speed from main thread (thread-safe)
        self.create_timer(0.05, self.publish_pending_teleop)  # 20Hz check

        # Get package directory for serving static files
        try:
            self.package_dir = Path(get_package_share_directory('gnss_web_dashboard'))
            self.static_dir = self.package_dir / 'static'
        except Exception:
            # Fallback for development
            self.package_dir = Path(__file__).parent.parent
            self.static_dir = self.package_dir / 'static'

        self.get_logger().info(f"Rover1 Web Dashboard started. Package dir: {self.package_dir}")
        
    def health_callback(self, msg: GnssHealth):
        """Process incoming GNSS health messages (throttled to reduce WebSocket traffic)"""
        import math
        with self.health_lock:
            # Convert ROS message to JSON-serializable dict
            self.latest_health = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'latitude': msg.latitude if not math.isnan(msg.latitude) else None,
                'longitude': msg.longitude if not math.isnan(msg.longitude) else None,
                'altitude': msg.altitude if not math.isnan(msg.altitude) else None,
                'sat_visible': msg.sat_visible,
                'sat_used': msg.sat_used,
                'ntrip_connected': msg.ntrip_connected,
                'rtcm_msgs_total': msg.rtcm_msgs_total,
                'rtcm_msgs_per_sec': msg.rtcm_msgs_per_sec,
                'rtcm_bytes_per_sec': msg.rtcm_bytes_per_sec,
                'corr_age_s': msg.corr_age_s,
                'rtk_state': msg.rtk_state,
                'h_acc_m': msg.h_acc_m if msg.h_acc_m > 0 else None,
                'v_acc_m': msg.v_acc_m if msg.v_acc_m > 0 else None,
                'dgps_id': msg.dgps_id,
                'battery_voltage': msg.battery_voltage if msg.battery_voltage > 0 else None
            }

        # Throttle broadcasts to 2Hz (health data doesn't need 10Hz updates)
        now = time.time()
        if now - self.last_health_broadcast >= self.health_broadcast_interval:
            self.last_health_broadcast = now
            if self.ws_clients:
                self.broadcast_data()

    def patrol_status_callback(self, msg: PatrolStatus):
        """Process incoming patrol status messages (throttled to reduce WebSocket traffic)"""
        with self.health_lock:
            self.latest_patrol_status = {
                'state': msg.state,
                'path_name': msg.path_name,
                'current_waypoint': msg.current_waypoint,
                'total_waypoints': msg.total_waypoints,
                'current_loop': msg.current_loop,
                'total_loops': msg.total_loops,
                'speed_percent': msg.speed_percent,
                'error_message': msg.error_message,
                'recorded_waypoint_count': msg.recorded_waypoint_count,
                'gps_fix_available': msg.gps_fix_available,
                'gps_waypoint_count': msg.gps_waypoint_count,
            }

        # Throttle broadcasts to 5Hz (patrol status updates are less critical than teleop)
        now = time.time()
        if now - self.last_patrol_broadcast >= self.patrol_broadcast_interval:
            self.last_patrol_broadcast = now
            if self.ws_clients:
                self.broadcast_data()

    def dog_follower_status_callback(self, msg: String):
        """Process incoming dog follower status messages"""
        with self.health_lock:
            self.latest_dog_follower_status = msg.data

        # Broadcast to all connected WebSocket clients
        if self.ws_clients:
            self.broadcast_data()

    def shoe_follower_status_callback(self, msg: String):
        """Process incoming shoe follower status messages"""
        with self.health_lock:
            self.latest_shoe_follower_status = msg.data

        # Broadcast to all connected WebSocket clients
        if self.ws_clients:
            self.broadcast_data()

    def annotated_image_callback(self, msg: Image):
        """Process annotated camera images (with bounding boxes) - preferred feed."""
        self.last_annotated_time = time.time()
        self.using_annotated_feed = True
        self._process_image(msg)

    def raw_image_callback(self, msg: Image):
        """Process raw camera images - fallback when detection is off."""
        # Only use raw feed if we haven't received annotated feed recently (1 second)
        if self.last_annotated_time > 0 and (time.time() - self.last_annotated_time) < 1.0:
            return  # Annotated feed is active, skip raw
        self.using_annotated_feed = False
        self._process_image(msg)

    def _process_image(self, msg: Image):
        """Common image processing for both feeds (optimized for low-latency teleop)."""
        now = time.time()
        if now - self.last_img_time < self.interval:
            return

        try:
            # Convert ROS Image to OpenCV BGR
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Downsample for faster transmission (320x240 ~10KB vs 640x480 ~40KB)
            cv_img = cv2.resize(cv_img, (self.stream_width, self.stream_height),
                               interpolation=cv2.INTER_AREA)

            # Encode as JPEG with aggressive compression for bandwidth
            # Quality 35 keeps images recognizable while minimizing size
            _, buffer = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 35])

            # Convert to Base64 string
            base64_img = base64.b64encode(buffer).decode('utf-8')

            with self.health_lock:
                self.latest_image_base64 = base64_img
                self.last_img_time = now

            # Broadcast immediately on image update if health is already there
            if self.ws_clients:
                self.broadcast_data()

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def check_power_status(self):
        """Check Pi power/throttle status via vcgencmd"""
        try:
            result = subprocess.run(
                ['vcgencmd', 'get_throttled'],
                capture_output=True,
                text=True,
                timeout=2.0
            )
            if result.returncode == 0:
                # Output format: "throttled=0x0" or "throttled=0x50005"
                output = result.stdout.strip()
                if '=' in output:
                    hex_val = output.split('=')[1]
                    throttle_bits = int(hex_val, 16)

                    # Decode throttle bits
                    # Bit 0: Under-voltage detected
                    # Bit 1: Arm frequency capped
                    # Bit 2: Currently throttled
                    # Bit 3: Soft temperature limit active
                    under_voltage = bool(throttle_bits & 0x1)
                    freq_capped = bool(throttle_bits & 0x2)
                    throttled = bool(throttle_bits & 0x4)
                    soft_temp_limit = bool(throttle_bits & 0x8)

                    # Bits 16-19: same but "has occurred"
                    under_voltage_occurred = bool(throttle_bits & 0x10000)
                    freq_capped_occurred = bool(throttle_bits & 0x20000)
                    throttled_occurred = bool(throttle_bits & 0x40000)
                    soft_temp_occurred = bool(throttle_bits & 0x80000)

                    # Determine overall status
                    if under_voltage or throttled:
                        status = 'RESTRICTED'
                        status_color = 'critical'
                    elif freq_capped or soft_temp_limit:
                        status = 'THROTTLED'
                        status_color = 'warning'
                    elif under_voltage_occurred or throttled_occurred:
                        status = 'RECOVERED'
                        status_color = 'warning'
                    else:
                        status = 'OK'
                        status_color = 'good'

                    self.power_status = {
                        'status': status,
                        'status_color': status_color,
                        'under_voltage': under_voltage,
                        'throttled': throttled,
                        'freq_capped': freq_capped,
                        'soft_temp_limit': soft_temp_limit,
                        'raw_hex': hex_val
                    }
        except Exception as e:
            self.get_logger().warn(f"Power status check failed: {e}")
            self.power_status = {'status': 'UNKNOWN', 'status_color': 'unknown'}

    def check_wifi_signal(self):
        """Check WiFi signal strength via nmcli"""
        try:
            result = subprocess.run(
                ['nmcli', '-t', '-f', 'IN-USE,SIGNAL', 'dev', 'wifi', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line.startswith('*:'):
                        # Line format: "*:75" where 75 is signal strength
                        self.wifi_signal = int(line.split(':')[1])
                        return
            self.wifi_signal = None
        except Exception as e:
            self.get_logger().debug(f"WiFi signal check failed: {e}")
            self.wifi_signal = None

    def broadcast_data(self):
        """Prepare and broadcast combined health + image + patrol data"""
        # Prepare combined data
        with self.health_lock:
            payload = {}
            if self.latest_health:
                payload.update(self.latest_health)
            if self.latest_image_base64:
                payload['image'] = self.latest_image_base64
            if self.power_status:
                payload['power_status'] = self.power_status
            if self.wifi_signal is not None:
                payload['wifi_signal'] = self.wifi_signal
            if self.latest_patrol_status:
                payload['patrol'] = self.latest_patrol_status
            # Include teleop speed for dashboard sync
            payload['teleop_speed'] = self.teleop_speed
            # Include dog follower status and detection state
            payload['dog_follower_status'] = self.latest_dog_follower_status
            payload['detection_enabled'] = self.detection_enabled
            payload['using_ai_feed'] = self.using_annotated_feed
            payload['dog_follower_running'] = self.is_follower_running('dog')
            # Shoe Follower status
            payload['shoe_follower_status'] = self.latest_shoe_follower_status
            payload['shoe_detection_enabled'] = self.shoe_detection_enabled
            payload['shoe_follower_running'] = self.is_follower_running('shoe')

        if not payload:
            return

        # Broadcast via the WebSocket thread's event loop
        if self.ws_loop:
            asyncio.run_coroutine_threadsafe(
                self.send_payload(payload),
                self.ws_loop
            )
    
    async def send_payload(self, data):
        """Send data to all connected WebSocket clients"""
        if not self.ws_clients:
            return
            
        message = json.dumps(data)
        disconnected = set()
        
        for client in self.ws_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
        
        # Clean up disconnected clients
        self.ws_clients -= disconnected
    
    async def handle_websocket(self, websocket, path):
        """Handle new WebSocket connections"""
        self.ws_clients.add(websocket)
        self.get_logger().info(f"New client connected. Total clients: {len(self.ws_clients)}")

        try:
            # Send current data immediately
            self.broadcast_data()

            # Handle incoming messages (commands)
            async for message in websocket:
                try:
                    cmd = json.loads(message)
                    response = await self.handle_command(cmd)
                    await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({'error': 'Invalid JSON'}))
                except Exception as e:
                    await websocket.send(json.dumps({'error': str(e)}))

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(f"Client disconnected. Total clients: {len(self.ws_clients)}")

    async def handle_command(self, cmd):
        """Handle commands from the dashboard"""
        action = cmd.get('action')
        auto_mode = cmd.get('auto_mode', self.current_auto_mode)

        if action == 'list_paths':
            return await self.call_list_paths()
        elif action == 'start_recording':
            return await self.call_trigger_service('start_recording')
        elif action == 'stop_recording':
            return await self.call_trigger_service('stop_recording')
        elif action == 'save_path':
            return await self.call_save_path(cmd.get('path_name', 'unnamed'))
        elif action == 'discard_recording':
            return await self.call_trigger_service('discard_recording')
        elif action == 'start_patrol':
            # Store the mode for subsequent pause/resume/stop
            self.current_auto_mode = auto_mode
            return await self.call_start_patrol(
                auto_mode,
                cmd.get('path_name'),
                cmd.get('loop_count', 0),
                cmd.get('reverse_mode', False),
                cmd.get('speed_percent', 1.0)
            )
        elif action == 'stop_patrol':
            return await self.call_trigger_service(f'{auto_mode}_stop')
        elif action == 'pause_patrol':
            return await self.call_trigger_service(f'{auto_mode}_pause')
        elif action == 'resume_patrol':
            return await self.call_trigger_service(f'{auto_mode}_resume')
        elif action == 'get_map_image':
            return self.get_map_image(cmd.get('path_name'))
        elif action == 'set_teleop_speed':
            return self.set_teleop_speed(cmd.get('speed_percent', 1.0))
        elif action == 'get_pointcloud':
            return self.get_pointcloud(cmd.get('path_name'))
        elif action == 'teleop':
            return self.set_teleop_velocity(
                cmd.get('linear_x', 0.0),
                cmd.get('linear_y', 0.0),
                cmd.get('angular_z', 0.0)
            )
        elif action == 'shutdown':
            return self.system_shutdown()
        elif action == 'reboot':
            return self.system_reboot()
        elif action == 'emergency_stop':
            return self.emergency_stop()
        # Dog Follower commands (with dynamic node spawning/killing)
        elif action == 'detection_enable':
            # Spawn node if not running, then enable detection
            spawn_result = await self.spawn_follower_node('dog')
            if not spawn_result.get('success'):
                return spawn_result
            result = await self.call_dog_follower_service('detection_enable')
            if result.get('success'):
                self.detection_enabled = True
            return result
        elif action == 'detection_disable':
            # Kill node entirely (true mothball - frees all CPU/memory)
            result = self.kill_follower_node('dog')
            self.detection_enabled = False
            return result
        elif action == 'dog_follow_enable':
            # Spawn node if not running, then enable following
            spawn_result = await self.spawn_follower_node('dog')
            if not spawn_result.get('success'):
                return spawn_result
            result = await self.call_dog_follower_service('enable')
            if result.get('success'):
                self.detection_enabled = True  # Following auto-enables detection
            return result
        elif action == 'dog_follow_disable':
            return await self.call_dog_follower_service('disable')
        elif action == 'dog_follow_reset':
            # Reset and kill node (full mothball)
            if self.is_follower_running('dog'):
                await self.call_dog_follower_service('reset')
            result = self.kill_follower_node('dog')
            self.detection_enabled = False
            return result
        # Shoe Follower commands (with dynamic node spawning/killing)
        elif action == 'shoe_detection_enable':
            # Spawn node if not running, then enable detection
            spawn_result = await self.spawn_follower_node('shoe')
            if not spawn_result.get('success'):
                return spawn_result
            result = await self.call_shoe_follower_service('detection_enable')
            if result.get('success'):
                self.shoe_detection_enabled = True
            return result
        elif action == 'shoe_detection_disable':
            # Kill node entirely (true mothball - frees all CPU/memory)
            result = self.kill_follower_node('shoe')
            self.shoe_detection_enabled = False
            return result
        elif action == 'shoe_follow_enable':
            # Spawn node if not running, then enable following
            spawn_result = await self.spawn_follower_node('shoe')
            if not spawn_result.get('success'):
                return spawn_result
            result = await self.call_shoe_follower_service('enable')
            if result.get('success'):
                self.shoe_detection_enabled = True  # Following auto-enables detection
            return result
        elif action == 'shoe_follow_disable':
            return await self.call_shoe_follower_service('disable')
        elif action == 'shoe_follow_reset':
            # Reset and kill node (full mothball)
            if self.is_follower_running('shoe'):
                await self.call_shoe_follower_service('reset')
            result = self.kill_follower_node('shoe')
            self.shoe_detection_enabled = False
            return result
        else:
            return {'success': False, 'message': f'Unknown action: {action}'}

    async def call_trigger_service(self, service_name):
        """Call a Trigger service and return the result"""
        client = self.patrol_clients.get(service_name)
        if not client:
            return {'success': False, 'message': f'Service {service_name} not found'}

        if not client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': f'Service {service_name} not available'}

        request = Trigger.Request()
        future = client.call_async(request)

        # Wait for result in a non-blocking way
        while not future.done():
            await asyncio.sleep(0.1)

        result = future.result()
        return {'success': result.success, 'message': result.message}

    async def call_dog_follower_service(self, service_name):
        """Call a dog follower Trigger service and return the result"""
        client = self.dog_follower_clients.get(service_name)
        if not client:
            return {'success': False, 'message': f'Dog follower service {service_name} not found'}

        if not client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': 'Dog follower node not running'}

        request = Trigger.Request()
        future = client.call_async(request)

        # Wait for result in a non-blocking way
        while not future.done():
            await asyncio.sleep(0.1)

        result = future.result()
        return {'success': result.success, 'message': result.message}

    async def call_shoe_follower_service(self, service_name):
        """Call a shoe follower Trigger service and return the result"""
        client = self.shoe_follower_clients.get(service_name)
        if not client:
            return {'success': False, 'message': f'Shoe follower service {service_name} not found'}

        if not client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': 'Shoe follower node not running'}

        request = Trigger.Request()
        future = client.call_async(request)

        # Wait for result in a non-blocking way
        while not future.done():
            await asyncio.sleep(0.1)

        result = future.result()
        return {'success': result.success, 'message': result.message}

    def is_follower_running(self, follower_type):
        """Check if a follower node process is running.

        Args:
            follower_type: 'dog' or 'shoe'

        Returns:
            bool: True if the process is running
        """
        if follower_type == 'dog':
            proc = self.dog_follower_process
        else:
            proc = self.shoe_follower_process

        return proc is not None and proc.poll() is None

    async def spawn_follower_node(self, follower_type):
        """Spawn a follower node via subprocess.

        Args:
            follower_type: 'dog' or 'shoe'

        Returns:
            dict with success and message
        """
        if self.is_follower_running(follower_type):
            return {'success': True, 'message': f'{follower_type.title()} follower already running'}

        self.get_logger().info(f'Spawning {follower_type} follower node...')

        try:
            # Build the ros2 run command
            if follower_type == 'dog':
                cmd = [
                    'ros2', 'run', 'rover1_vision', 'dog_follower',
                    '--ros-args',
                    '-p', 'model_path:=/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef',
                    '-p', 'confidence_threshold:=0.5',
                    '-p', 'linear_speed:=0.2',
                    '-p', 'angular_speed:=0.5',
                ]
                service_client = self.dog_follower_clients['detection_enable']
            else:
                cmd = [
                    'ros2', 'run', 'rover1_vision', 'shoe_follower',
                    '--ros-args',
                    '-p', 'model_path:=/home/andrewmeckley/ros2_ws/src/rover1/models/yolov8s.hef',
                    '-p', 'confidence_threshold:=0.5',
                    '-p', 'linear_speed:=0.2',
                    '-p', 'angular_speed:=0.5',
                ]
                service_client = self.shoe_follower_clients['detection_enable']

            # Spawn the process
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True  # Detach from parent process group
            )

            if follower_type == 'dog':
                self.dog_follower_process = proc
            else:
                self.shoe_follower_process = proc

            # Wait for service to become available (max 10 seconds)
            self.get_logger().info(f'Waiting for {follower_type} follower services...')
            for _ in range(100):  # 100 * 0.1s = 10s max
                if service_client.wait_for_service(timeout_sec=0.1):
                    self.get_logger().info(f'{follower_type.title()} follower node ready!')
                    return {'success': True, 'message': f'{follower_type.title()} follower started'}
                await asyncio.sleep(0.1)

            # Timeout - node may have crashed
            self.get_logger().error(f'{follower_type.title()} follower failed to start in time')
            self.kill_follower_node(follower_type)
            return {'success': False, 'message': f'{follower_type.title()} follower failed to start'}

        except Exception as e:
            self.get_logger().error(f'Failed to spawn {follower_type} follower: {e}')
            return {'success': False, 'message': str(e)}

    def kill_follower_node(self, follower_type):
        """Kill a follower node process.

        Args:
            follower_type: 'dog' or 'shoe'

        Returns:
            dict with success and message
        """
        import signal

        if follower_type == 'dog':
            proc = self.dog_follower_process
        else:
            proc = self.shoe_follower_process

        if proc is None:
            return {'success': True, 'message': f'{follower_type.title()} follower not running'}

        if proc.poll() is not None:
            # Process already exited
            if follower_type == 'dog':
                self.dog_follower_process = None
            else:
                self.shoe_follower_process = None
            return {'success': True, 'message': f'{follower_type.title()} follower already stopped'}

        self.get_logger().info(f'Killing {follower_type} follower node...')

        try:
            # Send SIGTERM for graceful shutdown
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't respond
                proc.kill()
                proc.wait(timeout=1.0)

            if follower_type == 'dog':
                self.dog_follower_process = None
                self.detection_enabled = False
                self.latest_dog_follower_status = 'idle'
            else:
                self.shoe_follower_process = None
                self.shoe_detection_enabled = False
                self.latest_shoe_follower_status = 'idle'

            self.get_logger().info(f'{follower_type.title()} follower stopped')
            return {'success': True, 'message': f'{follower_type.title()} follower stopped'}

        except Exception as e:
            self.get_logger().error(f'Failed to kill {follower_type} follower: {e}')
            return {'success': False, 'message': str(e)}

    async def call_list_paths(self):
        """Call list_paths service and return the result"""
        client = self.patrol_clients.get('list_paths')
        if not client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': 'list_paths service not available', 'paths': []}

        request = ListPaths.Request()
        future = client.call_async(request)

        while not future.done():
            await asyncio.sleep(0.1)

        result = future.result()
        paths = []
        for i, name in enumerate(result.path_names):
            paths.append({
                'name': name,
                'waypoint_count': result.waypoint_counts[i] if i < len(result.waypoint_counts) else 0,
                'recorded_date': result.recorded_dates[i] if i < len(result.recorded_dates) else '',
                'gps_waypoint_count': result.gps_waypoint_counts[i] if i < len(result.gps_waypoint_counts) else 0,
                'has_gps': result.has_gps[i] if i < len(result.has_gps) else False,
            })

        return {'success': True, 'paths': paths}

    async def call_save_path(self, path_name):
        """Call save_path service"""
        client = self.patrol_clients.get('save_path')
        if not client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': 'save_path service not available'}

        request = SavePath.Request()
        request.path_name = path_name

        future = client.call_async(request)
        while not future.done():
            await asyncio.sleep(0.1)

        result = future.result()
        return {'success': result.success, 'message': result.message}

    async def call_start_patrol(self, auto_mode, path_name, loop_count, reverse_mode, speed_percent):
        """Call start_patrol service based on selected auto mode"""
        # Map mode to service client key
        service_key = f'{auto_mode}_start'
        client = self.patrol_clients.get(service_key)

        if not client:
            return {'success': False, 'message': f'Unknown auto mode: {auto_mode}'}

        if not client.wait_for_service(timeout_sec=1.0):
            mode_names = {'odometry': 'Simple Waypoint Follower', 'gps': 'GPS Waypoint Follower', 'nav2': 'Nav2 Patrol Manager'}
            mode_name = mode_names.get(auto_mode, auto_mode)
            return {'success': False, 'message': f'{mode_name} service not available. Is the node running?'}

        request = StartPatrol.Request()
        request.path_name = path_name
        request.loop_count = int(loop_count)
        request.reverse_mode = bool(reverse_mode)
        request.speed_percent = float(speed_percent)

        future = client.call_async(request)
        while not future.done():
            await asyncio.sleep(0.1)

        result = future.result()
        mode_label = {'odometry': '[Odom]', 'gps': '[GPS]', 'nav2': '[Nav2]'}.get(auto_mode, '')
        return {'success': result.success, 'message': f'{mode_label} {result.message}'}

    def get_map_image(self, path_name):
        """Get base64-encoded map image for a path"""
        if not path_name:
            return {'success': False, 'message': 'No path name provided'}

        # Sanitize filename
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        img_path = self.paths_dir / f'{safe_name}_map.png'

        if not img_path.exists():
            return {'success': False, 'message': 'Map image not found'}

        try:
            with open(img_path, 'rb') as f:
                img_data = base64.b64encode(f.read()).decode('utf-8')
            return {'success': True, 'image': img_data}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def get_pointcloud(self, path_name):
        """Get base64-encoded PLY point cloud for a path"""
        if not path_name:
            return {'success': False, 'message': 'No path name provided'}

        # Sanitize filename
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        ply_path = self.paths_dir / f'{safe_name}_cloud.ply'

        if not ply_path.exists():
            return {'success': False, 'message': 'Point cloud not found. Save a path first to generate.'}

        try:
            with open(ply_path, 'rb') as f:
                ply_data = base64.b64encode(f.read()).decode('utf-8')

            # Get file size for info
            file_size_kb = ply_path.stat().st_size / 1024

            return {
                'success': True,
                'pointcloud': ply_data,
                'file_size_kb': round(file_size_kb, 1)
            }
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def publish_pending_teleop(self):
        """Publish pending teleop commands from main thread (thread-safe).

        Called by timer at 20Hz. This ensures ROS publish happens on the
        main thread where the executor runs, avoiding cross-thread issues.
        """
        # Publish teleop speed if pending
        if self.pending_teleop_speed is not None:
            speed = self.pending_teleop_speed
            self.pending_teleop_speed = None  # Clear before publish

            msg = Float32()
            msg.data = speed
            self.teleop_speed_pub.publish(msg)
            self.get_logger().info(f'Teleop speed published: {int(speed * 100)}%')

        # Publish cmd_vel if pending
        if self.pending_cmd_vel is not None:
            twist = self.pending_cmd_vel
            self.pending_cmd_vel = None  # Clear before publish
            self.cmd_vel_pub.publish(twist)

    def set_teleop_speed(self, speed_percent):
        """Set teleop speed scale (queued for thread-safe publishing).

        This is called from the WebSocket asyncio thread. We queue the
        speed value for the main thread timer to publish, avoiding
        cross-thread ROS publisher issues.
        """
        # Clamp to valid range [0.1, 1.0]
        speed = max(0.1, min(1.0, float(speed_percent)))
        self.teleop_speed = speed
        self.pending_teleop_speed = speed  # Queue for main thread to publish

        return {'success': True, 'message': f'Teleop speed set to {int(speed * 100)}%'}

    def set_teleop_velocity(self, linear_x, linear_y, angular_z):
        """Set teleop velocity command (queued for thread-safe publishing).

        This is called from the WebSocket asyncio thread for web-based driving.
        Velocities are scaled by the current teleop_speed setting.

        Args:
            linear_x: Forward/backward velocity (-1.0 to 1.0)
            linear_y: Strafe left/right velocity (-1.0 to 1.0)
            angular_z: Rotation velocity (-1.0 to 1.0)
        """
        # Base velocities (m/s and rad/s)
        MAX_LINEAR_VEL = 0.5   # Max linear velocity in m/s
        MAX_ANGULAR_VEL = 1.5  # Max angular velocity in rad/s

        # Scale by teleop speed setting
        scale = self.teleop_speed

        # Create Twist message
        twist = Twist()
        twist.linear.x = float(linear_x) * MAX_LINEAR_VEL * scale
        twist.linear.y = float(linear_y) * MAX_LINEAR_VEL * scale
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular_z) * MAX_ANGULAR_VEL * scale

        # Queue for main thread to publish
        self.pending_cmd_vel = twist

        return {'success': True}

    def system_shutdown(self):
        """Initiate system shutdown."""
        self.get_logger().warn('Shutdown requested via web dashboard!')
        try:
            # Use subprocess to call shutdown with a small delay
            # This allows the response to be sent before shutdown begins
            subprocess.Popen(
                ['sudo', 'shutdown', '-h', '+0'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return {'success': True, 'message': 'Shutdown initiated. Goodbye!'}
        except Exception as e:
            self.get_logger().error(f'Shutdown failed: {e}')
            return {'success': False, 'message': f'Shutdown failed: {e}'}

    def system_reboot(self):
        """Initiate system reboot."""
        self.get_logger().warn('Reboot requested via web dashboard!')
        try:
            subprocess.Popen(
                ['sudo', 'reboot'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return {'success': True, 'message': 'Reboot initiated. See you soon!'}
        except Exception as e:
            self.get_logger().error(f'Reboot failed: {e}')
            return {'success': False, 'message': f'Reboot failed: {e}'}

    def emergency_stop(self):
        """Emergency stop - send zero velocity command immediately."""
        self.get_logger().warn('EMERGENCY STOP activated via web dashboard!')

        # Send zero velocity command
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # Publish multiple times to ensure it gets through
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)

        self.get_logger().info('Emergency stop: Zero velocity published')
        return {'success': True, 'message': 'Emergency stop activated'}


class CustomHTTPHandler(SimpleHTTPRequestHandler):
    """Custom HTTP handler that serves from the static directory"""
    
    def __init__(self, *args, static_dir=None, **kwargs):
        self.static_dir = static_dir
        super().__init__(*args, **kwargs)
    
    def translate_path(self, path):
        """Translate URL path to filesystem path"""
        if path == '/':
            return str(self.static_dir / 'index.html')
        else:
            # Remove leading slash and serve from static dir
            relative_path = path.lstrip('/')
            return str(self.static_dir / relative_path)


def run_http_server(static_dir, port=8080):
    """Run HTTP server for static files"""
    os.chdir(static_dir)

    handler = lambda *args, **kwargs: CustomHTTPHandler(*args, static_dir=static_dir, **kwargs)
    server = HTTPServer(('0.0.0.0', port), handler)

    print(f"HTTP server starting on port {port}")
    server.serve_forever()


def run_websocket_server(dashboard_node, ready_event):
    """Run WebSocket server for real-time data"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    server = websockets.serve(
        dashboard_node.handle_websocket,
        '0.0.0.0',
        8081,
        ping_interval=20,
        ping_timeout=10
    )

    dashboard_node.ws_loop = loop
    loop.run_until_complete(server)

    # Signal that WebSocket is ready before entering run_forever
    print("WebSocket server ready on port 8081")
    ready_event.set()

    loop.run_forever()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    # Create dashboard node
    dashboard = Rover1WebDashboard()

    # Event to synchronize startup order
    ws_ready = threading.Event()

    # Start WebSocket server first (must be ready before HTTP serves pages)
    ws_thread = threading.Thread(
        target=run_websocket_server,
        args=(dashboard, ws_ready),
        daemon=True
    )
    ws_thread.start()

    # Wait for WebSocket to be ready (max 10 seconds)
    if not ws_ready.wait(timeout=10.0):
        dashboard.get_logger().error("WebSocket server failed to start in time!")

    # Now start HTTP server (clients can connect to WebSocket immediately)
    http_thread = threading.Thread(
        target=run_http_server,
        args=(dashboard.static_dir, 8080),
        daemon=True
    )
    http_thread.start()

    dashboard.get_logger().info("Rover1 Web Dashboard ready at http://<rover-ip>:8080")
    
    try:
        # Run ROS node
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()