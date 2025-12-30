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
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
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
            'start_patrol': self.create_client(StartPatrol, '/patrol/start_patrol'),
            'stop_patrol': self.create_client(Trigger, '/patrol/stop_patrol'),
            'pause_patrol': self.create_client(Trigger, '/patrol/pause_patrol'),
            'resume_patrol': self.create_client(Trigger, '/patrol/resume_patrol'),
        }

        # Camera integration
        self.bridge = CvBridge()
        self.latest_image_base64 = None
        self.last_img_time = 0
        self.fps_limit = 10.0
        self.interval = 1.0 / self.fps_limit

        # Subscribe to camera RGB feed
        # Topic as identified from ascamera_node defaults + namespace
        self.image_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Power status monitoring (Pi 5 USB-C PD)
        self.power_status = None
        self.power_timer = self.create_timer(3.0, self.check_power_status)
        self.check_power_status()  # Initial check

        # Teleop speed control publisher
        self.teleop_speed_pub = self.create_publisher(Float32, '/teleop/speed_scale', 10)
        self.teleop_speed = 1.0  # Track current speed for dashboard sync

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
        """Process incoming GNSS health messages"""
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
            
        # Broadcast to all connected WebSocket clients
        if self.ws_clients:
            self.broadcast_data()

    def patrol_status_callback(self, msg: PatrolStatus):
        """Process incoming patrol status messages"""
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
            }

        # Broadcast to all connected WebSocket clients
        if self.ws_clients:
            self.broadcast_data()

    def image_callback(self, msg: Image):
        """Process incoming camera images and encode for web"""
        now = time.time()
        if now - self.last_img_time < self.interval:
            return
            
        try:
            # Convert ROS Image to OpenCV BGR
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Downscale for performance if needed (already 640x480 usually)
            # cv_img = cv2.resize(cv_img, (320, 240)) 
            
            # Encode as JPEG
            # Quality 40-60 is a good balance for bandwidth vs clarity
            _, buffer = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 50])
            
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
            if self.latest_patrol_status:
                payload['patrol'] = self.latest_patrol_status
            # Include teleop speed for dashboard sync
            payload['teleop_speed'] = self.teleop_speed

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
            return await self.call_start_patrol(
                cmd.get('path_name'),
                cmd.get('loop_count', 0),
                cmd.get('reverse_mode', False),
                cmd.get('speed_percent', 1.0)
            )
        elif action == 'stop_patrol':
            return await self.call_trigger_service('stop_patrol')
        elif action == 'pause_patrol':
            return await self.call_trigger_service('pause_patrol')
        elif action == 'resume_patrol':
            return await self.call_trigger_service('resume_patrol')
        elif action == 'get_map_image':
            return self.get_map_image(cmd.get('path_name'))
        elif action == 'set_teleop_speed':
            return self.set_teleop_speed(cmd.get('speed_percent', 1.0))
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

    async def call_start_patrol(self, path_name, loop_count, reverse_mode, speed_percent):
        """Call start_patrol service"""
        client = self.patrol_clients.get('start_patrol')
        if not client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'message': 'start_patrol service not available'}

        request = StartPatrol.Request()
        request.path_name = path_name
        request.loop_count = loop_count
        request.reverse_mode = reverse_mode
        request.speed_percent = speed_percent

        future = client.call_async(request)
        while not future.done():
            await asyncio.sleep(0.1)

        result = future.result()
        return {'success': result.success, 'message': result.message}

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

    def set_teleop_speed(self, speed_percent):
        """Set teleop speed scale and publish to ROS topic"""
        # Clamp to valid range [0.1, 1.0]
        speed = max(0.1, min(1.0, float(speed_percent)))
        self.teleop_speed = speed

        # Publish to ROS topic
        msg = Float32()
        msg.data = speed
        self.teleop_speed_pub.publish(msg)

        self.get_logger().info(f'Teleop speed set to {int(speed * 100)}%')
        return {'success': True, 'message': f'Teleop speed set to {int(speed * 100)}%'}


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


def run_websocket_server(dashboard_node):
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
    
    print("WebSocket server starting on port 8081")
    dashboard_node.ws_loop = loop
    loop.run_until_complete(server)
    loop.run_forever()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    # Create dashboard node
    dashboard = Rover1WebDashboard()
    
    # Start HTTP server in separate thread
    http_thread = threading.Thread(
        target=run_http_server,
        args=(dashboard.static_dir, 8080),
        daemon=True
    )
    http_thread.start()
    
    # Start WebSocket server in separate thread  
    ws_thread = threading.Thread(
        target=run_websocket_server,
        args=(dashboard,),
        daemon=True
    )
    ws_thread.start()
    
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