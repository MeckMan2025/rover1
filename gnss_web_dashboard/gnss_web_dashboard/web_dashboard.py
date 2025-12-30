#!/usr/bin/env python3
"""
GNSS Web Dashboard Server

Simple web server that subscribes to /gnss/health and serves real-time
GPS status via a clean web interface accessible from any device.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
import asyncio
import websockets
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os
from pathlib import Path
import subprocess

from gnss_health_monitor.msg import GnssHealth
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import time
from ament_index_python.packages import get_package_share_directory


class GnssWebDashboard(Node):
    """Web dashboard server for GNSS health monitoring"""
    
    def __init__(self):
        super().__init__('gnss_web_dashboard')
        
        # Latest GNSS health data
        self.latest_health = None
        self.health_lock = threading.Lock()
        
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

        # Get package directory for serving static files
        try:
            self.package_dir = Path(get_package_share_directory('gnss_web_dashboard'))
            self.static_dir = self.package_dir / 'static'
        except Exception:
            # Fallback for development
            self.package_dir = Path(__file__).parent.parent
            self.static_dir = self.package_dir / 'static'
        
        self.get_logger().info(f"GNSS Web Dashboard started. Package dir: {self.package_dir}")
        
    def health_callback(self, msg: GnssHealth):
        """Process incoming GNSS health messages"""
        with self.health_lock:
            # Convert ROS message to JSON-serializable dict
            self.latest_health = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
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
        """Prepare and broadcast combined health + image data"""
        if self.latest_health is None:
            return

        # Prepare combined data
        with self.health_lock:
            payload = self.latest_health.copy()
            if self.latest_image_base64:
                payload['image'] = self.latest_image_base64
            if self.power_status:
                payload['power_status'] = self.power_status
            
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
            # Send current health data immediately
            if self.latest_health:
                await websocket.send(json.dumps(self.latest_health))
            
            # Keep connection alive
            async for message in websocket:
                pass  # Client doesn't need to send anything
                
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(f"Client disconnected. Total clients: {len(self.ws_clients)}")


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
    dashboard = GnssWebDashboard()
    
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
    
    dashboard.get_logger().info("GNSS Web Dashboard ready at http://<rover-ip>:8080")
    
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