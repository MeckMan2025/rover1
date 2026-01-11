#!/usr/bin/env python3
"""
Pack Robot Dashboard - Minimal web interface for rover2

Provides essential controls for pack robot operation:
- Web-based teleop (WASD + touch controls) - MANDATORY
- Person follower controls (enable/disable detection and following)
- Battery monitoring  
- Camera feed
- Emergency stop

Simplified version focused on human following functionality.
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

from std_srvs.srv import Trigger
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import base64
import time


class PackRobotDashboard(Node):
    """Minimal web dashboard for pack robot operation"""

    def __init__(self):
        super().__init__('pack_robot_dashboard')

        # Callback group for service calls
        self.cb_group = ReentrantCallbackGroup()

        # Latest data
        self.latest_battery_voltage = 0.0
        self.latest_battery_status = "UNKNOWN"
        self.latest_person_follower_status = "idle"
        self.latest_camera_image = None
        
        # Broadcast throttling (reduce WebSocket traffic)
        self.last_broadcast_time = 0.0
        self.broadcast_interval = 0.2  # 5Hz max broadcast rate
        
        # Connected WebSocket clients
        self.websocket_clients = set()
        
        # WebSocket event loop (will be set when WebSocket server starts)
        self.ws_loop = None
        
        # OpenCV bridge for image processing
        self.bridge = CvBridge()
        
        # Image QoS profile
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Subscriptions
        self.battery_voltage_sub = self.create_subscription(
            Float32, '/battery/voltage', self.battery_voltage_callback, 10
        )
        self.battery_status_sub = self.create_subscription(
            String, '/battery/status', self.battery_status_callback, 10
        )
        self.person_follower_status_sub = self.create_subscription(
            String, '/person_follower/status', self.person_follower_status_callback, 10
        )
        
        # Camera feed (low-latency for teleop)
        self.camera_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.camera_callback,
            qos_profile=self.image_qos
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pending_cmd_vel = None  # Thread-safe queue
        self.teleop_active = False
        
        # Service clients
        self.person_follower_enable_detection_client = self.create_client(
            Trigger, '/person_follower/enable_detection', callback_group=self.cb_group
        )
        self.person_follower_disable_detection_client = self.create_client(
            Trigger, '/person_follower/disable_detection', callback_group=self.cb_group
        )
        self.person_follower_enable_following_client = self.create_client(
            Trigger, '/person_follower/enable_following', callback_group=self.cb_group
        )
        self.person_follower_disable_following_client = self.create_client(
            Trigger, '/person_follower/disable_following', callback_group=self.cb_group
        )
        
        # Timer to publish pending commands from main thread (thread-safe)
        self.create_timer(0.05, self.publish_pending_commands)  # 20Hz
        
        # Start web servers
        self.start_web_servers()
        
        self.get_logger().info("Pack Robot Dashboard started - Web interface available at http://rover-ip:8080")
    
    def start_web_servers(self):
        """Start HTTP and WebSocket servers."""
        # Start HTTP server for static files
        self.http_thread = threading.Thread(target=self.run_http_server)
        self.http_thread.daemon = True
        self.http_thread.start()
        
        # Start WebSocket server
        self.websocket_thread = threading.Thread(target=self.run_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
    
    def run_http_server(self):
        """Run HTTP server for static files."""
        try:
            # Serve pack robot HTML and assets from static directory
            os.chdir(str(Path(__file__).parent.parent / 'static'))
            
            class CustomHandler(SimpleHTTPRequestHandler):
                def end_headers(self):
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Expires', '0')
                    super().end_headers()
            
            httpd = HTTPServer(('0.0.0.0', 8080), CustomHandler)
            httpd.serve_forever()
            
        except Exception as e:
            self.get_logger().error(f"HTTP server error: {e}")
    
    def run_websocket_server(self):
        """Run WebSocket server for real-time communication."""
        async def websocket_handler(websocket, path):
            self.websocket_clients.add(websocket)
            self.get_logger().info("Client connected to pack robot dashboard")
            try:
                async for message in websocket:
                    await self.handle_websocket_message(websocket, message)
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self.websocket_clients.discard(websocket)
                self.get_logger().info("Client disconnected from pack robot dashboard")
        
        async def start_server():
            async with websockets.serve(websocket_handler, "0.0.0.0", 8765):
                await asyncio.Future()  # Run forever
        
        try:
            # Create and store the WebSocket event loop
            self.ws_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.ws_loop)
            self.ws_loop.run_until_complete(start_server())
        except Exception as e:
            self.get_logger().error(f"WebSocket server error: {e}")
    
    async def handle_websocket_message(self, websocket, message):
        """Handle incoming WebSocket messages."""
        try:
            cmd = json.loads(message)
            action = cmd.get('action', '')
            
            if action == 'teleop':
                await self.handle_teleop_command(cmd)
            elif action == 'emergency_stop':
                await self.handle_emergency_stop()
            elif action == 'person_follower_enable_detection':
                await self.handle_person_follower_service('enable_detection')
            elif action == 'person_follower_disable_detection':
                await self.handle_person_follower_service('disable_detection')
            elif action == 'person_follower_enable_following':
                await self.handle_person_follower_service('enable_following')
            elif action == 'person_follower_disable_following':
                await self.handle_person_follower_service('disable_following')
            else:
                self.get_logger().warn(f"Unknown action: {action}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling WebSocket message: {e}")
    
    async def handle_teleop_command(self, cmd):
        """Handle teleop velocity commands."""
        try:
            twist = Twist()
            twist.linear.x = float(cmd.get('linear_x', 0.0))
            twist.linear.y = float(cmd.get('linear_y', 0.0))
            twist.angular.z = float(cmd.get('angular_z', 0.0))
            
            # Queue for thread-safe publishing
            self.pending_cmd_vel = twist
            self.teleop_active = (abs(twist.linear.x) > 0.01 or 
                                 abs(twist.linear.y) > 0.01 or 
                                 abs(twist.angular.z) > 0.01)
            
        except Exception as e:
            self.get_logger().error(f"Error in teleop command: {e}")
    
    async def handle_emergency_stop(self):
        """Handle emergency stop command."""
        try:
            # Send immediate stop command
            stop_twist = Twist()  # All zeros
            self.pending_cmd_vel = stop_twist
            self.teleop_active = False
            
            # Also disable person following for safety
            await self.handle_person_follower_service('disable_following')
            
            self.get_logger().warn("EMERGENCY STOP activated")
            
        except Exception as e:
            self.get_logger().error(f"Error in emergency stop: {e}")
    
    async def handle_person_follower_service(self, service_name):
        """Handle person follower service calls."""
        try:
            if service_name == 'enable_detection':
                client = self.person_follower_enable_detection_client
            elif service_name == 'disable_detection':
                client = self.person_follower_disable_detection_client
            elif service_name == 'enable_following':
                client = self.person_follower_enable_following_client
            elif service_name == 'disable_following':
                client = self.person_follower_disable_following_client
            else:
                return
            
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"Person follower service {service_name} not available")
                return
                
            request = Trigger.Request()
            future = client.call_async(request)
            
            # Note: In a full implementation, you'd want to handle the response
            # For now, we'll just fire and forget
            
        except Exception as e:
            self.get_logger().error(f"Error calling person follower service {service_name}: {e}")
    
    def publish_pending_commands(self):
        """Publish pending commands from main thread (thread-safe)."""
        if self.pending_cmd_vel is not None:
            cmd_vel = self.pending_cmd_vel
            self.pending_cmd_vel = None  # Clear before publish
            self.cmd_vel_pub.publish(cmd_vel)
    
    # Data callbacks
    def battery_voltage_callback(self, msg):
        self.latest_battery_voltage = msg.data
        self.maybe_broadcast_update()
    
    def battery_status_callback(self, msg):
        self.latest_battery_status = msg.data
        self.maybe_broadcast_update()
    
    def person_follower_status_callback(self, msg):
        self.latest_person_follower_status = msg.data
        self.maybe_broadcast_update()
    
    def camera_callback(self, msg):
        """Process camera image for web display."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Resize for web display (reduce bandwidth)
            height, width = cv_image.shape[:2]
            if width > 640:
                scale = 640 / width
                new_width = 640
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Convert to base64
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            self.latest_camera_image = img_base64
            
            self.maybe_broadcast_update()
            
        except Exception as e:
            self.get_logger().error(f"Camera processing error: {e}")
    
    def maybe_broadcast_update(self):
        """Broadcast update to all clients (with throttling)."""
        now = time.time()
        if now - self.last_broadcast_time > self.broadcast_interval:
            self.broadcast_update()
            self.last_broadcast_time = now
    
    def broadcast_update(self):
        """Broadcast current status to all connected clients."""
        if not self.websocket_clients:
            return
            
        try:
            payload = {
                'type': 'status_update',
                'battery_voltage': self.latest_battery_voltage,
                'battery_status': self.latest_battery_status,
                'person_follower_status': self.latest_person_follower_status,
                'camera_image': self.latest_camera_image,
                'teleop_active': self.teleop_active,
            }
            
            message = json.dumps(payload)
            
            # Send to all connected clients
            disconnected = set()
            for client in self.websocket_clients:
                try:
                    if self.ws_loop is not None:
                        asyncio.run_coroutine_threadsafe(
                            client.send(message), 
                            self.ws_loop
                        )
                except Exception:
                    disconnected.add(client)
            
            # Remove disconnected clients
            self.websocket_clients -= disconnected
            
        except Exception as e:
            self.get_logger().error(f"Broadcast error: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PackRobotDashboard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()