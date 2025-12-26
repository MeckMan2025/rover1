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

from gnss_health_monitor.msg import GnssHealth


class GnssWebDashboard(Node):
    """Web dashboard server for GNSS health monitoring"""
    
    def __init__(self):
        super().__init__('gnss_web_dashboard')
        
        # Latest GNSS health data
        self.latest_health = None
        self.health_lock = threading.Lock()
        
        # Connected WebSocket clients
        self.ws_clients = set()
        
        # Subscribe to GNSS health with reliable QoS
        self.health_sub = self.create_subscription(
            GnssHealth,
            '/gnss/health',
            self.health_callback,
            10
        )
        
        # Get package directory for serving static files
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
                'dgps_id': msg.dgps_id
            }
            
        # Broadcast to all connected WebSocket clients
        if self.ws_clients:
            asyncio.new_event_loop().run_until_complete(
                self.broadcast_health_data()
            )
    
    async def broadcast_health_data(self):
        """Send health data to all connected WebSocket clients"""
        if not self.latest_health or not self.ws_clients:
            return
            
        message = json.dumps(self.latest_health)
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