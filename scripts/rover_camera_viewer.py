#!/usr/bin/env python3
"""
Rover Camera Viewer - Full-screen live camera feed for 7-inch touchscreen

Launches automatically after brain selection to provide immediate visual feedback
for rover operations. Optimized for touchscreen interaction and field use.

Usage:
    python3 rover_camera_viewer.py [--rover1|--rover2]

Features:
- Full-screen live camera feed
- Auto-connects to appropriate camera topic
- Touch anywhere to exit
- Graceful handling of camera availability
- Optimized for 7-inch display
"""

import sys
import os
import signal
import time
from pathlib import Path

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, 
                           QWidget, QMessageBox, QProgressBar, QHBoxLayout)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt, QSize
from PyQt5.QtGui import QPixmap, QImage, QFont, QPalette, QColor

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    """ROS2 node to subscribe to camera feed"""
    
    def __init__(self, camera_topic):
        super().__init__('rover_camera_viewer')
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_topic = camera_topic
        self.subscription = None
        self.connected = False
        
        # Try to create subscription
        self.create_camera_subscription()
        
        # Timer to check topic availability
        self.connection_timer = self.create_timer(1.0, self.check_connection)
        
    def create_camera_subscription(self):
        """Create camera subscription if topic is available"""
        try:
            self.subscription = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                qos_profile_sensor_data
            )
            self.get_logger().info(f'Subscribed to {self.camera_topic}')
        except Exception as e:
            self.get_logger().warn(f'Failed to subscribe to {self.camera_topic}: {e}')
    
    def check_connection(self):
        """Check if we're receiving images"""
        if self.latest_image is not None and not self.connected:
            self.connected = True
            self.get_logger().info('Camera feed connected successfully')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')


class CameraWorker(QThread):
    """Worker thread for ROS2 camera subscription"""
    image_received = pyqtSignal(object)
    connection_status = pyqtSignal(bool, str)
    
    def __init__(self, camera_topic):
        super().__init__()
        self.camera_topic = camera_topic
        self.running = True
        self.node = None
        
    def run(self):
        """Main worker thread loop"""
        try:
            rclpy.init()
            self.node = CameraSubscriber(self.camera_topic)
            
            self.connection_status.emit(False, f"Connecting to {self.camera_topic}...")
            
            # Process ROS messages
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
                # Emit latest image if available
                if self.node.latest_image is not None:
                    self.image_received.emit(self.node.latest_image.copy())
                    if not self.node.connected:
                        self.connection_status.emit(True, "Camera connected")
                        
        except Exception as e:
            self.connection_status.emit(False, f"Error: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
            rclpy.shutdown()
    
    def stop(self):
        """Stop the worker thread"""
        self.running = False


class CameraViewer(QMainWindow):
    """Main camera viewer window"""
    
    def __init__(self, rover_type="rover1"):
        super().__init__()
        
        # Determine camera topic based on rover type
        if rover_type == "rover2":
            self.camera_topic = "/ascamera_hp60c/camera_publisher/rgb0/image"
            self.window_title = "Rover2 Pack Robot - Camera Feed"
        else:
            self.camera_topic = "/ascamera_hp60c/camera_publisher/rgb0/image" 
            self.window_title = "Rover1 - Camera Feed"
            
        self.rover_type = rover_type
        self.init_ui()
        self.init_camera()
        
    def init_ui(self):
        """Initialize user interface"""
        self.setWindowTitle(self.window_title)
        self.setGeometry(0, 0, 1024, 600)  # 7-inch display resolution
        
        # Set black background
        self.setStyleSheet("background-color: black;")
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Status bar at top
        self.status_widget = QWidget()
        self.status_widget.setFixedHeight(40)
        self.status_widget.setStyleSheet("background-color: rgba(0, 0, 0, 180);")
        
        status_layout = QHBoxLayout(self.status_widget)
        
        # Status label
        self.status_label = QLabel("Starting camera...")
        self.status_label.setStyleSheet("color: white; font-size: 14px; font-weight: bold;")
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        
        # Exit instruction
        self.exit_label = QLabel("Touch anywhere to exit")
        self.exit_label.setStyleSheet("color: #888888; font-size: 12px;")
        self.exit_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        status_layout.addWidget(self.exit_label)
        
        # Camera display
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: none;")
        self.image_label.setScaledContents(True)
        
        # Set placeholder
        self.show_placeholder()
        
        layout.addWidget(self.status_widget)
        layout.addWidget(self.image_label, 1)  # Stretch factor 1 for main content
        
        # Make window full screen
        self.showFullScreen()
        
        # Set cursor invisible for kiosk mode
        self.setCursor(Qt.BlankCursor)
        
    def init_camera(self):
        """Initialize camera worker thread"""
        self.camera_worker = CameraWorker(self.camera_topic)
        self.camera_worker.image_received.connect(self.update_image)
        self.camera_worker.connection_status.connect(self.update_status)
        self.camera_worker.start()
        
    def show_placeholder(self):
        """Show placeholder when no camera feed"""
        placeholder = QPixmap(640, 480)
        placeholder.fill(QColor(40, 40, 40))
        
        # Scale to fit display
        scaled_placeholder = placeholder.scaled(
            self.image_label.size(), 
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        self.image_label.setPixmap(scaled_placeholder)
        
    def update_image(self, cv_image):
        """Update display with new camera image"""
        try:
            # Convert OpenCV image to Qt format
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            
            # Convert to pixmap and scale to fit screen
            pixmap = QPixmap.fromImage(q_image)
            
            # Get current widget size for scaling
            label_size = self.image_label.size()
            scaled_pixmap = pixmap.scaled(
                label_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            self.image_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            print(f"Error updating image: {e}")
    
    def update_status(self, connected, message):
        """Update connection status"""
        if connected:
            self.status_label.setStyleSheet("color: #00ff88; font-size: 14px; font-weight: bold;")
            self.status_label.setText(f"● {self.rover_type.upper()} CAMERA LIVE")
        else:
            self.status_label.setStyleSheet("color: #ffc107; font-size: 14px; font-weight: bold;")
            self.status_label.setText(f"○ {message}")
    
    def mousePressEvent(self, event):
        """Exit on any mouse/touch press"""
        self.close()
        
    def keyPressEvent(self, event):
        """Exit on any key press"""
        self.close()
        
    def closeEvent(self, event):
        """Clean shutdown"""
        if hasattr(self, 'camera_worker'):
            self.camera_worker.stop()
            self.camera_worker.wait(2000)  # Wait up to 2 seconds for clean shutdown
        event.accept()


def signal_handler(signum, frame):
    """Handle system signals for clean shutdown"""
    print("\nReceived signal, shutting down...")
    QApplication.quit()


def main():
    """Main application entry point"""
    
    # Parse command line arguments
    rover_type = "rover1"  # Default
    if len(sys.argv) > 1:
        if sys.argv[1] in ["--rover2", "rover2"]:
            rover_type = "rover2"
        elif sys.argv[1] in ["--rover1", "rover1"]:
            rover_type = "rover1"
    
    # Install signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create QApplication
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)
    
    # Set application properties for kiosk mode
    app.setOverrideCursor(Qt.BlankCursor)
    
    # Create and show camera viewer
    viewer = CameraViewer(rover_type)
    viewer.show()
    
    print(f"Rover Camera Viewer started for {rover_type}")
    print("Touch anywhere on screen to exit")
    
    # Run application
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\nShutting down...")
        viewer.close()


if __name__ == "__main__":
    main()