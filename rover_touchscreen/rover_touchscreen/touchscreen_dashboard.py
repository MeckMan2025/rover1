#!/usr/bin/env python3
"""
Simple PyQt Touchscreen Dashboard for 7" Rover Display
Low CPU impact with minimal elements:
- Live camera feed (main area)
- Battery voltage (bottom strip)
- GPS satellite count (bottom strip)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSizePolicy)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QPixmap, QFont, QImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from gnss_health_monitor.msg import GnssHealth
from cv_bridge import CvBridge
import cv2
import numpy as np


class DataUpdater(QObject):
    """QObject to handle ROS data updates and emit Qt signals"""
    
    # Qt signals for thread-safe updates
    image_updated = pyqtSignal(np.ndarray)
    battery_updated = pyqtSignal(float)
    gps_updated = pyqtSignal(int)


class RoverTouchscreenNode(Node):
    """ROS 2 Node for touchscreen data collection"""
    
    def __init__(self, data_updater):
        super().__init__('rover_touchscreen')
        self.data_updater = data_updater
        self.bridge = CvBridge()
        
        # Subscribe to camera feed
        self.camera_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.camera_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Subscribe to battery voltage
        self.battery_sub = self.create_subscription(
            Float32,
            '/battery_voltage',
            self.battery_callback,
            10
        )
        
        # Subscribe to GPS health
        self.gps_sub = self.create_subscription(
            GnssHealth,
            '/gnss/health',
            self.gps_callback,
            10
        )
        
        self.get_logger().info('Rover touchscreen node started')
    
    def camera_callback(self, msg):
        """Convert ROS image to OpenCV and emit signal"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.data_updater.image_updated.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Camera conversion error: {e}')
    
    def battery_callback(self, msg):
        """Emit battery voltage update"""
        if msg.data > 0:
            self.data_updater.battery_updated.emit(msg.data)
    
    def gps_callback(self, msg):
        """Emit GPS satellite count update"""
        self.data_updater.gps_updated.emit(msg.sat_used)


class TouchscreenDashboard(QMainWindow):
    """Simple touchscreen dashboard with large, touch-friendly elements"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover Dashboard")
        self.setGeometry(0, 0, 800, 480)  # 7" screen typical resolution
        
        # Make fullscreen for kiosk mode
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.showFullScreen()
        
        # Set dark background
        self.setStyleSheet("background-color: #1a1a1a; color: white;")
        
        # Initialize data variables
        self.current_battery = 0.0
        self.current_sats = 0
        
        self.setup_ui()
        self.setup_timers()
        
    def setup_ui(self):
        """Create the simple UI layout"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main vertical layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # Camera feed (main area - 80% of height)
        self.camera_label = QLabel("WAITING FOR CAMERA...")
        self.camera_label.setStyleSheet("""
            background-color: #000000;
            border: 2px solid #333333;
            color: #888888;
            font-size: 24px;
            font-weight: bold;
        """)
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_label.setMinimumHeight(320)
        
        main_layout.addWidget(self.camera_label, 8)  # 80% weight
        
        # Status strip (bottom - 20% of height)
        status_layout = QHBoxLayout()
        status_layout.setSpacing(20)
        
        # Battery status
        self.battery_label = QLabel("BATT: --V")
        self.battery_label.setStyleSheet("""
            background-color: #2a2a2a;
            border: 2px solid #444444;
            padding: 15px;
            font-size: 28px;
            font-weight: bold;
            color: #00ff88;
        """)
        self.battery_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.battery_label)
        
        # GPS satellites status  
        self.gps_label = QLabel("GPS: -- SATS")
        self.gps_label.setStyleSheet("""
            background-color: #2a2a2a;
            border: 2px solid #444444;
            padding: 15px;
            font-size: 28px;
            font-weight: bold;
            color: #0088ff;
        """)
        self.gps_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.gps_label)
        
        main_layout.addLayout(status_layout, 2)  # 20% weight
        
    def setup_timers(self):
        """Setup 3-second update timers for low CPU impact"""
        # Timer for status updates (3 seconds as requested)
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status_display)
        self.status_timer.start(3000)  # 3 seconds
        
    def update_camera(self, cv_image):
        """Update camera display (called from ROS callback)"""
        try:
            # Resize image to fit display while maintaining aspect ratio
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            
            # Scale down for performance (7" screen doesn't need full res)
            display_width = 640
            display_height = int(height * (display_width / width))
            resized = cv2.resize(cv_image, (display_width, display_height))
            
            # Convert BGR to RGB for Qt
            rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            # Create Qt pixmap
            height, width, channel = rgb_image.shape
            bytes_per_line = 3 * width
            q_image = QPixmap.fromImage(
                QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            )
            
            # Update label
            self.camera_label.setPixmap(q_image.scaled(
                self.camera_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            ))
            
        except Exception as e:
            print(f"Camera update error: {e}")
    
    def update_battery(self, voltage):
        """Update battery voltage display"""
        self.current_battery = voltage
        
    def update_gps(self, sat_count):
        """Update GPS satellite count"""
        self.current_sats = sat_count
        
    def update_status_display(self):
        """Update status labels - called every 3 seconds"""
        # Update battery with color coding
        if self.current_battery > 0:
            battery_text = f"BATT: {self.current_battery:.1f}V"
            
            # Color code based on voltage level
            if self.current_battery >= 12.0:
                color = "#00ff88"  # Green
            elif self.current_battery >= 11.1:
                color = "#ffc107"  # Yellow  
            else:
                color = "#ff4444"  # Red
                
            self.battery_label.setText(battery_text)
            self.battery_label.setStyleSheet(f"""
                background-color: #2a2a2a;
                border: 2px solid #444444;
                padding: 15px;
                font-size: 28px;
                font-weight: bold;
                color: {color};
            """)
        else:
            self.battery_label.setText("BATT: --V")
            
        # Update GPS with color coding
        if self.current_sats > 0:
            gps_text = f"GPS: {self.current_sats} SATS"
            
            # Color code based on satellite count
            if self.current_sats >= 20:
                color = "#00ff88"  # Green
            elif self.current_sats >= 10:
                color = "#ffc107"  # Yellow
            else:
                color = "#ff4444"  # Red
                
            self.gps_label.setText(gps_text)
            self.gps_label.setStyleSheet(f"""
                background-color: #2a2a2a;
                border: 2px solid #444444;
                padding: 15px;
                font-size: 28px;
                font-weight: bold;
                color: {color};
            """)
        else:
            self.gps_label.setText("GPS: -- SATS")
    
    def keyPressEvent(self, event):
        """Handle escape key to exit fullscreen for development"""
        if event.key() == Qt.Key_Escape:
            self.showNormal()
        super().keyPressEvent(event)


def main(args=None):
    """Main application entry point"""
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create data updater for thread-safe communication
    data_updater = DataUpdater()
    
    # Create main window
    dashboard = TouchscreenDashboard()
    
    # Connect signals to dashboard update methods
    data_updater.image_updated.connect(dashboard.update_camera)
    data_updater.battery_updated.connect(dashboard.update_battery)
    data_updater.gps_updated.connect(dashboard.update_gps)
    
    # Create ROS node
    ros_node = RoverTouchscreenNode(data_updater)
    
    # Setup ROS timer to process callbacks
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(50)  # 20Hz ROS processing
    
    # Show dashboard
    dashboard.show()
    
    try:
        # Run Qt event loop
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()