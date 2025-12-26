#!/usr/bin/env python3
"""
GNSS Health Monitor Node

Aggregates GPS/RTK/NTRIP status from multiple topics into a single
clean message for Foxglove dashboard visualization.

Author: Rover1 Project
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from collections import deque
import math
import time
from typing import Optional, Any, Dict

# Message imports
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ByteMultiArray
from gnss_health_monitor.msg import GnssHealth
from builtin_interfaces.msg import Time as TimeMsg

# Try to import common u-blox message types (graceful fallback if not available)
try:
    from ublox_msgs.msg import NavSAT
    UBLOX_AVAILABLE = True
except ImportError:
    UBLOX_AVAILABLE = False
    
try:
    from rtcm_msgs.msg import Message as RTCMMessage
    RTCM_MSGS_AVAILABLE = True
except ImportError:
    RTCM_MSGS_AVAILABLE = False


class RTCMStatistics:
    """Rolling statistics for RTCM message rates"""
    
    def __init__(self, window_seconds: float = 5.0):
        self.window_seconds = window_seconds
        self.messages = deque()  # (timestamp, byte_count)
        self.total_messages = 0
        self.last_message_time = None
        
    def add_message(self, byte_count: int) -> None:
        """Add a new RTCM message to statistics"""
        now = time.time()
        self.messages.append((now, byte_count))
        self.total_messages += 1
        self.last_message_time = now
        
        # Remove old messages outside window
        cutoff_time = now - self.window_seconds
        while self.messages and self.messages[0][0] < cutoff_time:
            self.messages.popleft()
    
    def get_rates(self) -> tuple[float, float]:
        """Get (messages_per_sec, bytes_per_sec)"""
        if not self.messages:
            return 0.0, 0.0
            
        if len(self.messages) == 1:
            return 1.0 / self.window_seconds, self.messages[0][1] / self.window_seconds
            
        time_span = self.messages[-1][0] - self.messages[0][0]
        if time_span <= 0:
            return 0.0, 0.0
            
        msg_rate = len(self.messages) / time_span
        byte_rate = sum(msg[1] for msg in self.messages) / time_span
        
        return msg_rate, byte_rate
    
    def get_age_seconds(self) -> float:
        """Get seconds since last message"""
        if self.last_message_time is None:
            return float('inf')
        return time.time() - self.last_message_time


class GnssHealthMonitorNode(Node):
    """
    GNSS Health Monitor Node
    
    Subscribes to various GPS/RTK topics and publishes aggregated health status
    """
    
    def __init__(self):
        super().__init__('gnss_health_monitor_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navsat_topic', '/gps/filtered'),
                ('navsat_fallback_topic', '/fix'),
                ('navsat_timeout_s', 2.0),
                ('ubx_nav_sat_topic', '/ubx_nav_sat'),
                ('rtcm_topic', '/ntrip_client/rtcm'),
                ('rtcm_fallback_topic', '/rtcm'),
                ('rtcm_timeout_s', 5.0),
                ('publish_rate_hz', 5.0),
                ('rtk_fixed_h_acc_threshold_m', 0.05),
                ('rtk_float_h_acc_threshold_m', 0.30),
                ('sat_used_flag_mask', 0x08),
                ('use_ublox_carrsoln_if_available', True),
            ]
        )
        
        # Get parameters
        self.params = {
            'navsat_topic': self.get_parameter('navsat_topic').value,
            'navsat_fallback_topic': self.get_parameter('navsat_fallback_topic').value,
            'navsat_timeout_s': self.get_parameter('navsat_timeout_s').value,
            'ubx_nav_sat_topic': self.get_parameter('ubx_nav_sat_topic').value,
            'rtcm_topic': self.get_parameter('rtcm_topic').value,
            'rtcm_fallback_topic': self.get_parameter('rtcm_fallback_topic').value,
            'rtcm_timeout_s': self.get_parameter('rtcm_timeout_s').value,
            'publish_rate_hz': self.get_parameter('publish_rate_hz').value,
            'rtk_fixed_h_acc_threshold_m': self.get_parameter('rtk_fixed_h_acc_threshold_m').value,
            'rtk_float_h_acc_threshold_m': self.get_parameter('rtk_float_h_acc_threshold_m').value,
            'sat_used_flag_mask': self.get_parameter('sat_used_flag_mask').value,
            'use_ublox_carrsoln_if_available': self.get_parameter('use_ublox_carrsoln_if_available').value,
        }
        
        # State variables
        self.last_navsat_time = None
        self.last_navsat_msg = None
        self.sat_visible = 0
        self.sat_used = 0
        self.dgps_id = -1
        self.rtcm_stats = RTCMStatistics(window_seconds=5.0)
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher
        self.health_pub = self.create_publisher(
            GnssHealth,
            '/gnss/health',
            10
        )
        
        # Setup subscriptions
        self._setup_subscriptions()
        
        # Publisher timer
        timer_period = 1.0 / self.params['publish_rate_hz']
        self.timer = self.create_timer(timer_period, self.publish_health_status)
        
        # Status logging timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.log_status)
        
        self.get_logger().info(f"GNSS Health Monitor started. Publishing at {self.params['publish_rate_hz']} Hz")
        
    def _setup_subscriptions(self):
        """Setup all topic subscriptions with fallback logic"""
        
        # NavSat subscription (primary)
        try:
            self.navsat_sub = self.create_subscription(
                NavSatFix,
                self.params['navsat_topic'],
                self.navsat_callback,
                self.sensor_qos
            )
            self.get_logger().info(f"Subscribed to NavSat: {self.params['navsat_topic']}")
        except Exception as e:
            self.get_logger().warn(f"Failed to subscribe to {self.params['navsat_topic']}: {e}")
            
        # NavSat fallback subscription
        try:
            self.navsat_fallback_sub = self.create_subscription(
                NavSatFix,
                self.params['navsat_fallback_topic'],
                self.navsat_fallback_callback,
                self.sensor_qos
            )
            self.get_logger().info(f"Subscribed to NavSat fallback: {self.params['navsat_fallback_topic']}")
        except Exception as e:
            self.get_logger().warn(f"Failed to subscribe to {self.params['navsat_fallback_topic']}: {e}")
            
        # Satellite info subscription (if ublox available)
        if UBLOX_AVAILABLE:
            try:
                self.nav_sat_sub = self.create_subscription(
                    NavSAT,
                    self.params['ubx_nav_sat_topic'],
                    self.nav_sat_callback,
                    self.sensor_qos
                )
                self.get_logger().info(f"Subscribed to u-blox NavSAT: {self.params['ubx_nav_sat_topic']}")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to {self.params['ubx_nav_sat_topic']}: {e}")
        else:
            self.get_logger().warn("ublox_msgs not available, satellite counts will be unavailable")
            
        # RTCM subscription (primary)
        self._setup_rtcm_subscription(self.params['rtcm_topic'], is_primary=True)
        
        # RTCM fallback subscription
        self._setup_rtcm_subscription(self.params['rtcm_fallback_topic'], is_primary=False)
        
    def _setup_rtcm_subscription(self, topic: str, is_primary: bool):
        """Setup RTCM subscription with multiple message type support"""
        
        # Try RTCM message type first
        if RTCM_MSGS_AVAILABLE:
            try:
                sub = self.create_subscription(
                    RTCMMessage,
                    topic,
                    lambda msg, primary=is_primary: self.rtcm_callback(msg, primary),
                    10
                )
                label = "primary" if is_primary else "fallback"
                self.get_logger().info(f"Subscribed to RTCM ({label}): {topic} [rtcm_msgs/Message]")
                return
            except Exception as e:
                self.get_logger().warn(f"Failed RTCM subscription to {topic}: {e}")
        
        # Fallback to ByteMultiArray
        try:
            sub = self.create_subscription(
                ByteMultiArray,
                topic,
                lambda msg, primary=is_primary: self.rtcm_bytes_callback(msg, primary),
                10
            )
            label = "primary" if is_primary else "fallback"
            self.get_logger().info(f"Subscribed to RTCM ({label}): {topic} [std_msgs/ByteMultiArray]")
        except Exception as e:
            label = "primary" if is_primary else "fallback"
            self.get_logger().warn(f"Failed RTCM ({label}) subscription to {topic}: {e}")
    
    def navsat_callback(self, msg: NavSatFix):
        """Primary NavSat callback"""
        self.last_navsat_msg = msg
        self.last_navsat_time = self.get_clock().now()
        
    def navsat_fallback_callback(self, msg: NavSatFix):
        """Fallback NavSat callback (only used if primary is stale)"""
        if (self.last_navsat_time is None or 
            (self.get_clock().now() - self.last_navsat_time).nanoseconds > 
            self.params['navsat_timeout_s'] * 1e9):
            self.last_navsat_msg = msg
            self.last_navsat_time = self.get_clock().now()
    
    def nav_sat_callback(self, msg):
        """u-blox NavSAT callback for satellite information"""
        try:
            # Detect message structure
            if hasattr(msg, 'sv'):
                satellites = msg.sv
            elif hasattr(msg, 'sats'):
                satellites = msg.sats
            elif hasattr(msg, 'satellites'):
                satellites = msg.satellites
            else:
                self.get_logger().warn_once("Unknown NavSAT message structure")
                return
                
            self.sat_visible = len(satellites)
            self.sat_used = 0
            
            # Count satellites used in solution
            for sat in satellites:
                used = False
                
                # Try different field names for "used in solution"
                if hasattr(sat, 'used') and sat.used:
                    used = True
                elif hasattr(sat, 'usedInNav') and sat.usedInNav:
                    used = True
                elif hasattr(sat, 'flags'):
                    # Check flags with mask
                    used = (sat.flags & self.params['sat_used_flag_mask']) != 0
                elif hasattr(sat, 'quality') and hasattr(sat, 'quality') and sat.quality > 0:
                    used = True
                    
                if used:
                    self.sat_used += 1
                    
        except Exception as e:
            self.get_logger().warn(f"Error processing NavSAT message: {e}")
    
    def rtcm_callback(self, msg, is_primary: bool):
        """RTCM message callback (rtcm_msgs/Message)"""
        try:
            byte_count = len(msg.message) if hasattr(msg, 'message') else len(msg.data)
            self.rtcm_stats.add_message(byte_count)
        except Exception as e:
            self.get_logger().warn(f"Error processing RTCM message: {e}")
    
    def rtcm_bytes_callback(self, msg: ByteMultiArray, is_primary: bool):
        """RTCM ByteMultiArray callback"""
        try:
            byte_count = len(msg.data)
            self.rtcm_stats.add_message(byte_count)
        except Exception as e:
            self.get_logger().warn(f"Error processing RTCM ByteMultiArray: {e}")
    
    def compute_accuracy_from_covariance(self, covariance) -> tuple[float, float]:
        """Extract horizontal and vertical accuracy from NavSatFix covariance"""
        try:
            if not covariance or len(covariance) < 9:
                return float('nan'), float('nan')
                
            # Check if covariance is all zeros (invalid)
            if all(abs(c) < 1e-9 for c in covariance):
                return float('nan'), float('nan')
                
            # Extract diagonal elements (variance)
            # NavSatFix covariance: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
            var_x = max(covariance[0], 0.0)  # East variance
            var_y = max(covariance[4], 0.0)  # North variance  
            var_z = max(covariance[8], 0.0)  # Up variance
            
            h_acc = math.sqrt(var_x + var_y)  # Horizontal accuracy (RMS)
            v_acc = math.sqrt(var_z)          # Vertical accuracy
            
            return h_acc, v_acc
            
        except Exception as e:
            self.get_logger().warn(f"Error computing accuracy: {e}")
            return float('nan'), float('nan')
    
    def determine_rtk_state(self, h_acc: float, ntrip_connected: bool) -> str:
        """Determine RTK state based on accuracy and NTRIP status"""
        
        # Check if we have recent NavSat data
        if (self.last_navsat_time is None or 
            (self.get_clock().now() - self.last_navsat_time).nanoseconds > 
            self.params['navsat_timeout_s'] * 1e9):
            return "NO_FIX"
            
        # If no NTRIP corrections
        if not ntrip_connected:
            return "DGPS"
            
        # If we have valid accuracy, use thresholds (-1.0 means invalid)
        if h_acc > 0:  # Valid accuracy data
            if h_acc <= self.params['rtk_fixed_h_acc_threshold_m']:
                return "FIXED"
            elif h_acc <= self.params['rtk_float_h_acc_threshold_m']:
                return "FLOAT"
                
        return "DGPS"
    
    def publish_health_status(self):
        """Publish aggregated GNSS health status"""
        
        try:
            msg = GnssHealth()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            
            # Satellite information
            msg.sat_visible = int(self.sat_visible)
            msg.sat_used = int(self.sat_used)
            
            # RTCM statistics
            corr_age = self.rtcm_stats.get_age_seconds()
            msg.corr_age_s = float(corr_age) if corr_age != float('inf') else 999.9
            msg.ntrip_connected = corr_age <= self.params['rtcm_timeout_s']
            msg.rtcm_msgs_total = int(self.rtcm_stats.total_messages)
            
            msgs_per_sec, bytes_per_sec = self.rtcm_stats.get_rates()
            msg.rtcm_msgs_per_sec = float(msgs_per_sec)
            msg.rtcm_bytes_per_sec = float(bytes_per_sec)
            
            # Accuracy and RTK state
            if self.last_navsat_msg:
                h_acc, v_acc = self.compute_accuracy_from_covariance(self.last_navsat_msg.position_covariance)
                # Use -1.0 instead of NaN for invalid values
                msg.h_acc_m = float(h_acc) if not math.isnan(h_acc) else -1.0
                msg.v_acc_m = float(v_acc) if not math.isnan(v_acc) else -1.0
                
                # Set last update time
                msg.last_update_time = self.last_navsat_time.to_msg()
            else:
                msg.h_acc_m = -1.0  # Use -1.0 to indicate no data
                msg.v_acc_m = -1.0
                # Create zero time
                msg.last_update_time.sec = 0
                msg.last_update_time.nanosec = 0
                
            msg.rtk_state = self.determine_rtk_state(msg.h_acc_m, msg.ntrip_connected)
            
            # DGPS ID (placeholder for future enhancement)  
            msg.dgps_id = int(self.dgps_id)
            
            self.health_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing health status: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
    
    def log_status(self):
        """Log status summary (throttled to 1 Hz)"""
        
        corr_age = self.rtcm_stats.get_age_seconds()
        ntrip_status = "CONNECTED" if corr_age <= self.params['rtcm_timeout_s'] else "DISCONNECTED"
        
        h_acc_str = f"{self.last_navsat_msg and not math.isnan(self.compute_accuracy_from_covariance(self.last_navsat_msg.position_covariance)[0]) and f'{self.compute_accuracy_from_covariance(self.last_navsat_msg.position_covariance)[0]:.3f}m' or 'N/A'}"
        
        rtk_state = self.determine_rtk_state(
            self.last_navsat_msg and self.compute_accuracy_from_covariance(self.last_navsat_msg.position_covariance)[0] or float('nan'),
            corr_age <= self.params['rtcm_timeout_s']
        )
        
        msgs_per_sec, _ = self.rtcm_stats.get_rates()
        
        self.get_logger().info(
            f"GNSS: {self.sat_used}/{self.sat_visible} sats | "
            f"{rtk_state} | "
            f"H_ACC: {h_acc_str} | "
            f"CORR_AGE: {corr_age:.1f}s | "
            f"RTCM: {msgs_per_sec:.1f} msg/s | "
            f"NTRIP: {ntrip_status}"
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GnssHealthMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()