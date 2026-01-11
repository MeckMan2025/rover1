#!/usr/bin/env python3
"""
Battery Monitor Node - Monitors rover battery voltage via I2C

This node reads battery voltage from the Hiwonder controller and publishes
battery status information. Critical for pack robot safety.

Essential component for Pack Robot safety monitoring.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import smbus2
import time


class BatteryMonitor(Node):
    """ROS 2 node for battery voltage monitoring."""
    
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x34)
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('low_voltage_threshold', 10.5)  # Volts
        self.declare_parameter('critical_voltage_threshold', 10.0)  # Volts
        
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.low_voltage_threshold = self.get_parameter('low_voltage_threshold').value
        self.critical_voltage_threshold = self.get_parameter('critical_voltage_threshold').value
        
        # I2C setup
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)
            self.get_logger().info(f"Battery monitor I2C initialized on bus {self.i2c_bus}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C for battery monitor: {e}")
            return
        
        # State
        self.last_voltage = 0.0
        self.voltage_warnings_sent = False
        self.critical_warnings_sent = False
        
        # Publishers
        self.voltage_pub = self.create_publisher(Float32, '/battery/voltage', 10)
        self.status_pub = self.create_publisher(String, '/battery/status', 10)
        
        # Timer for regular readings
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.read_battery)
        
        self.get_logger().info("Battery monitor initialized")
    
    def read_battery(self):
        """Read battery voltage from I2C and publish status."""
        try:
            # Read battery voltage from controller
            # Adjust register address based on your hardware
            data = self.bus.read_i2c_block_data(self.i2c_address, 0x30, 2)
            
            # Convert to voltage (adjust scaling based on your hardware)
            # This is a simplified example - adjust for your setup
            raw_value = (data[0] << 8) | data[1]
            voltage = raw_value * 0.01  # Example scaling factor
            
            # Publish voltage
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.voltage_pub.publish(voltage_msg)
            
            # Determine status and publish warnings
            status = self.get_battery_status(voltage)
            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)
            
            # Log voltage changes
            if abs(voltage - self.last_voltage) > 0.1:  # Only log significant changes
                self.get_logger().info(f"Battery voltage: {voltage:.2f}V ({status})")
                self.last_voltage = voltage
                
        except Exception as e:
            self.get_logger().error(f"Failed to read battery voltage: {e}")
    
    def get_battery_status(self, voltage: float) -> str:
        """Determine battery status and handle warnings."""
        if voltage < self.critical_voltage_threshold:
            if not self.critical_warnings_sent:
                self.get_logger().error(f"CRITICAL: Battery voltage very low: {voltage:.2f}V - SHUTDOWN RECOMMENDED")
                self.critical_warnings_sent = True
            return "CRITICAL"
        elif voltage < self.low_voltage_threshold:
            if not self.voltage_warnings_sent:
                self.get_logger().warn(f"WARNING: Battery voltage low: {voltage:.2f}V")
                self.voltage_warnings_sent = True
            return "LOW"
        else:
            # Reset warning flags when voltage recovers
            self.voltage_warnings_sent = False
            self.critical_warnings_sent = False
            if voltage > 12.0:
                return "GOOD"
            else:
                return "OK"
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        try:
            if hasattr(self, 'bus'):
                self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BatteryMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()