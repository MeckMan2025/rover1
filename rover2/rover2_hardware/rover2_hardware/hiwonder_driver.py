#!/usr/bin/env python3
"""
Hiwonder Motor Driver Node - Controls mecanum wheel motors via I2C

This node communicates with the Hiwonder motor controller board to drive
the four mecanum wheels. It subscribes to /cmd_vel_motors and publishes
wheel encoder feedback.

Essential component for Pack Robot mobility.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import smbus2
import struct
import time
import threading


class HiwonderDriver(Node):
    """ROS 2 node for Hiwonder motor controller communication."""
    
    def __init__(self):
        super().__init__('hiwonder_driver')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x34)
        self.declare_parameter('invert_fl', False)
        self.declare_parameter('invert_fr', False)
        self.declare_parameter('invert_rl', False)
        self.declare_parameter('invert_rr', False)
        
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.invert_fl = self.get_parameter('invert_fl').value
        self.invert_fr = self.get_parameter('invert_fr').value
        self.invert_rl = self.get_parameter('invert_rl').value
        self.invert_rr = self.get_parameter('invert_rr').value
        
        # I2C setup
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)
            self.get_logger().info(f"I2C bus {self.i2c_bus} initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")
            return
            
        # State
        self.last_cmd_time = time.time()
        self.watchdog_timeout = 0.5  # 500ms
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Float32MultiArray,
            '/cmd_vel_motors',
            self.cmd_vel_callback,
            10
        )
        
        self.encoder_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_encoders',
            10
        )
        
        # Watchdog timer
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        
        # Encoder reading timer
        self.encoder_timer = self.create_timer(0.05, self.read_encoders)  # 20Hz
        
        self.get_logger().info("Hiwonder driver initialized")
    
    def cmd_vel_callback(self, msg: Float32MultiArray):
        """Receive motor commands [fl, fr, rl, rr] in range [-100, 100]."""
        if len(msg.data) != 4:
            self.get_logger().warn("Expected 4 motor values")
            return
            
        fl, fr, rl, rr = msg.data
        
        # Apply inversions
        if self.invert_fl:
            fl = -fl
        if self.invert_fr:
            fr = -fr
        if self.invert_rl:
            rl = -rl
        if self.invert_rr:
            rr = -rr
            
        # Send to motor controller
        self.set_motors(fl, fr, rl, rr)
        self.last_cmd_time = time.time()
    
    def set_motors(self, fl: float, fr: float, rl: float, rr: float):
        """Send motor commands to Hiwonder controller."""
        try:
            # Clamp values
            fl = max(-100, min(100, fl))
            fr = max(-100, min(100, fr))
            rl = max(-100, min(100, rl))
            rr = max(-100, min(100, rr))
            
            # Convert to integers
            fl_int = int(fl)
            fr_int = int(fr)
            rl_int = int(rl)
            rr_int = int(rr)
            
            # Write to I2C (adjust registers based on your controller)
            # This is a simplified example - adjust for your hardware
            data = [fl_int, fr_int, rl_int, rr_int]
            self.bus.write_i2c_block_data(self.i2c_address, 0x10, data)
            
        except Exception as e:
            self.get_logger().error(f"Failed to set motors: {e}")
    
    def read_encoders(self):
        """Read encoder values from motor controller."""
        try:
            # Read encoder data from I2C
            # This is a simplified example - adjust for your hardware
            data = self.bus.read_i2c_block_data(self.i2c_address, 0x20, 16)
            
            # Parse encoder values (4 x 4 bytes = 16 bytes)
            encoders = []
            for i in range(4):
                value = struct.unpack('<i', bytes(data[i*4:(i+1)*4]))[0]
                encoders.append(float(value))
            
            # Publish encoder data
            msg = Float32MultiArray()
            msg.data = encoders
            self.encoder_pub.publish(msg)
            
        except Exception as e:
            # Don't spam error logs for encoder reads
            pass
    
    def watchdog_check(self):
        """Safety watchdog - stop motors if no recent commands."""
        if time.time() - self.last_cmd_time > self.watchdog_timeout:
            self.set_motors(0, 0, 0, 0)  # Stop all motors
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        try:
            self.set_motors(0, 0, 0, 0)  # Stop motors
            if hasattr(self, 'bus'):
                self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HiwonderDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()