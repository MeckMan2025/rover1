#!/usr/bin/env python3
"""
Mecanum Kinematics Node - Converts /cmd_vel to individual wheel speeds

This node takes Twist messages (linear x, y, angular z) and converts them
to individual mecanum wheel speeds for the motor driver.

Essential component for Pack Robot mobility.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math


class MecanumKinematics(Node):
    """ROS 2 node for mecanum wheel kinematics."""
    
    def __init__(self):
        super().__init__('mecanum_kinematics')
        
        # Robot parameters
        self.declare_parameter('wheel_radius', 0.05)  # meters
        self.declare_parameter('wheel_base_width', 0.235)  # meters
        self.declare_parameter('wheel_base_length', 0.185)  # meters
        self.declare_parameter('max_wheel_speed', 100.0)  # motor units
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_width = self.get_parameter('wheel_base_width').value
        self.wheel_base_length = self.get_parameter('wheel_base_length').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        
        # Calculate robot geometry
        self.robot_width = self.wheel_base_width
        self.robot_length = self.wheel_base_length
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.motor_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/cmd_vel_motors',
            10
        )
        
        self.get_logger().info("Mecanum kinematics initialized")
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist to individual wheel speeds."""
        # Extract velocities
        vx = msg.linear.x   # Forward/backward (m/s)
        vy = msg.linear.y   # Left/right strafe (m/s)
        wz = msg.angular.z  # Rotation (rad/s)
        
        # Mecanum wheel kinematics
        # FL = Front Left, FR = Front Right, RL = Rear Left, RR = Rear Right
        
        # Calculate wheel speeds (m/s)
        wheel_fl = vx - vy - wz * (self.robot_width + self.robot_length) / 2
        wheel_fr = vx + vy + wz * (self.robot_width + self.robot_length) / 2
        wheel_rl = vx + vy - wz * (self.robot_width + self.robot_length) / 2
        wheel_rr = vx - vy + wz * (self.robot_width + self.robot_length) / 2
        
        # Convert to motor units (assuming linear relationship)
        # You may need to adjust this based on your motor controller
        motor_scale = 100.0 / (2.0 * self.wheel_radius)  # Scale to motor units
        
        motor_fl = wheel_fl * motor_scale
        motor_fr = wheel_fr * motor_scale
        motor_rl = wheel_rl * motor_scale
        motor_rr = wheel_rr * motor_scale
        
        # Find max speed and scale if needed
        max_speed = max(abs(motor_fl), abs(motor_fr), abs(motor_rl), abs(motor_rr))
        if max_speed > self.max_wheel_speed:
            scale_factor = self.max_wheel_speed / max_speed
            motor_fl *= scale_factor
            motor_fr *= scale_factor
            motor_rl *= scale_factor
            motor_rr *= scale_factor
        
        # Publish motor commands
        motor_msg = Float32MultiArray()
        motor_msg.data = [motor_fl, motor_fr, motor_rl, motor_rr]
        self.motor_cmd_pub.publish(motor_msg)
        
        # Debug logging
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
            self.get_logger().debug(
                f"cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f} -> "
                f"motors: [{motor_fl:.1f}, {motor_fr:.1f}, {motor_rl:.1f}, {motor_rr:.1f}]"
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MecanumKinematics()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()