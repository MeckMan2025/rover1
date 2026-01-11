#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
import math

class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')
        
        # Parameters to tune
        self.declare_parameter('wheel_separation_width', 0.20)  # meters (left to right)
        self.declare_parameter('wheel_separation_length', 0.16) # meters (front to back)
        self.declare_parameter('wheel_radius', 0.04)           # meters
        self.declare_parameter('max_rpm', 200)                 # Motor max RPM
        self.declare_parameter('pwm_limit', 100)               # Max PWM/Speed value for driver
        self.declare_parameter('rotation_scale', 2.0)          # Moderate rotation boost
        
        # Topics
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.publisher_ = self.create_publisher(Int32MultiArray, 'wheel_speeds_raw', 10)
        
        # Subscribe to Real Encoders
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_callback,
            10)
            
        # Odometry Publisher
        self.odom_pub_ = self.create_publisher(Odometry, 'odom/wheel_odom', 10)
        
        # State
        self.last_ticks = None
        self.last_time = None
        
        # Calibration (Pulse per Rotation)
        # JGB37-520 usually has 11 PPR * 90 Gear Ratio ~= 990 ticks/rev?
        # User manual says "Range: depending on specific engine".
        # Let's assume standard 11 PPR magnet * ratio. 
        # But we need to calibrate this. For now, estimate based on 110RPM spec.
        # If 110 RPM max speed, and ticks are raw Hall sensor?
        # Let's declare a parameter ticks_per_rev.
        self.declare_parameter('ticks_per_rev', 3171.44) # Calibrated on 2025-12-22
        
        self.get_logger().info('Mecanum Kinematics Node Started')

    def encoder_callback(self, msg):
        if len(msg.data) != 4: return
        
        current_ticks = msg.data
        current_time = self.get_clock().now()
        
        if self.last_ticks is None:
            self.last_ticks = current_ticks
            self.last_time = current_time
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0: return

        # Calculate Delta Ticks
        # Handle wrap-around if necessary? 32-bit int is huge, so we likely won't wrap soon.
        d_ticks = [c - l for c, l in zip(current_ticks, self.last_ticks)]
        
        self.last_ticks = current_ticks
        self.last_time = current_time
        
        # Convert to Angular Velocity (rad/s)
        # w = (delta_ticks / dt) / ticks_per_rev * 2pi
        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        w_wheels = []
        for d in d_ticks:
            w = (d / dt) / ticks_per_rev * (2.0 * math.pi)
            w_wheels.append(w)
            
        w_fl, w_fr, w_rl, w_rr = w_wheels
        
        # Forward Kinematics (Mecanum)
        # lx = separations...
        Lx = self.get_parameter('wheel_separation_width').value / 2.0
        Ly = self.get_parameter('wheel_separation_length').value / 2.0
        R = self.get_parameter('wheel_radius').value
        k = Lx + Ly
        
        # Invert the matrix:
        # vx = (w_fl + w_fr + w_rl + w_rr) * R / 4
        # vy = (-w_fl + w_fr + w_rl - w_rr) * R / 4
        # wz = (-w_fl + w_fr - w_rl + w_rr) * R / (4 * k)
        
        vx = (w_fl + w_fr + w_rl + w_rr) * R / 4.0
        vy = (-w_fl + w_fr + w_rl - w_rr) * R / 4.0
        wz = (-w_fl + w_fr - w_rl + w_rr) * R / (4.0 * k)
        
        # Publish Odometry (Twist only for EKF)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz
        
        # Covariance (Trust encoders moderately)
        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1 
        ]
        
        self.odom_pub_.publish(odom_msg)

    def cmd_vel_callback(self, msg):
        # 1. Kinematics (Cmd -> Wheels)
        # ... (Inverse Kinematics Logic Remains Same) ...
        Lx = self.get_parameter('wheel_separation_width').value / 2.0
        Ly = self.get_parameter('wheel_separation_length').value / 2.0
        R = self.get_parameter('wheel_radius').value
        
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z * self.get_parameter('rotation_scale').value
        
        k = Lx + Ly
        
        w_fl = (vx - vy - k * wz) / R
        w_fr = (vx + vy + k * wz) / R
        w_rl = (vx + vy - k * wz) / R
        w_rr = (vx - vy + k * wz) / R
        
        def to_driver_units(rad_s):
            rpm = rad_s * 9.55
            max_rpm = self.get_parameter('max_rpm').value
            pwm_limit = self.get_parameter('pwm_limit').value
            norm = rpm / max_rpm
            val = int(norm * pwm_limit)
            return max(min(val, pwm_limit), -pwm_limit)

        speeds = Int32MultiArray()
        speeds.data = [
            to_driver_units(w_fl),
            to_driver_units(w_fr),
            to_driver_units(w_rl),
            to_driver_units(w_rr)
        ]
        self.publisher_.publish(speeds)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
