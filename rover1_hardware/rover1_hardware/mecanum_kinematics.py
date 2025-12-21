#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
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
        
        # Topics
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.publisher_ = self.create_publisher(Int32MultiArray, 'wheel_speeds_raw', 10)
        
        self.get_logger().info('Mecanum Kinematics Node Started')

    def cmd_vel_callback(self, msg):
        # Robot geometry
        Lx = self.get_parameter('wheel_separation_width').value / 2.0
        Ly = self.get_parameter('wheel_separation_length').value / 2.0
        R = self.get_parameter('wheel_radius').value
        
        # Desired velocities
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Mecanum Inverse Kinematics
        # w_fl = (1/R) * (vx - vy - (Lx + Ly)*wz)
        # w_fr = (1/R) * (vx + vy + (Lx + Ly)*wz)
        # w_rl = (1/R) * (vx + vy - (Lx + Ly)*wz)
        # w_rr = (1/R) * (vx - vy + (Lx + Ly)*wz)
        
        k = Lx + Ly
        
        w_fl = (vx - vy - k * wz) / R
        w_fr = (vx + vy + k * wz) / R
        w_rl = (vx + vy - k * wz) / R
        w_rr = (vx - vy + k * wz) / R
        
        # Convert rad/s to approximately -100 to 100 scale for Hiwonder driver
        # Mapping: w (rad/s) -> RPM -> PWM/Speed Unit
        # 1 rad/s = 9.55 RPM
        
        def to_driver_units(rad_s):
            rpm = rad_s * 9.55
            max_rpm = self.get_parameter('max_rpm').value
            pwm_limit = self.get_parameter('pwm_limit').value
            
            # Normalize to -1.0 to 1.0
            norm = rpm / max_rpm
            
            # Scale to driver limit (e.g. 100)
            val = int(norm * pwm_limit)
            
            return max(min(val, pwm_limit), -pwm_limit)

        speeds = Int32MultiArray()
        speeds.data = [
            to_driver_units(w_fl), # FL
            to_driver_units(w_fr), # FR
            to_driver_units(w_rl), # RL
            to_driver_units(w_rr)  # RR
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
