#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
import time
import math

class OdomCalibrator(Node):
    def __init__(self):
        super().__init__('odom_calibrator')
        
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_cb,
            10)
            
        self.start_ticks = None
        self.current_ticks = None
        self.collecting = False
        
        print("\n=== Odometry Calibrator ===")
        print("1. Mark the robot's front wheel position on the floor.")
        print("2. Measure exactly 1.0 meter forward and mark the target line.")
        print("3. Press ENTER to start recording start ticks.")
        input("> ")
        self.collecting = True
        print("Recording initial position... Drive exactly 1.0m forward.")
        
    def encoder_cb(self, msg):
        if not self.collecting: return
        
        self.current_ticks = msg.data
        if self.start_ticks is None:
            self.start_ticks = list(msg.data)
            print(f"Start Ticks: {self.start_ticks}")

    def finish(self):
        if self.start_ticks is None or self.current_ticks is None:
            print("No data received!")
            return

        end_ticks = self.current_ticks
        
        # Calculate Average Delta
        deltas = [abs(e - s) for e, s in zip(end_ticks, self.start_ticks)]
        avg_ticks = sum(deltas) / 4.0
        
        print("\n=== Results ===")
        print(f"Start: {self.start_ticks}")
        print(f"End:   {end_ticks}")
        print(f"Deltas: {deltas}")
        print(f"Average Delta Ticks for 1.0m: {avg_ticks:.2f}")
        
        # Formula: dist = (delta_ticks / ticks_per_rev) * (2 * pi * radius)
        # 1.0 = (avg_ticks / X) * (2 * pi * 0.04)
        # X = avg_ticks * (2 * pi * 0.04)
        
        wheel_radius = 0.04 # m
        wheel_circumference = 2 * math.pi * wheel_radius
        
        new_ticks_per_rev = avg_ticks * wheel_circumference
        
        print(f"Estimated ticks_per_rev: {new_ticks_per_rev:.2f}")
        print("Update this value in mecanum_kinematics.py or launch file.")

def main(args=None):
    rclpy.init(args=args)
    node = OdomCalibrator()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
