#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import smbus2
import struct

class HiwonderDriver(Node):
    def __init__(self):
        super().__init__('hiwonder_driver')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x34)
        
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        
        # Topics
        # Subscribes to wheel speeds in pulses/10ms (approx raw unit for this driver)
        # Layout: [FrontLeft, FrontRight, RearLeft, RearRight]
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'wheel_speeds_raw',
            self.listener_callback,
            10)
            
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            self.get_logger().info(f'Connected to Hiwonder Hardware at 0x{self.address:02X}')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C bus: {e}')
            self.bus = None


        # Safety Features
        self.last_msg_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_callback)
        self.motors_active = False
        
        # Polarity (1 or -1)
        self.declare_parameter('invert_fl', False)
        self.declare_parameter('invert_fr', False)
        self.declare_parameter('invert_rl', False)
        self.declare_parameter('invert_rr', False)

    def watchdog_callback(self):
        # If no message for 0.5s, stop motors
        safety_timeout = 0.5 # seconds
        time_diff = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        
        if time_diff > safety_timeout and self.motors_active:
            self.get_logger().warn('Watchdog: No command received, stopping motors.')
            self.stop_motors()

    def stop_motors(self):
        if self.bus is not None:
            try:
                # Stop all
                for i in range(4):
                    self.write_motor(i + 1, 0)
                self.motors_active = False
            except Exception as e:
                self.get_logger().error(f'Stop Error: {e}')

    def write_motor(self, motor_id, speed):
        # Send to hardware
        if speed < 0:
            speed_packed = 256 + speed
        else:
            speed_packed = speed
            
        self.bus.write_i2c_block_data(self.address, 51, [motor_id, speed_packed])

    def listener_callback(self, msg):
        if self.bus is None:
            return

        self.last_msg_time = self.get_clock().now()
        self.motors_active = True

        if len(msg.data) != 4:
            self.get_logger().warn('Invalid wheel speed array length')
            return
        
        speeds = msg.data
        
        # Apply Polarity
        fl_sign = -1 if self.get_parameter('invert_fl').value else 1
        fr_sign = -1 if self.get_parameter('invert_fr').value else 1
        rl_sign = -1 if self.get_parameter('invert_rl').value else 1
        rr_sign = -1 if self.get_parameter('invert_rr').value else 1
        
        signs = [fl_sign, fr_sign, rl_sign, rr_sign]
        
        for i in range(4):
            motor_id = i + 1
            raw_speed = int(speeds[i]) * signs[i]
            
            # Constrain -100 to 100
            constrained_speed = max(min(raw_speed, 100), -100)
            
            try:
                self.write_motor(motor_id, constrained_speed)
            except Exception as e:
                self.get_logger().error(f'I2C Write Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HiwonderDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
