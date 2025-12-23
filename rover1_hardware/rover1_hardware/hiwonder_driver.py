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

        # Encoder Reader
        self.create_timer(0.05, self.encoder_callback) # 20Hz
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'wheel_encoders', 10)
        
    def encoder_callback(self):
        if self.bus is None: return
        
        try:
            # Read 16 bytes starting at 0x3C
            # MOTOR_ENCODER_TOTAL_ADDR = 0x3C
            data = self.bus.read_i2c_block_data(self.address, 0x3C, 16)
            vals = struct.unpack('<iiii', bytes(data))
            
            # Mapping based on User Verification:
            # Index 0: Rear Left  (Motor 1) -> Fwd=Inc
            # Index 1: Front Left (Motor 2) -> Fwd=Dec
            # Index 2: Rear Right (Motor 3) -> Fwd=Dec
            # Index 3: Front Right(Motor 4) -> Fwd=Inc
            
            raw_rl = vals[0]
            raw_fl = vals[1]
            raw_rr = vals[2]
            raw_fr = vals[3]
            
            # Apply Polarity to match ROS Standard (Forward = Positive)
            # Rear Left: Inc -> Positive (No change)
            # Front Left: Dec -> Negative (Invert)
            # Rear Right: Dec -> Negative (Invert)
            # Front Right: Inc -> Positive (No change)
            
            enc_fl = raw_fl * -1
            enc_fr = raw_fr
            enc_rl = raw_rl
            enc_rr = raw_rr * -1
            
            msg = Int32MultiArray()
            msg.data = [enc_fl, enc_fr, enc_rl, enc_rr]
            self.encoder_pub.publish(msg)
            
        except Exception as e:
            # self.get_logger().warn(f'Encoder Read Error: {e}')
            pass

    def watchdog_callback(self):
        # If no message for 0.2s, stop motors
        safety_timeout = 0.2 # seconds
        time_diff = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        
        if time_diff > safety_timeout and self.motors_active:
            self.get_logger().warn('Watchdog: No command received, stopping motors.')
            self.stop_motors()

    def write_motor(self, register, speed):
        # Protocol: write_byte_data(addr, register, signed_byte_speed)
        if speed < 0:
            val = 256 + speed
        else:
            val = speed
        
        try:
            self.bus.write_byte_data(self.address, register, val)
        except Exception as e:
            self.get_logger().error(f'I2C Write Error (Reg {register}): {e}')

    def stop_motors(self):
        if self.bus is not None:
            # Stop all mapped registers
            # Mapped: 51=RL, 52=FL, 53=RR, 54=FR
            self.write_motor(51, 0)
            self.write_motor(52, 0)
            self.write_motor(53, 0)
            self.write_motor(54, 0)
            self.motors_active = False

    def listener_callback(self, msg):
        if self.bus is None:
            return

        self.last_msg_time = self.get_clock().now()
        self.motors_active = True

        if len(msg.data) != 4:
            self.get_logger().warn('Invalid wheel speed array length')
            return
        
        speeds = msg.data
        
        # Polarity
        fl_sign = -1 if self.get_parameter('invert_fl').value else 1
        fr_sign = -1 if self.get_parameter('invert_fr').value else 1
        rl_sign = -1 if self.get_parameter('invert_rl').value else 1
        rr_sign = -1 if self.get_parameter('invert_rr').value else 1
        
        # Constrain and Sign
        def process(idx, sign):
            raw = int(speeds[idx]) * sign
            return max(min(raw, 100), -100)

        speed_fl = process(0, fl_sign)
        speed_fr = process(1, fr_sign)
        speed_rl = process(2, rl_sign)
        speed_rr = process(3, rr_sign)

        # Mapping to Registers:
        # FL (Input 0) -> Reg 52
        # FR (Input 1) -> Reg 54
        # RL (Input 2) -> Reg 51
        # RR (Input 3) -> Reg 53
        
        self.write_motor(52, speed_fl)
        self.write_motor(54, speed_fr)
        self.write_motor(51, speed_rl)
        self.write_motor(53, speed_rr)

def main(args=None):
    rclpy.init(args=args)
    node = HiwonderDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
