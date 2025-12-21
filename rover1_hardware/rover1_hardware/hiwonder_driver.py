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

    def listener_callback(self, msg):
        if self.bus is None:
            return

        if len(msg.data) != 4:
            self.get_logger().warn('Invalid wheel speed array length')
            return

        # Motors map: 1=M1, 2=M2, 3=M3, 4=M4
        # Assuming typical Mecanum layout input: FL, FR, RL, RR
        # Hiwonder usually marks M1..M4 on board. 
        # We'll map Input[0]->M1, Input[1]->M2, etc. and let calibration fix direction.
        
        speeds = msg.data # Expected values approx -100 to 100
        
        for i in range(4):
            motor_id = i + 1
            handlers_speed = int(speeds[i])
            
            # Constrain speed to typical byte range if needed, 
            # though this specific register likely takes a signed 16-bit or 8-bit value.
            # Based on search, register 51 takes speed. 
            # Protocol usually: [Register, MotorID, Speed_Low, Speed_High] or similar.
            # Without exact datasheet, we assume a common Hiwonder protocol:
            # write_i2c_block_data(addr, register, [motor_id, speed])
            
            try:
                # Based on common Hiwonder I2C robot protocol found in searches:
                # Reg 51 (Fixed Speed): [MotorID, Speed(Signed 16-bit?)]
                # If just speed, checking if we write 4 bytes in one go or 1 by 1.
                # Heuristic: Write individual motor speed.
                
                # Handling signed integer packing
                # If the register expects 1 byte speed (-100 to 100)
                if handlers_speed > 100: handlers_speed = 100
                if handlers_speed < -100: handlers_speed = -100
                
                if handlers_speed < 0:
                    speed_packed = 256 + handlers_speed # 2's complement for 8-bit
                else:
                    speed_packed = handlers_speed
                    
                # The search result said "Register 51".
                # Often: bus.write_i2c_block_data(0x34, 51, [motor_id, speed_packed])
                self.bus.write_i2c_block_data(self.address, 51, [motor_id, speed_packed])
                
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
