#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import struct
import smbus2

# LSM6DSL Registers
LSM6DSL_ADDRESS = 0x6A
LSM6DSL_CTRL1_XL = 0x10
LSM6DSL_CTRL2_G  = 0x11
LSM6DSL_CTRL3_C  = 0x12
LSM6DSL_OUTX_L_G = 0x22
LSM6DSL_OUTX_L_XL = 0x28

class BerryIMUDriver(Node):
    def __init__(self):
        super().__init__('berry_imu_driver')
        
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        # 50Hz (0.02s) matches typical ODR
        self.timer = self.create_timer(0.02, self.timer_callback) 
        
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        
        self.bus_num = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(f'Initializing BerryIMU on Bus {self.bus_num}...')
        
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            self.init_lsm6dsl()
            self.get_logger().info('LSM6DSL Initialized Successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C: {e}')
            self.bus = None

    def init_lsm6dsl(self):
        # Initialise the accelerometer
        # ODR = 104 Hz, FS = 4g
        # 0100 (104Hz) 10 (4g) 00
        # Hex: 0x4A
        self.bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL1_XL, 0x4A)
        
        # Initialise the gyroscope
        # ODR = 104 Hz, FS = 2000 dps
        # 0100 (104Hz) 11 (2000dps) 00
        # Hex: 0x4C
        self.bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL2_G, 0x4C)
        
        # IF_INC = 1 (Auto-increment register address)
        self.bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL3_C, 0x04)

        # Scales
        self.accel_scale = 0.122 / 1000.0 * 9.80665 # mg/LSB -> g -> m/s^2 (FS=4g)
        self.gyro_scale = 70.0 / 1000.0 * (math.pi / 180.0) # mdps/LSB -> dps -> rad/s (FS=2000dps)

    def read_word_2c(self, addr, reg):
        # Read two bytes: Low, High
        low = self.bus.read_byte_data(addr, reg)
        high = self.bus.read_byte_data(addr, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def timer_callback(self):
        if self.bus is None:
            return

        try:
            # Read GyroBytes (6 bytes starting at OUTX_L_G)
            # Using read_i2c_block_data for efficiency if supported, else word reads
            # We'll stick to robust word reads for now or block if possible
            # Block read is safer/faster
            
            # Note: smbus2 read_i2c_block_data returns a list
            # Gyro: X_L, X_H, Y_L, Y_H, Z_L, Z_H
            gyro_data = self.bus.read_i2c_block_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G, 6)
            accel_data = self.bus.read_i2c_block_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_XL, 6)
            
            # Helper to unpack
            def unpack_axis(data, offset):
                val = data[offset] | (data[offset+1] << 8)
                if val >= 32768: val -= 65536
                return val

            gx_raw = unpack_axis(gyro_data, 0)
            gy_raw = unpack_axis(gyro_data, 2)
            gz_raw = unpack_axis(gyro_data, 4)
            
            ax_raw = unpack_axis(accel_data, 0)
            ay_raw = unpack_axis(accel_data, 2)
            az_raw = unpack_axis(accel_data, 4)
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            # Convert raw to SI units
            # ENU Convention check:
            # Assume Sensor Matches Robot: +X fwd, +Y left, +Z up
            # Check markings on board. 
            # If standard BerryIMU:
            # Y is usually long axis, X is short? 
            # We will map 1:1 for now and user can calibrate/tf via base_link->imu_link rotation
            
            msg.angular_velocity.x = gx_raw * self.gyro_scale
            msg.angular_velocity.y = gy_raw * self.gyro_scale
            msg.angular_velocity.z = gz_raw * self.gyro_scale
            
            msg.linear_acceleration.x = ax_raw * self.accel_scale
            msg.linear_acceleration.y = ay_raw * self.accel_scale
            msg.linear_acceleration.z = az_raw * self.accel_scale
            
            # Orientation: No fusion yet, leave as 0 or incomplete
            # msg.orientation...
            msg.orientation_covariance[0] = -1.0 # Indicate no orientation estimate
            
            # Covariances (diagonal)
            # Values from datasheet or tuning. Starts conservative.
            msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
            ]
            
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'I2C Read Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BerryIMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
