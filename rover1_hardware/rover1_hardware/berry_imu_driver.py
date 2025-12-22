#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import smbus2
import time

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
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibrate_steps', 200)

        self.bus_num = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # State Variables
        self.last_time = self.get_clock().now()
        
        # Filter State (Euler Angles in Radians)
        # ENU: Roll (X), Pitch (Y), Yaw (Z)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Bias (Calculated on startup)
        self.gb = [0.0, 0.0, 0.0] # Gyro Bias X, Y, Z
        self.ab = [0.0, 0.0, 0.0] # Accel Bias (Not typically subtracted blindly, but useful)

        self.get_logger().info(f'Initializing BerryIMU on Bus {self.bus_num}...')
        
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            self.init_lsm6dsl()
            self.get_logger().info('LSM6DSL Hardware Initialized')
            
            # Perform Calibration
            self.calibrate_gyro(self.get_parameter('calibrate_steps').value)
            
            # Start Loop
            self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C: {e}')
            self.bus = None

    def init_lsm6dsl(self):
        # 104 Hz, 4g Accel
        self.bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL1_XL, 0x4A)
        # 104 Hz, 2000 dps Gyro
        self.bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL2_G, 0x4C)
        # Auto-increment
        self.bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL3_C, 0x04)

        # Scales
        # Accel: 0.122 mg/LSB -> g -> m/s^2
        self.accel_scale = 0.122 / 1000.0 * 9.80665 
        # Gyro: 70 mdps/LSB -> dps -> rad/s
        self.gyro_scale = 70.0 / 1000.0 * (math.pi / 180.0)

    def calibrate_gyro(self, steps):
        self.get_logger().info(f'Starting Calibration ({steps} steps). KEEP ROBOT STILL...')
        gx_sum, gy_sum, gz_sum = 0.0, 0.0, 0.0
        
        for _ in range(steps):
            data = self.bus.read_i2c_block_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G, 6)
            gx_sum += self.unpack_axis(data, 0)
            gy_sum += self.unpack_axis(data, 2)
            gz_sum += self.unpack_axis(data, 4)
            time.sleep(0.01)
            
        self.gb[0] = (gx_sum / steps) * self.gyro_scale
        self.gb[1] = (gy_sum / steps) * self.gyro_scale
        self.gb[2] = (gz_sum / steps) * self.gyro_scale
        
        self.get_logger().info(f'Calibration Complete. Bias: {self.gb}')

    def unpack_axis(self, data, offset):
        val = data[offset] | (data[offset+1] << 8)
        if val >= 32768: val -= 65536
        return val

    def euler_to_quaternion(self, r, p, y):
        qx = math.sin(r/2) * math.cos(p/2) * math.cos(y/2) - math.cos(r/2) * math.sin(p/2) * math.sin(y/2)
        qy = math.cos(r/2) * math.sin(p/2) * math.cos(y/2) + math.sin(r/2) * math.cos(p/2) * math.sin(y/2)
        qz = math.cos(r/2) * math.cos(p/2) * math.sin(y/2) - math.sin(r/2) * math.sin(p/2) * math.cos(y/2)
        qw = math.cos(r/2) * math.cos(p/2) * math.cos(y/2) + math.sin(r/2) * math.sin(p/2) * math.sin(y/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def timer_callback(self):
        if self.bus is None: return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        try:
            # Read Raw
            g_data = self.bus.read_i2c_block_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G, 6)
            a_data = self.bus.read_i2c_block_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_XL, 6)

            # Process Gyro (Subtract Bias)
            gx = (self.unpack_axis(g_data, 0) * self.gyro_scale) - self.gb[0]
            gy = (self.unpack_axis(g_data, 2) * self.gyro_scale) - self.gb[1]
            gz = (self.unpack_axis(g_data, 4) * self.gyro_scale) - self.gb[2]

            # Process Accel
            ax = self.unpack_axis(a_data, 0) * self.accel_scale
            ay = self.unpack_axis(a_data, 2) * self.accel_scale
            az = self.unpack_axis(a_data, 4) * self.accel_scale

            # Complementary Filter (Fusion)
            # 1. Calculate Pitch/Roll from Accel (Gravity Vector)
            # NOTE: Standard ENU on Pi: +X Fwd, +Y Left, +Z Up
            # Pitch is rotation around Y. Roll is rotation around X.
            
            # Roll (Around X axis)
            accel_roll = math.atan2(ay, az)
            
            # Pitch (Around Y axis)
            # Note: -ax because tilting nose-down (+pitch) creates -X gravity component in some frames.
            # We will adhere to ROS Rep 103: RHR
            accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

            # 2. Fuse
            alpha = 0.98
            self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * accel_roll
            self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * accel_pitch
            
            # 3. Yaw (Gyro Integration only for now)
            self.yaw += gz * dt
            
            # Create Message
            msg = Imu()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = self.frame_id

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

            # Covariances (Tuned for EKF)
            # Orientation: High confidence in Pitch/Roll, Low in Yaw (Drift)
            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.1  # Higher variance for Yaw
            ]
            
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'I2C Cycle Fail: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BerryIMUDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
