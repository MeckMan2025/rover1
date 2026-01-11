#!/usr/bin/env python3
import time
import smbus2
import struct

# Registers
LSM6DSL_ADDRESS = 0x6A
LSM6DSL_CTRL1_XL = 0x10
LSM6DSL_CTRL2_G  = 0x11
LSM6DSL_OUTX_L_G = 0x22
LSM6DSL_OUTX_L_XL = 0x28

def init_sensor(bus):
    # Init similar to driver
    bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL1_XL, 0x4A) # 4g
    bus.write_byte_data(LSM6DSL_ADDRESS, LSM6DSL_CTRL2_G, 0x4C) # 2000dps

def read_raw(bus):
    gyro_data = bus.read_i2c_block_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G, 6)
    accel_data = bus.read_i2c_block_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_XL, 6)
    
    def unpack(data, i):
        val = data[i] | (data[i+1] << 8)
        if val >= 32768: val -= 65536
        return val

    gx = unpack(gyro_data, 0)
    gy = unpack(gyro_data, 2)
    gz = unpack(gyro_data, 4)
    ax = unpack(accel_data, 0)
    ay = unpack(accel_data, 2)
    az = unpack(accel_data, 4)
    return gx, gy, gz, ax, ay, az

def main():
    bus = smbus2.SMBus(1)
    init_sensor(bus)
    print("Calibrating Gyroscope. Keep robot stationary for 10 seconds...")
    time.sleep(1)
    
    gx_sum = 0
    gy_sum = 0
    gz_sum = 0
    count = 0
    start = time.time()
    
    while time.time() - start < 10:
        gx, gy, gz, _, _, _ = read_raw(bus)
        gx_sum += gx
        gy_sum += gy
        gz_sum += gz
        count += 1
        time.sleep(0.02)
        
    print(f"Samples: {count}")
    print(f"Gyro X Offset: {gx_sum/count:.2f}")
    print(f"Gyro Y Offset: {gy_sum/count:.2f}")
    print(f"Gyro Z Offset: {gz_sum/count:.2f}")
    print("Update these values in your driver if needed.")

if __name__ == "__main__":
    main()
