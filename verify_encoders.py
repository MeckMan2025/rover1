import smbus2
import time
import struct

ADDRESS = 0x34
MOTOR_ENCODER_TOTAL_ADDR = 0x3C

def main():
    bus = smbus2.SMBus(1)
    print("Encoder Verification Tool")
    print("-------------------------")
    print("Reading 16 bytes from Register 0x3C...")
    print("Columns: [Motor 1] [Motor 2] [Motor 3] [Motor 4]")
    
    try:
        while True:
            try:
                # Read 16 bytes
                # Note: SMBus block read is limited to 32 bytes usually, so 16 is fine.
                data = bus.read_i2c_block_data(ADDRESS, MOTOR_ENCODER_TOTAL_ADDR, 16)
                
                # Unpack 4 signed integers (Little Endian)
                # 'iiii' = 4x 32-bit integers
                encoders = struct.unpack('<iiii', bytes(data))
                
                print(f"Encoders: {encoders[0]:10} {encoders[1]:10} {encoders[2]:10} {encoders[3]:10}", end='\r')
                
            except Exception as e:
                pass
                
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
