import smbus2
import time
import sys

bus = smbus2.SMBus(1)
ADDRESS = 0x34

def write_motor(register, speed):
    try:
        # Convert -100/100 to byte
        if speed < 0:
            val = 256 + speed
        else:
            val = speed
        
        # Try writing single byte first (Protocol A)
        bus.write_byte_data(ADDRESS, register, val)
        print(f"Wrote Speed {speed} to Register {register}")
    except Exception as e:
        print(f"Error: {e}")

def stop_all():
    print("Stopping all known registers (51-54)...")
    for r in range(51, 55):
        write_motor(r, 0)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 probe_motors.py [REGISTER] [SPEED]")
        print("Example: python3 probe_motors.py 51 50  (Spin M1? at 50)")
        sys.exit(1)

    reg = int(sys.argv[1])
    spd = int(sys.argv[2])
    
    try:
        write_motor(reg, spd)
        time.sleep(2)
        write_motor(reg, 0)
        print("Done.")
    except KeyboardInterrupt:
        stop_all()
