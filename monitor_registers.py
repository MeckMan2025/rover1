import smbus2
import time
import sys
import os

ADDRESS = 0x34
BUS = 1

def main():
    bus = smbus2.SMBus(BUS)
    print("Live Register Monitor")
    print("---------------------")
    print("Trying to find encoders. Watching registers 0-20...")
    print("Columns: [Reg 0] [Reg 1] [Reg 2] ...")
    
    try:
        while True:
            # Read first 16 registers
            row = []
            for r in range(16):
                try:
                    val = bus.read_byte_data(ADDRESS, r)
                    row.append(f"{val:3}")
                except:
                    row.append("XXX")
            
            # Print in place (carriage return)
            print(" | ".join(row), end='\r')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nstopped.")

if __name__ == "__main__":
    main()
