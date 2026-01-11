import smbus2
import time
import sys

# Hiwonder HAT Address
ADDRESS = 0x34
BUS = 1

def scan_registers(bus):
    values = {}
    try:
        # Read a safe range. 0-64 is typical for these HATs
        for reg in range(64):
            try:
                values[reg] = bus.read_byte_data(ADDRESS, reg)
            except:
                values[reg] = None
    except Exception as e:
        pass
    return values

def main():
    bus = smbus2.SMBus(BUS)
    print("Interactive Encoder Mapper")
    print("--------------------------")
    
    wheels = ["Front Left", "Front Right", "Rear Left", "Rear Right"]
    
    for wheel in wheels:
        print(f"\n>> Please spin the **{wheel}** wheel manually now...")
        print("   (Scanning for 5 seconds)")
        
        # Take a baseline
        baseline = scan_registers(bus)
        
        # Monitor for changes
        changes = {}
        start = time.time()
        while time.time() - start < 5.0:
            current = scan_registers(bus)
            for reg, val in current.items():
                if val is not None and baseline.get(reg) is not None:
                    if val != baseline[reg]:
                        if reg not in changes:
                            changes[reg] = []
                        changes[reg].append(val)
            time.sleep(0.05)
            
        if not changes:
            print("   No registers changed.")
        else:
            print(f"   Registers that changed for {wheel}:")
            for reg, vals in changes.items():
                unique = list(set(vals))
                print(f"     - Reg {reg}: {unique[:5]}")

if __name__ == "__main__":
    main()
