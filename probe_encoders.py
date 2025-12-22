import smbus2
import time
import sys

# Hiwonder HAT Address
ADDRESS = 0x34
BUS = 1

def scan_registers(bus):
    # Read registers 0-60
    values = {}
    try:
        # standard Byte read
        for reg in range(60):
            try:
                values[reg] = bus.read_byte_data(ADDRESS, reg)
            except:
                values[reg] = None
    except Exception as e:
        print(f"Error reading: {e}")
    return values

def main():
    bus = smbus2.SMBus(BUS)
    print("Probe Encoders Tool")
    print("-------------------")
    print("1. Spinning Rear Left Motor (Reg 51) at Speed 40...")
    
    # Start Motor
    try:
        bus.write_byte_data(ADDRESS, 51, 40)
    except Exception as e:
        print(f"Failed to write motor: {e}")
        return

    print("2. Scanning registers for changes (5 seconds)...")
    
    history = []
    start_time = time.time()
    while time.time() - start_time < 5.0:
        history.append(scan_registers(bus))
        time.sleep(0.1)
        
    # Stop Motor
    print("3. Stopping Motor...")
    bus.write_byte_data(ADDRESS, 51, 0)
    
    # Analyze
    print("4. Analysis:")
    changing_regs = []
    
    # Check each register
    for reg in range(60):
        vals = [snapshot.get(reg) for snapshot in history if snapshot.get(reg) is not None]
        if not vals: continue
        
        # Check if values changed over time
        unique_vals = set(vals)
        if len(unique_vals) > 1:
            print(f" -> Register {reg} CHANGED! Values: {list(unique_vals)[:5]}...")
            changing_regs.append(reg)
            
    if not changing_regs:
        print("No registers changed value. Are you sure encoders are active?")
    else:
        print(f"\nPotential Encoder Registers: {changing_regs}")

if __name__ == "__main__":
    main()
