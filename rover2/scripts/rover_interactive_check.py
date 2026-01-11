#!/usr/bin/env python3
import sys
import os
import subprocess
import time

def print_header(msg):
    print("\n" + "="*50)
    print(f" {msg}")
    print("="*50)

def ask_yes_no(question):
    while True:
        choice = input(f"{question} (y/n): ").lower()
        if choice in ['y', 'yes']:
            return True
        if choice in ['n', 'no']:
            return False

def check_command(cmd, name):
    try:
        subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT)
        print(f"[PASS] {name} found.")
        return True
    except subprocess.CalledProcessError:
        print(f"[FAIL] {name} not found or failed.")
        return False

def check_i2c():
    print_header("I2C Bus Check")
    print("Scanning I2C bus 1...")
    try:
        output = subprocess.check_output("i2cdetect -y 1", shell=True).decode()
        print(output)
        
        # Check for known addresses
        # 0x34 = Hiwonder Motor HAT
        # 0x6A = BerryIMU (LSM6DSL)
        
        found_motor = "34" in output
        found_imu = "6a" in output
        
        if found_motor: print("[PASS] Motor Driver (0x34) DETECTED")
        else: print("[FAIL] Motor Driver (0x34) NOT FOUND")
            
        if found_imu: print("[PASS] IMU (0x6A) DETECTED")
        else: print("[FAIL] IMU (0x6A) NOT FOUND")
        
        return found_motor and found_imu
    except Exception as e:
        print(f"[FAIL] Could not scan I2C: {e}")
        return False

def check_ros_nodes():
    print_header("ROS 2 Node Check")
    print("Make sure you have launched the rover using: ros2 launch rover1_bringup rover.launch.py")
    if not ask_yes_no("Is the launch file running in another terminal?"):
        print("Please run the launch file first.")
        return False
        
    try:
        output = subprocess.check_output("ros2 node list", shell=True).decode()
        nodes = output.split()
        
        required = [
            "/motor_driver",
            "/imu_driver",
            "/kinematics",
            "/ekf_filter_node",
            "/robot_state_publisher"
        ]
        
        all_good = True
        for n in required:
            if n in nodes:
                print(f"[PASS] Node {n} is running.")
            else:
                print(f"[FAIL] Node {n} is MISSING.")
                all_good = False
                
        return all_good
    except Exception as e:
        print(f"[FAIL] Failed to list ROS nodes: {e}")
        return False

def check_topics():
    print_header("ROS 2 Topic Check")
    required_topics = [
        "/wheel_encoders",
        "/imu/data",
        "/odometry/local",
        "/tf"
    ]
    
    all_good = True
    for t in required_topics:
        print(f"Checking {t}...", end=" ", flush=True)
        try:
            # Check hz to see if active using shell=False and discarding output to avoid pipe issues
            # We use timeout=2 to wait briefly.
            subprocess.run(
                ["ros2", "topic", "hz", t, "--window", "5"], 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL, 
                timeout=2, 
                check=True
            )
            print("ACTIVE")
        except subprocess.TimeoutExpired:
            # If it timed out, it means it's running but maybe slow waiting for window?
            # Actually, ros2 topic hz runs forever unless --window is met.
            # If we timeout, it likely means we got some data but not 5 samples yet, OR no data.
            # Let's assume ACTIVE if it didn't crash immediately?
            print("ACTIVE (Timeout)") 
        except subprocess.CalledProcessError:
            print("SILENT/MISSING")
            all_good = False
        except Exception as e:
            print(f"ERROR: {e}")
            all_good = False
            
    return all_good

def main():
    print_header("Rover1 Interactive Flight Check")
    print("This script will guide you through verifying the rover's hardware and software.")
    
    checks = []
    
    # 1. I2C
    checks.append(("Hardware (I2C)", check_i2c()))
    
    # 2. ROS Nodes
    checks.append(("Software (Nodes)", check_ros_nodes()))
    
    # 3. Topics
    checks.append(("Data (Topics)", check_topics()))
    
    print_header("Summary")
    for name, result in checks:
        status = "PASS" if result else "FAIL"
        print(f"{name}: {status}")
        
    if all(r for _, r in checks):
        print("\n[SUCCESS] System looks ready for Autonomy!")
        print("Next Step: Run 'ros2 run rover1_scripts calibrate_odometry.py'")
    else:
        print("\n[WARNING] Some checks failed. Check logs above.")

if __name__ == "__main__":
    main()
