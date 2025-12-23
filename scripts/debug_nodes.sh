#!/bin/bash
echo ">>> Debugging Missing Nodes..."
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "--- 1. Testing xacro ---"
xacro ~/ros2_ws/src/rover1/rover1_description/urdf/rover.urdf.xacro > /dev/null
if [ $? -eq 0 ]; then
    echo "[PASS] Xacro conversion successful."
else
    echo "[FAIL] Xacro failed!"
    xacro ~/ros2_ws/src/rover1/rover1_description/urdf/rover.urdf.xacro
fi

echo "--- 2. Testing Mecanum Kinematics ---"
# Run for 2 seconds then kill
timeout 2s ros2 run rover1_hardware mecanum_kinematics
RET=$?
if [ $RET -eq 124 ]; then
    echo "[PASS] Kinematics started and ran for 2s"
elif [ $RET -eq 0 ]; then
     echo "[WARN] Kinematics exited immediately (code 0)?"
else
    echo "[FAIL] Kinematics crashed with code $RET"
fi

echo "--- 3. Testing EKF Node ---"
if [ -f ~/ros2_ws/install/rover1_bringup/share/rover1_bringup/config/ekf.yaml ]; then
    echo "[PASS] ekf.yaml found."
else
    echo "[FAIL] ekf.yaml MISSING!"
fi

echo "--- 4. Testing Motor Driver & Battery ---"
# Run for 2 seconds then kill
timeout 2s ros2 run rover1_hardware hiwonder_driver > /tmp/driver_log.txt 2>&1
RET=$?
if [ $RET -eq 124 ]; then
    echo "[PASS] Motor Driver started successfully."
    grep "battery_state" /tmp/driver_log.txt > /dev/null
    # Since topic listing requires a running node, we just check if it crashed
    echo "[INFO] Checking for errors in driver..."
    if grep -q "ImportError\|AttributeError\|NameError" /tmp/driver_log.txt; then
        echo "[FAIL] Driver crashed with error:"
        grep "ImportError\|AttributeError\|NameError" /tmp/driver_log.txt
    else
        echo "[PASS] No common Python errors in driver startup."
    fi
else
    echo "[FAIL] Motor Driver failed to start or crashed (Code $RET)"
    cat /tmp/driver_log.txt
fi

echo ">>> Debug Complete"
