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
# Check if hardware node is already running in background
RUNNING_NODES=$(ros2 node list)
if echo "$RUNNING_NODES" | grep -q 'motor_driver\|hiwonder_driver'; then
    echo "[PASS] Motor driver node is running."
else
    echo "[INFO] Running hiwonder_driver manually to test..."
    timeout 5s ros2 run rover1_hardware hiwonder_driver > /tmp/driver_log.txt 2>&1 &
    PID=$!
    sleep 4
    kill $PID 2>/dev/null
fi

ros2 topic list | grep -i "battery" > /tmp/topic_check.txt
if [ -s /tmp/topic_check.txt ]; then
    echo "[PASS] battery_state topic found."
    timeout 2s ros2 topic echo /battery_state --once
else
    echo "[FAIL] battery_state topic NOT FOUND."
    echo "[INFO] Last 20 lines of driver logs:"
    cat /tmp/driver_log.txt | tail -n 20
fi

echo ">>> Debug Complete"
