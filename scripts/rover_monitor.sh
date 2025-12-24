#!/bin/bash
# Rover1 "Cockpit" Monitoring Dashboard
# Run this in a separate terminal via: ./scripts/rover_monitor.sh

# Colors for the dashboard
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

while true; do
    clear
    echo -e "${CYAN}================================================================${NC}"
    echo -e "${CYAN}                  🛸 ROVER1 SYSTEM MONITOR 🛸                  ${NC}"
    echo -e "${CYAN}================================================================${NC}"

    # 1. BRAIN STATUS (Systemd)
    STATUS=$(systemctl is-active rover1.service)
    if [ "$STATUS" == "active" ]; then
        echo -e "SYSTEMD BRAIN:  [ ${GREEN}RUNNING${NC} ]"
    else
        echo -e "SYSTEMD BRAIN:  [ ${RED}FAILED/STOPPED (${STATUS})${NC} ]"
    fi

    # 2. NETWORK INFO
    IP=$(hostname -I | awk '{print $1}')
    TEMP=$(vcgencmd measure_temp | cut -d'=' -f2)
    echo -e "NETWORK:        IP: ${YELLOW}${IP}${NC}  |  CPU TEMP: ${YELLOW}${TEMP}${NC}"
    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 3. NODE HEALTH CHECK
    echo -e "${CYAN}CORE NODE HEALTH:${NC}"
    NODES=("imu_driver" "motor_driver" "kinematics" "ekf_filter_node" "ekf_global_node" "navsat_transform" "foxglove_bridge" "stadia_teleop" "ublox_dgnss")
    
    ACTIVE_NODES=$(ros2 node list 2>/dev/null)
    for node in "${NODES[@]}"; do
        if echo "$ACTIVE_NODES" | grep -q "$node"; then
            printf "  %-18s [ ${GREEN}OK${NC} ]\n" "$node"
        else
            printf "  %-18s [ ${RED}MISSING${NC} ]\n" "$node"
        fi
    done

    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 4. TOPIC HEARTBEAT (Hz Check)
    echo -e "${CYAN}TOPIC HEARTBEETS (Last 1s):${NC}"
    # Use a quick timeout for Hz checks
    timeout 0.5 ros2 topic hz /imu/data --window 1 2>/dev/null | grep "average rate" | awk '{print "  IMU DATA:       [ " $4 " Hz ]"}' || echo -e "  IMU DATA:       [ ${RED}STALLED${NC} ]"
    timeout 0.5 ros2 topic hz /fix --window 1 2>/dev/null | grep "average rate" | awk '{print "  GPS FIX:        [ " $4 " Hz ]"}' || echo -e "  GPS FIX:        [ ${RED}STALLED${NC} ]"
    timeout 0.5 ros2 topic hz /battery_voltage --window 1 2>/dev/null | grep "average rate" | awk '{print "  BATTERY:        [ " $4 " Hz ]"}' || echo -e "  BATTERY:        [ ${RED}STALLED${NC} ]"
    timeout 0.5 ros2 topic hz /cmd_vel --window 1 2>/dev/null | grep "average rate" | awk '{print "  CMD_VEL:        [ " $4 " Hz ]"}' || echo -e "  CMD_VEL:        [ ${RED}IDLE${NC} ]"

    echo -e "${CYAN}================================================================${NC}"
    echo -e "Press Ctrl+C to exit monitor."
    sleep 2
done
