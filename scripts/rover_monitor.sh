#!/bin/bash
# Rover1 "Cockpit" Monitoring Dashboard v2.6 (Hardware Audit Edition)

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

while true; do
    clear
    echo -e "${CYAN}================================================================${NC}"
    echo -e "${CYAN}                  🛸 ROVER1 SYSTEM MONITOR 🛸                  ${NC}"
    echo -e "${CYAN}================================================================${NC}"

    # 1. BRAIN STATUS
    STATUS=$(systemctl is-active rover1.service 2>/dev/null || echo "not-found")
    if [ "$STATUS" == "active" ]; then
        echo -e "SYSTEMD BRAIN:  [ ${GREEN}RUNNING${NC} ]"
    else
        echo -e "SYSTEMD BRAIN:  [ ${RED}FAILED/STOPPED (${STATUS})${NC} ]"
    fi

    # 2. NETWORK INFO
    IP=$(hostname -I | awk '{print $1}' 2>/dev/null)
    TEMP=$(vcgencmd measure_temp 2>/dev/null | cut -d'=' -f2)
    echo -e "NETWORK:        IP: ${YELLOW}${IP}${NC}  |  CPU TEMP: ${YELLOW}${TEMP}${NC}"
    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 3. NODE HEALTH
    echo -e "${CYAN}CORE NODE HEALTH:${NC}"
    NODES=("imu_driver" "motor_driver" "kinematics" "ekf_filter_node" "ekf_global_node" "navsat_transform" "foxglove_bridge" "stadia_teleop" "ublox_dgnss" "ntrip_client" "fix_to_nmea")
    
    ACTIVE_NODES=$(ros2 node list 2>/dev/null)
    for node in "${NODES[@]}"; do
        if echo "$ACTIVE_NODES" | grep -q "$node"; then
            printf "  %-18s [ ${GREEN}OK${NC} ]\n" "$node"
        else
            printf "  %-18s [ ${RED}MISSING${NC} ]\n" "$node"
        fi
    done

    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 4. GPS QUALITY & SAT AUDIT
    echo -en "${CYAN}GPS STATUS:     ${NC}"
    # Grab NavSatFix status
    FIX_DATA=$(timeout 0.5 ros2 topic echo /fix --count 1 2>/dev/null)
    if [ -z "$FIX_DATA" ]; then
        echo -e "[ ${RED}SEARCHING / NO ANTENNA LINK${NC} ]"
    else
        # Fix types are: -1=No, 0=Fix, 1=SBAS, 2=GBAS (RTK)
        STATUS_VAL=$(echo "$FIX_DATA" | grep "status" | head -n 1 | awk '{print $2}')
        case "$STATUS_VAL" in
            "0") echo -e "[ ${YELLOW}STANDARD 3D FIX${NC} ]" ;;
            "1") echo -e "[ ${CYAN}RTK FLOAT${NC} ]" ;;
            "2") echo -e "[ ${GREEN}RTK FIXED (OPTIMAL)${NC} ]" ;;
            *)   echo -e "[ ${RED}NO FIX (${STATUS_VAL})${NC} ]" ;;
        esac
    fi

    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 5. TOPIC HEARTBEATS (Increased timeout to 0.8s for Pi performance)
    echo -e "${CYAN}TOPIC HEARTBEETS (Hz):${NC}"
    
    timeout 0.8 ros2 topic hz /imu/data --window 2 2>/dev/null | grep "average rate" | awk '{print "  IMU DATA:       [ " $4 " Hz ]"}' || echo -e "  IMU DATA:       [ ${RED}STALLED${NC} ]"
    timeout 0.8 ros2 topic hz /fix --window 2 2>/dev/null | grep "average rate" | awk '{print "  GPS FIX:        [ " $4 " Hz ]"}' || echo -e "  GPS FIX:        [ ${RED}STALLED${NC} ]"
    
    # Check both potential RTCM paths
    timeout 0.8 ros2 topic hz /ntrip_client/rtcm --window 2 2>/dev/null | grep "average rate" | awk '{print "  RTCM (NET):     [ " $4 " Hz ]"}' || \
    timeout 0.8 ros2 topic hz /rtcm --window 2 2>/dev/null | grep "average rate" | awk '{print "  RTCM (NET):     [ " $4 " Hz ]"}' || \
    echo -e "  RTCM (NET):     [ ${RED}IDLE${NC} ]"
    
    timeout 0.8 ros2 topic hz /cmd_vel --window 2 2>/dev/null | grep "average rate" | awk '{print "  CMD_VEL:        [ " $4 " Hz ]"}' || echo -e "  CMD_VEL:        [ ${RED}IDLE${NC} ]"

    echo -e "${CYAN}================================================================${NC}"
    echo -e "Press Ctrl+C to exit monitor."
    sleep 5
done
