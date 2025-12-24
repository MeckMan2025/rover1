#!/bin/bash
# Rover1 "Cockpit" Monitoring Dashboard v2.2 (Verified RTK Paths)
# Now checking /rtcm and /ntrip_client/rtcm specifically.

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[1;36m'
NC='\033[0m'

while true; do
    clear
    echo -e "${CYAN}================================================================${NC}"
    echo -e "${CYAN}                  🛸 ROVER1 SYSTEM MONITOR 🛸                  ${NC}"
    echo -e "${CYAN}================================================================${NC}"

    # 1. BRAIN STATUS
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

    # 3. NODE HEALTH
    echo -e "${CYAN}CORE NODE HEALTH:${NC}"
    NODES=("imu_driver" "motor_driver" "kinematics" "ekf_filter_node" "ekf_global_node" "navsat_transform" "foxglove_bridge" "stadia_teleop" "ublox_dgnss" "ntrip_client")
    
    ACTIVE_NODES=$(ros2 node list 2>/dev/null)
    for node in "${NODES[@]}"; do
        if echo "$ACTIVE_NODES" | grep -q "$node"; then
            printf "  %-18s [ ${GREEN}OK${NC} ]\n" "$node"
        else
            printf "  %-18s [ ${RED}MISSING${NC} ]\n" "$node"
        fi
    done

    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 4. GPS QUALITY (Timeout-protected)
    echo -en "${CYAN}GPS QUALITY:    ${NC}"
    FIX_TYPE=$(timeout 0.3 ros2 topic echo /ublox/nav_pvt --count 1 --field fix_type 2>/dev/null)
    case "$FIX_TYPE" in
        "3") echo -e "[ ${YELLOW}3 - STANDARD 3D${NC} ]" ;;
        "4") echo -e "[ ${GREEN}4 - RTK FIXED (OPTIMAL)${NC} ]" ;;
        "5") echo -e "[ ${CYAN}5 - RTK FLOAT (HIGH)${NC} ]" ;;
        "")  echo -e "[ ${RED}SEARCHING / NO PVT DATA${NC} ]" ;;
        *)   echo -e "[ ${RED}UNKNOWN (${FIX_TYPE})${NC} ]" ;;
    esac

    # 5. TOPIC HEARTBEATS
    echo -e "${CYAN}TOPIC HEARTBEETS (Last 0.5s):${NC}"
    timeout 0.3 ros2 topic hz /imu/data --window 1 2>/dev/null | grep "average rate" | awk '{print "  IMU DATA:       [ " $4 " Hz ]"}' || echo -e "  IMU DATA:       [ ${RED}STALLED${NC} ]"
    timeout 0.3 ros2 topic hz /fix --window 1 2>/dev/null | grep "average rate" | awk '{print "  GPS FIX:        [ " $4 " Hz ]"}' || echo -e "  GPS FIX:        [ ${RED}STALLED${NC} ]"
    
    # Check both potential RTCM paths
    timeout 0.3 ros2 topic hz /rtcm --window 1 2>/dev/null | grep "average rate" | awk '{print "  RTCM (NET):     [ " $4 " Hz ]"}' || \
    timeout 0.3 ros2 topic hz /ntrip_client/rtcm --window 1 2>/dev/null | grep "average rate" | awk '{print "  RTCM (NET):     [ " $4 " Hz ]"}' || \
    echo -e "  RTCM (NET):     [ ${RED}IDLE${NC} ]"
    
    timeout 0.3 ros2 topic hz /cmd_vel --window 1 2>/dev/null | grep "average rate" | awk '{print "  CMD_VEL:        [ " $4 " Hz ]"}' || echo -e "  CMD_VEL:        [ ${RED}IDLE${NC} ]"

    echo -e "${CYAN}================================================================${NC}"
    echo -e "Press Ctrl+C to exit monitor."
    sleep 1
done
