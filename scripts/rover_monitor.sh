#!/bin/bash
# Rover1 "Cockpit" Monitoring Dashboard v2.5 (High-Precision GPS Audit)

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null

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
    # Grabbing full NavPVT data at once for speed
    GPS_DATA=$(timeout 0.2 ros2 topic echo /ublox/nav_pvt --count 1 2>/dev/null)
    
    FIX_TYPE=$(echo "$GPS_DATA" | grep "fix_type" | awk '{print $2}')
    SATS=$(echo "$GPS_DATA" | grep "num_sv" | awk '{print $2}')
    H_ACC=$(echo "$GPS_DATA" | grep "h_acc" | awk '{print $2}')
    
    echo -en "${CYAN}GPS STATUS:     ${NC}"
    if [ -z "$FIX_TYPE" ]; then
        echo -e "[ ${RED}SEARCHING...${NC} ]"
    else
        case "$FIX_TYPE" in
            "3") echo -ne "[ ${YELLOW}3D FIX${NC} ]" ;;
            "4") echo -ne "[ ${GREEN}RTK FIXED${NC} ]" ;;
            "5") echo -ne "[ ${CYAN}RTK FLOAT${NC} ]" ;;
            *)   echo -ne "[ ${RED}NO FIX (${FIX_TYPE})${NC} ]" ;;
        esac
        # Add Satellite Count & Accuracy (H-Acc is in mm, converting to cm)
        ACC_CM=$(awk "BEGIN {print $H_ACC / 10}")
        echo -e " | ${YELLOW}SATS: ${SATS}${NC} | ${YELLOW}ACC: ${ACC_CM}cm${NC}"
    fi

    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 5. TOPIC HEARTBEATS
    echo -e "${CYAN}TOPIC HEARTBEETS (Last 0.2s):${NC}"
    timeout 0.2 ros2 topic hz /imu/data --window 1 2>/dev/null | grep "average rate" | awk '{print "  IMU DATA:       [ " $4 " Hz ]"}' || echo -e "  IMU DATA:       [ ${RED}STALLED${NC} ]"
    timeout 0.2 ros2 topic hz /fix --window 1 2>/dev/null | grep "average rate" | awk '{print "  GPS FIX:        [ " $4 " Hz ]"}' || echo -e "  GPS FIX:        [ ${RED}STALLED${NC} ]"
    
    # RTCM CHECK
    timeout 0.2 ros2 topic hz /ntrip_client/rtcm --window 1 2>/dev/null | grep "average rate" | awk '{print "  RTCM (NET):     [ " $4 " Hz ]"}' || \
    timeout 0.2 ros2 topic hz /rtcm --window 1 2>/dev/null | grep "average rate" | awk '{print "  RTCM (NET):     [ " $4 " Hz ]"}' || \
    echo -e "  RTCM (NET):     [ ${RED}IDLE${NC} ]"
    
    timeout 0.2 ros2 topic hz /cmd_vel --window 1 2>/dev/null | grep "average rate" | awk '{print "  CMD_VEL:        [ " $4 " Hz ]"}' || echo -e "  CMD_VEL:        [ ${RED}IDLE${NC} ]"

    echo -e "${CYAN}================================================================${NC}"
    echo -e "Press Ctrl+C to exit monitor."
    sleep 0.5
done
