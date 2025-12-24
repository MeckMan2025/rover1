#!/bin/bash
# Rover1 "Cockpit" Monitoring Dashboard v2.8 (RTK Edition)

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
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

    # 2. NETWORK & POWER INFO
    IP=$(hostname -I | awk '{print $1}' 2>/dev/null)
    TEMP=$(vcgencmd measure_temp 2>/dev/null | cut -d'=' -f2)

    # Battery voltage from /battery_voltage topic
    BATT_V=$(timeout 2 ros2 topic echo /battery_voltage --once 2>/dev/null | grep "data:" | awk '{printf "%.1f", $2}')
    if [ -n "$BATT_V" ]; then
        # Color code: green > 14V, yellow 12-14V, red < 12V
        if (( $(echo "$BATT_V > 14.0" | bc -l) )); then
            BATT_STATUS="${GREEN}${BATT_V}V${NC}"
        elif (( $(echo "$BATT_V > 12.0" | bc -l) )); then
            BATT_STATUS="${YELLOW}${BATT_V}V${NC}"
        else
            BATT_STATUS="${RED}${BATT_V}V LOW!${NC}"
        fi
    else
        BATT_STATUS="${RED}--${NC}"
    fi

    echo -e "NETWORK:        IP: ${YELLOW}${IP}${NC}  |  CPU: ${YELLOW}${TEMP}${NC}  |  BATT: ${BATT_STATUS}"
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

    # 4. GPS & RTK STATUS (using /ubx_nav_pvt for accurate RTK info)
    echo -e "${CYAN}GPS & RTK STATUS:${NC}"
    PVT_DATA=$(timeout 3 ros2 topic echo /ubx_nav_pvt --once 2>/dev/null)

    if [ -z "$PVT_DATA" ]; then
        echo -e "  FIX TYPE:       [ ${RED}NO GPS DATA${NC} ]"
        echo -e "  RTK STATUS:     [ ${RED}OFFLINE${NC} ]"
        echo -e "  SATELLITES:     [ ${RED}--${NC} ]"
        echo -e "  ACCURACY:       [ ${RED}--${NC} ]"
    else
        # Extract values
        FIX_TYPE=$(echo "$PVT_DATA" | grep "fix_type:" | awk '{print $2}')
        DIFF_SOLN=$(echo "$PVT_DATA" | grep "diff_soln:" | awk '{print $2}')
        CARR_SOLN=$(echo "$PVT_DATA" | grep -A1 "carr_soln:" | grep "status:" | awk '{print $2}')
        NUM_SV=$(echo "$PVT_DATA" | grep "num_sv:" | awk '{print $2}')
        H_ACC=$(echo "$PVT_DATA" | grep "h_acc:" | awk '{print $2}')

        # Fix Type
        case "$FIX_TYPE" in
            "0") echo -e "  FIX TYPE:       [ ${RED}NO FIX${NC} ]" ;;
            "2") echo -e "  FIX TYPE:       [ ${YELLOW}2D FIX${NC} ]" ;;
            "3") echo -e "  FIX TYPE:       [ ${GREEN}3D FIX${NC} ]" ;;
            *)   echo -e "  FIX TYPE:       [ ${RED}UNKNOWN ($FIX_TYPE)${NC} ]" ;;
        esac

        # RTK Status (the important one!)
        if [ "$DIFF_SOLN" == "true" ]; then
            case "$CARR_SOLN" in
                "0") echo -e "  RTK STATUS:     [ ${YELLOW}DGPS${NC} ]" ;;
                "1") echo -e "  RTK STATUS:     [ ${CYAN}RTK FLOAT${NC} ] (~5cm)" ;;
                "2") echo -e "  RTK STATUS:     [ ${GREEN}RTK FIXED${NC} ] (~2cm)" ;;
                *)   echo -e "  RTK STATUS:     [ ${YELLOW}DGPS${NC} ]" ;;
            esac
        else
            echo -e "  RTK STATUS:     [ ${RED}NO CORRECTIONS${NC} ]"
        fi

        # Satellites
        if [ -n "$NUM_SV" ]; then
            if [ "$NUM_SV" -ge 20 ]; then
                echo -e "  SATELLITES:     [ ${GREEN}${NUM_SV} SVs${NC} ]"
            elif [ "$NUM_SV" -ge 10 ]; then
                echo -e "  SATELLITES:     [ ${YELLOW}${NUM_SV} SVs${NC} ]"
            else
                echo -e "  SATELLITES:     [ ${RED}${NUM_SV} SVs${NC} ]"
            fi
        fi

        # Accuracy (h_acc is in mm)
        if [ -n "$H_ACC" ]; then
            if [ "$H_ACC" -lt 50 ]; then
                echo -e "  ACCURACY:       [ ${GREEN}${H_ACC}mm${NC} ] (RTK quality)"
            elif [ "$H_ACC" -lt 500 ]; then
                echo -e "  ACCURACY:       [ ${CYAN}${H_ACC}mm${NC} ] (sub-meter)"
            elif [ "$H_ACC" -lt 2000 ]; then
                echo -e "  ACCURACY:       [ ${YELLOW}${H_ACC}mm${NC} ] (meter-level)"
            else
                echo -e "  ACCURACY:       [ ${RED}${H_ACC}mm${NC} ] (poor)"
            fi
        fi
    fi

    echo -e "${CYAN}----------------------------------------------------------------${NC}"

    # 5. TOPIC HEARTBEATS (simple alive check with generous timeouts)
    echo -e "${CYAN}TOPIC HEARTBEATS:${NC}"

    # IMU - check if publishing (3s timeout for DDS discovery)
    IMU_CHECK=$(timeout 3 ros2 topic echo /imu/data --once 2>/dev/null | head -1)
    if [ -n "$IMU_CHECK" ]; then
        echo -e "  IMU DATA:       [ ${GREEN}ACTIVE${NC} ]"
    else
        echo -e "  IMU DATA:       [ ${RED}STALLED${NC} ]"
    fi

    # GPS - must use best_effort QoS (3s timeout)
    GPS_CHECK=$(timeout 3 ros2 topic echo /fix --qos-reliability best_effort --once 2>/dev/null | head -1)
    if [ -n "$GPS_CHECK" ]; then
        echo -e "  GPS FIX:        [ ${GREEN}ACTIVE${NC} ]"
    else
        echo -e "  GPS FIX:        [ ${RED}STALLED${NC} ]"
    fi

    # RTCM - check if corrections flowing (3s timeout)
    RTCM_CHECK=$(timeout 3 ros2 topic echo /ntrip_client/rtcm --once 2>/dev/null | head -1)
    if [ -n "$RTCM_CHECK" ]; then
        echo -e "  RTCM (NTRIP):   [ ${GREEN}ACTIVE${NC} ]"
    else
        echo -e "  RTCM (NTRIP):   [ ${YELLOW}WAITING${NC} ]"
    fi

    # CMD_VEL - only active when controller used (1s is fine, it's fast)
    CMD_CHECK=$(timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null | head -1)
    if [ -n "$CMD_CHECK" ]; then
        echo -e "  CMD_VEL:        [ ${GREEN}ACTIVE${NC} ]"
    else
        echo -e "  CMD_VEL:        [ ${YELLOW}IDLE${NC} ]"
    fi

    echo -e "${CYAN}================================================================${NC}"
    echo -e "Press Ctrl+C to exit monitor. Refresh: 5s"
    sleep 5
done
