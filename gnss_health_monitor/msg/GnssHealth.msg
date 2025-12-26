# GNSS Health Status Message for Foxglove Dashboard
# Aggregates GPS/RTK/NTRIP status into a single clean topic

std_msgs/Header header

# Satellite Information
uint16 sat_visible          # Number of satellites visible
uint16 sat_used             # Number of satellites used in navigation solution

# NTRIP/Corrections Status
bool ntrip_connected        # True if RTCM corrections are flowing
uint32 rtcm_msgs_total      # Total RTCM messages received (monotonic counter)
float32 rtcm_msgs_per_sec   # RTCM message rate (rolling average)
float32 rtcm_bytes_per_sec  # RTCM data rate in bytes/sec (rolling average)
float32 corr_age_s          # Seconds since last RTCM correction received

# RTK Status
string rtk_state            # NO_FIX, DGPS, FLOAT, FIXED
float32 h_acc_m             # Horizontal accuracy estimate (meters)
float32 v_acc_m             # Vertical accuracy estimate (meters)

# Enhanced Status (suggested upgrades)
builtin_interfaces/Time last_update_time  # Timestamp of last valid GPS data
uint16 dgps_id              # DGPS station ID if available (0 if unknown)