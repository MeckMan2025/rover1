# GNSS Health Monitor

A ROS 2 package that aggregates GPS/RTK/NTRIP status into a single clean topic for Foxglove dashboard visualization.

## Overview

This package subscribes to multiple GPS-related topics and publishes a unified health status message containing:
- Satellite counts (visible/used)
- NTRIP connection status  
- RTCM correction rates and age
- RTK status (NO_FIX, DGPS, FLOAT, FIXED)
- Position accuracy estimates

## Quick Start

### Build
```bash
colcon build --packages-select gnss_health_monitor
source install/setup.bash
```

### Run
```bash
# Launch with default parameters
ros2 launch gnss_health_monitor gnss_health_monitor.launch.py

# Launch with custom topics
ros2 launch gnss_health_monitor gnss_health_monitor.launch.py \
    navsat_topic:=/my_gps/fix \
    rtcm_topic:=/my_rtcm
```

### Test
```bash
# View health messages
ros2 topic echo /gnss/health

# Check message rate
ros2 topic hz /gnss/health
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `navsat_topic` | `/gps/filtered` | Primary NavSat topic |
| `navsat_fallback_topic` | `/fix` | Fallback NavSat topic |
| `navsat_timeout_s` | `2.0` | NavSat timeout (seconds) |
| `ubx_nav_sat_topic` | `/ubx_nav_sat` | u-blox satellite info topic |
| `rtcm_topic` | `/ntrip_client/rtcm` | Primary RTCM topic |
| `rtcm_fallback_topic` | `/rtcm` | Fallback RTCM topic |
| `rtcm_timeout_s` | `5.0` | RTCM connection timeout |
| `publish_rate_hz` | `5.0` | Health message publish rate |
| `rtk_fixed_h_acc_threshold_m` | `0.05` | RTK FIXED accuracy threshold |
| `rtk_float_h_acc_threshold_m` | `0.30` | RTK FLOAT accuracy threshold |

## Message Format

```
std_msgs/Header header
uint16 sat_visible              # Satellites visible
uint16 sat_used                 # Satellites used in solution
bool ntrip_connected            # NTRIP corrections flowing
uint32 rtcm_msgs_total          # Total RTCM messages received
float32 rtcm_msgs_per_sec       # RTCM message rate
float32 rtcm_bytes_per_sec      # RTCM data rate
float32 corr_age_s              # Correction age (seconds)
string rtk_state                # NO_FIX, DGPS, FLOAT, FIXED
float32 h_acc_m                 # Horizontal accuracy (meters)
float32 v_acc_m                 # Vertical accuracy (meters)
builtin_interfaces/Time last_update_time  # Last GPS update
uint16 dgps_id                  # DGPS station ID (-1 if unknown)
```

## Foxglove Integration

1. Connect to your robot's Foxglove bridge
2. Add panels for `/gnss/health`:
   - **Raw Messages**: View all fields
   - **Plot**: Track `h_acc_m`, `corr_age_s`, `sat_used`
   - **State Transitions**: Monitor `rtk_state` changes
   - **Table**: Display current values

## Troubleshooting

### No satellite data
- Check if `ublox_msgs` package is installed
- Verify `/ubx_nav_sat` topic exists
- Check node logs for message structure warnings

### No RTCM data  
- Verify RTCM topic names match your setup
- Check if `rtcm_msgs` package is available
- Monitor `/ntrip_client/rtcm` or `/rtcm` topics

### Accuracy shows NaN
- NavSat covariance may be invalid/zero
- Check GPS fix quality and RTK status
- Verify NavSat topic is publishing valid data

## Dependencies

- **Required**: `rclpy`, `std_msgs`, `sensor_msgs`, `builtin_interfaces`
- **Optional**: `ublox_msgs` (for satellite info), `rtcm_msgs` (for RTCM data)

## License

MIT