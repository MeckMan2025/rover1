# Vendor Sample Code & Duplicate Infrastructure Archive

**Date Archived:** 2026-01-11  
**Reason:** Codebase simplification - removing unused vendor samples and duplicate infrastructure

## What Was Archived

### YahBoom Hardware Samples
- `yahboomcar_astra/` - Depth camera tutorial demos
- `yahboomcar_visual/` - Computer vision demos (QR codes, AR, object detection)
- `yahboomcar_kcftracker/` - KCF tracking algorithm demos
- `yahboomcar_slam/` - SLAM tutorial implementations
- `yahboomcar_msgs/` - Custom message definitions for demos
- `yahboomcar_mediapipe/` - MediaPipe integration demos

### ROS 2 Learning Materials
- `ros2_ws/` - Complete ROS 2 tutorial workspace with learning packages
- `yahboomcar_ros2_ws/` - YahBoom-specific tutorial workspace

### ORB-SLAM2 Demos
- `ros2-ORB_SLAM2/` - Vendor-provided SLAM implementation

### Rover2 Duplicates (Removed 2026-01-11)
- `rover2_Camera_Specs/` - Complete duplicate of Camera_Specs from rover2/
- **Note:** Complete `rover2/` directory containing 496 duplicate files was removed from active codebase

## What Remains in Production

### Active Camera Driver
- `Camera_Specs/ascam_ros2_ws/src/ascamera/` - Production camera driver
  - Used by `rover.launch.py` with namespace `ascamera_hp60c`
  - Supports Nuwa-HP60C camera hardware
  - Required for rover vision systems

### Active Rover1 Packages
- `rover1_bringup/` - Main rover launch configurations
- `rover1_patrol/` - Autonomous patrol and navigation
- `rover1_hardware/` - Hardware drivers and interfaces
- `rover1_vision/` - Production vision systems
- `gnss_web_dashboard/` - Web interface and monitoring

## Cumulative Impact

| Phase | Files Removed | Total Reduction |
|-------|---------------|-----------------|
| **Vendor Samples** (Jan 11) | 757 files | 82% |
| **Rover2 Duplicates** (Jan 11) | 496 files | ~**95% total** |

## Recovery

If any archived code is needed:
1. Check this archive directory first
2. All vendor code is preserved with original structure  
3. Rover2 duplicate removal can be reversed by recreating from rover1 packages if needed
4. Can be moved back to active codebase if required

## Production Verification

- ✅ No production launch files reference rover2
- ✅ No rover1 packages import rover2 modules  
- ✅ All camera functionality preserved via consolidated drivers
- ✅ Full rover1 functionality maintained