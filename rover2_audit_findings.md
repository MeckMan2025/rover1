# Rover2 Codebase Audit Findings

## Components to Keep (Core Functionality)

### 1. Hardware Drivers
- **rover1_hardware/**: Keep all hardware drivers
  - `battery_monitor.py` - Battery voltage monitoring
  - `berry_imu_driver.py` - IMU/orientation data
  - `hiwonder_driver.py` - Motor control
  - `mecanum_kinematics.py` - Wheel kinematics 
  - `stadia_teleop.py` - Controller input
  - `fix_to_nmea.py` - GPS message conversion

### 2. Vision System (Perfect for Rover2!)
- **rover1_vision/shoe_follower.py**: EXACTLY what we need!
  - Uses Hailo-8L AI accelerator for YOLOv8 person detection
  - Implements human following using bounding box centering
  - Has teleop override safety features
  - Already uses HP60C camera (compatible with nuwa60c)
  - Service-based enable/disable control
  - Status reporting for dashboard integration

### 3. Web Dashboard
- **gnss_web_dashboard/**: Keep with modifications
  - `web_dashboard.py` - Flask web server
  - `static/index.html` - Web interface (needs person-follow controls)
  - `static/js/` - JavaScript modules (remove patrol.js, keep others)
  - `static/css/` - Styling (remove patrol.css, keep others)

### 4. Camera Integration
- **Camera_Specs/ascam_ros2_ws/**: Camera drivers available
  - `src/ascamera/launch/nuwa.launch.py` - Launch file for nuwa camera
  - `src/ascamera/src/` - Camera publisher nodes

### 5. Core Configuration
- **rover1_bringup/config/**: Keep essential configs
  - `ekf.yaml` - Sensor fusion configuration
  - `navsat.yaml` - GPS configuration
  - Navigation configs (for basic positioning)

## Components to Remove (Bloat for Rover2)

### 1. Patrol/Autonomy System
- **rover1_patrol/**: Remove entirely
  - `waypoint_recorder.py`
  - `patrol_manager.py` 
  - `gps_waypoint_follower.py`
  - `trace_recorder.py`
  - `trace_follower.py`
  - All patrol-related launch files

### 2. Navigation Stack
- **rover1_nav/**: Remove (no complex navigation needed)
- Nav2 related configs in bringup

### 3. Dashboard Components
- Remove patrol-related UI components
- `static/js/patrol.js`
- `static/css/patrol.css`

## Key Finding: shoe_follower.py is Perfect!

The existing `rover1_vision/rover1_vision/shoe_follower.py` file is **exactly** what rover2 needs:

1. **Human Detection**: Uses COCO class 0 (person) detection with YOLOv8
2. **Following Logic**: Centers human bounding box in camera frame
3. **Camera Compatibility**: Already uses `/ascamera_hp60c/camera_publisher/rgb0/image` 
4. **Safety Features**: Teleop override, timeout detection, recovery scanning
5. **Service Control**: Enable/disable detection and following via ROS services
6. **Status Reporting**: Publishes status for dashboard integration

## Rover2 Package Structure Plan

```
rover2/
├── rover2_bringup/          # Simplified launch files
├── rover2_hardware/         # Copy of rover1_hardware (unchanged)
├── rover2_vision/           # Adapted from rover1_vision (person_follower.py)
└── rover2_dashboard/        # Simplified web dashboard
```

## Implementation Strategy

1. **Start with shoe_follower.py**: Rename to `person_follower.py` and adapt for nuwa60c camera
2. **Simplify Dashboard**: Remove patrol UI, add person-follow controls
3. **Streamline Launch**: Create minimal launch file for person-following mode
4. **Camera Integration**: Update camera topic names for nuwa60c if needed

This audit shows rover2 will be much simpler than expected - we already have working person detection and following code!