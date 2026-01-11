# Rover2 Project Plan: Personal Pack Robot

**Status**: Active Development  
**Goal**: Create a simplified autonomous cart that follows you using computer vision  
**Use Case**: Personal pack robot for carrying items outdoors  

---

## 🎯 Core Requirements

### Primary Functionality
- **Human Detection & Following**: Use computer vision to detect and follow human form
- **Single Person Outdoor Use**: Optimized for one person in outdoor environments  
- **Distance Control**: Maintain following distance by centering human bounding box in camera frame
- **Pack Robot Behavior**: Simple load/unload workflow - walk to destination, rover follows

### Hardware Stack
- **Camera**: Nuwa60c (existing working code available)
- **Sensors**: RTK GPS (for accuracy), BerryIMU v3 (orientation)
- **Mobility**: Mecanum holonomic drive (from rover1)
- **Compute**: Raspberry Pi 5

---

## 🎛 Interface Requirements

### Web Dashboard (Keep)
- Live camera feed display
- Teleop controls (WASD/virtual joystick)  
- Person follow mode controls (start/stop following)
- GPS location and map display
- Battery/system status

### Removed Features (Simplify)
- Patrol/waypoint following modes
- Teach & repeat autonomy  
- Trace recorder/follower
- Complex navigation planning
- Multi-mode autonomy switching

---

## 🏗 Implementation Plan

### Phase 1: Code Audit & Cleanup
- [ ] Identify components to keep vs remove from rover1 codebase
- [ ] Create simplified package structure for rover2
- [ ] Remove unnecessary patrol/autonomy packages

### Phase 2: Computer Vision Integration  
- [ ] Integrate nuwa60c camera drivers
- [ ] Implement human detection using OpenCV/MediaPipe
- [ ] Create person bounding box tracking system
- [ ] Test CV performance in outdoor conditions

### Phase 3: Following Control Logic
- [ ] Implement bounding box centering control algorithm
- [ ] Create motor control commands to follow detected person
- [ ] Add safety features (stop on person loss, obstacle detection)
- [ ] Tune following responsiveness and stability

### Phase 4: Dashboard Updates
- [ ] Add person-follow mode controls to web interface
- [ ] Integrate live camera feed with bounding box overlay
- [ ] Update UI for simplified rover2 feature set
- [ ] Test web interface functionality

### Phase 5: System Integration & Testing
- [ ] Create simplified launch files for rover2
- [ ] Test end-to-end person following functionality  
- [ ] Validate outdoor performance and reliability
- [ ] Document usage and troubleshooting

---

## 🎪 Success Criteria

1. **Person Detection**: Reliably detects and tracks human form in outdoor lighting
2. **Following Behavior**: Maintains following distance by centering person in camera frame
3. **Web Control**: Simple dashboard for starting/stopping follow mode and teleop
4. **Reliability**: Stable operation for typical pack robot use cases
5. **Simplicity**: Clean, focused codebase without rover1 complexity bloat

---

## 📁 Package Structure (Planned)

```
rover2/
├── rover2_bringup/          # Launch files (simplified)
├── rover2_hardware/         # Hardware drivers (GPS, IMU, motors, battery)  
├── rover2_vision/           # Person detection and following logic
├── rover2_dashboard/        # Web interface (simplified from gnss_web_dashboard)
└── rover2_description/      # Robot model (if needed)
```

---

## 🔄 Development Status

- [x] Requirements gathering complete
- [ ] Code audit in progress  
- [ ] CV integration pending
- [ ] Control logic pending
- [ ] Dashboard updates pending
- [ ] Testing pending