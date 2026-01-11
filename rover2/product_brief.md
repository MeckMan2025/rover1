# Rover1: The DIY Artificial Intelligence & Autonomy Kit

**Status**: Active Development  
**Target Audience**: Students, Hobbyists, Future Engineers  
**Mission**: Go from zero experience to a fully autonomous rover in just one week.

---

## 🚀 Overview
Rover1 is a democratized autonomous waypoint-navigating rover designed to showcase the power of AI and robotics. It strips away the complexity of industrial robotics, providing a streamlined, "batteries-included" experience that allows anyone to build, deploy, and watch their code drive a physical machine with centimeter-level precision.

## ✨ Key Experience
Impress your friends and family with a professional-grade autonomy demo that fits in a backpack.

### 1. **Zero-Install User Interface**
No app store required. Rover1 hosts its own high-speed Web UI accessible from any:
- Laptop
- Tablet
- Smartphone
(Simply connect to the rover's Wi-Fi/Hotspot and start driving).

### 2. **Professional Controls**
- **Teleoperation**: Drive manaully using WASD keys or touch-screen virtual joysticks.
- **RTK GPS Integration**: Visualize real-time satellite lock and confirm <2cm accuracy before launching missions.

### 3. **Teach & Repeat Autonomy**
Forget complex coding for every path.
1.  **Record**: Press "Start Recording" and drive your desired path manually.
2.  **Process**: Rover1 learns the line geometry between your waypoints.
3.  **Execute**: Press "Start Auto". The rover follows your exact path.
4.  **Endless Patrol**: Upon reaching the end, Rover1 performs a **180° tactical turn** and navigates the path in reverse, repeating the loop indefinitely until you say stop.

## 🛠 Tech Stack
- **Brain**: Raspberry Pi 5
- **OS**: Ubuntu Server 24.04 (ROS 2 Jazzy)
- **Sensors**: 
  - U-Blox ZED-F9R (RTK GPS)
  - BerryIMU v3 (Orientation)
- **Mobility**: Mecanum Holonomic Drive

## 🎓 Learning Outcomes
Building Rover1 teaches:
- **Linux & Networking**: SSH, service management, and Wi-Fi bridging.
- **Robotics (ROS 2)**: Nodes, topics, TF2 transforms, and launch files.
- **Control Theory**: PID loops, Ekf sensor fusion, and inverse kinematics.
- **Navigation**: GPS coordinate math and autonomous path planning.
