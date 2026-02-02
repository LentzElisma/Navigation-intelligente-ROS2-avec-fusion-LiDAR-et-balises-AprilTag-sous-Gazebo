# TurtleBot3 Autonomous Navigation with Visual Servoing & LiDAR Fusion

> **Advanced autonomous navigation system combining AprilTag visual servoing, progressive LiDAR-based obstacle avoidance, and mission-based trajectory control**


---

## Project Overview

This project implements an **intelligent navigation system** for the TurtleBot3 Waffle Pi robot, featuring:

-  **Visual Servoing**: Real-time AprilTag detection and centering for precise waypoint navigation
-  **LiDAR Fusion**: Progressive obstacle avoidance with multi-zone safety system
-  **Mission Control**: Tag-based commands (turn, accelerate, stop)
-  **Hybrid Control Architecture**: Vision-driven navigation + LiDAR safety constraints

---

## Tech Stack

| Technology | Purpose |
|------------|---------|
| **ROS2 Humble** | Robot middleware, pub/sub architecture |
| **Gazebo Classic** | 3D physics simulation environment |
| **OpenCV** | Image processing, visual servoing |
| **dt_apriltags** | Fiducial marker detection (tag36h11) |
| **NumPy** | Mathematical operations, sensor fusion |
| **LiDAR (360°)** | Real-time obstacle detection (0.12m - 3.5m range) |

---

## Key Features

###  **Visual Servoing System**
- Automatic AprilTag centering in camera frame
- Proportional angular correction based on pixel offset
- Distance-based approach control

###  **Progressive LiDAR Safety**
Multi-zone obstacle avoidance with anticipatory behavior:

```
Zone Libre (>80cm)      → Normal speed, no correction
Zone Vigilance (60-80cm) → Gentle correction (40% power)
Zone Danger (40-60cm)    → Strong correction + slowdown
Zone Critique (<35cm)    → Emergency stop
```

## Project Structure

```
voitureautonomeIA/
├── navigation.py          # Main navigation node (visual servoing + LiDAR)
├── main.py                # Entry point with startup banner
├── thescene/
│   ├── launch.py          # Gazebo + robot spawner
│   ├── myenv.world 
│    
└── README.md
```

---

## Installation

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic 11
- TurtleBot3 packages

### Setup
```bash
# Install dependencies
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs python3-opencv

# Install Python packages
pip3 install dt-apriltags opencv-python numpy


# Build workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Usage

### Launch Simulation
```bash
# Terminal 1: Start Gazebo environment
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch ~/ros2_ws/src/voitureautonomeIA/thescene/launch.py
```

### Run Navigation System
```bash
# Terminal 2: Start autonomous navigation
cd ~/ros2_ws/src/voitureautonomeIA/
python3 main.py
```


## Demo

### Visualization Features
-  Real-time AprilTag detection with green contours
-  Tag ID and distance display
-  LiDAR distance indicators (Left/Right/Front)
-  Current speed overlay

---

## Future Enhancements

-  Integration of YOLOv8 for traffic light detection
-  Dynamic obstacle avoidance (moving objects)
-  Path planning with Nav2 integration
-  Multi-robot coordination
-  ROS2 bag recording for replay analysis

---

## Author
**Lentz E.**
