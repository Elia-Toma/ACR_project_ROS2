# ROS2 Multi-Robot Warehouse Simulation

A ROS2-based multi-robot coordination system for warehouse package delivery simulation, designed to work with a Unity visualization counterpart.

## Overview

This project implements a decentralized multi-robot system where autonomous robots coordinate to pick up and deliver packages in a warehouse environment. Key features include:

- **Auction-based Task Allocation** - Robots bid on available tasks based on distance and battery level
- **A* Path Planning** - Optimal path computation using occupancy grid maps
- **Collision Avoidance** - Robots yield based on priority and adjust velocities to avoid collisions
- **Battery Management** - Robots monitor battery levels and autonomously go to charging stations
- **ROS-Unity Integration** - Seamless communication with Unity for visualization via `ros_tcp_endpoint`

---

## Requirements

- **ROS2 Humble** (or compatible distribution)
- **Python 3.10+**
- **ros-tcp-endpoint** package (for Unity communication)

---

## Project Structure

```
src/
├── monitor_node/          # Simulation coordinator
│   ├── launch/
│   │   └── simulation_launch.py
│   └── monitor_node/
│       └── monitor_node.py
├── path_planner/          # A* path planning service
│   └── path_planner/
│       └── path_planner_server.py
├── robot_nodes/           # Individual robot logic
│   └── robot_nodes/
│       └── robot_node.py
└── robot_interfaces/      # Custom messages and actions
    ├── msg/
    │   ├── Bid.msg
    │   ├── Task.msg
    │   └── SpawnedPackage.msg
    ├── action/
    │   └── ExecuteShift.action
    └── CMakeLists.txt
```

---

## Installation

### 1. Clone the repository

```bash
git clone <repository-url>
cd progettoRobotics
```

### 2. Install dependencies

```bash
# Install ros_tcp_endpoint (if not already installed)
sudo apt install ros-humble-ros-tcp-endpoint

# Or clone and build from source
cd src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b main-ros2
cd ..
```

### 3. Build the workspace

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Running the Simulation

> ⚠️ **IMPORTANT**: The ROS2 nodes must be launched **BEFORE** starting the Unity simulation.

### Quick Start

```bash
# Terminal 1: Source and launch the simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch monitor_node simulation_launch.py
```

### What the launch file starts:

| Node | Description |
|------|-------------|
| `ros_tcp_endpoint` | Unity communication bridge (port 10000) |
| `path_planner_server_robotX` | A* path planners (1 per robot) |
| `monitor_node` | Simulation coordinator |
| `robotX` | Robot nodes (4 robots by default) |

---

## Simulation Output

After all packages are delivered, a report is generated in the `output/` directory:

```
output/
└── run_YYMMDD_HHMMSS.txt
```

The report includes:
- Total simulation time
- Packages delivered per robot
- Average delivery time
- Complete event log

---

## Troubleshooting

### Unity connection issues
- Ensure ROS2 is running **before** starting Unity
- Verify port `10000` is not blocked

### Path planning failures
- Ensure `/map` topic is being published from Unity
- Check that start/goal positions are not inside obstacles

---

## License

This project is part of an academic robotics course project.
