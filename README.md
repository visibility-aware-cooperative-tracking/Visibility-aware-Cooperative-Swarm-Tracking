# Visibility-aware Cooperative Swarm Tracking

Source code for the manuscript: *"Visibility-aware Cooperative Aerial Tracking with Decentralized LiDAR-based Swarm Robots".*

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Tips](#tips)
- [License](#license)



---

## Prerequisites

### System Requirements

- **OS**: Ubuntu 20.04
- **ROS**: [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (Desktop-Full recommended)

### Dependencies

Install the following system libraries:

```bash
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libarmadillo-dev \
    libglew-dev \
    libglfw3-dev \
    libboost-all-dev
```

> **Note**: OpenMP support is included with GCC by default on Ubuntu 20.04 and does not require a separate installation.

---

## Installation

**Clone the repository into your desired path and build**:

```bash
git clone https://github.com/visibility-aware-cooperative-tracking/Visibility-aware-Cooperative-Swarm-Tracking.git
cd Visibility-aware-Cooperative-Swarm-Tracking
catkin_make -j1
```

---

## Quick Start

### Example 1: 3-UAV Swarm Tracking (Mid360)

Simulates three trackers equipped with regular Mid360 LiDARs tracking a target in an environment with cylinder obstacles.

Open a new terminal and execute the following commands one by one:

```bash
source devel/setup.bash
roslaunch st_planner swarm_tracking_three_mid360.launch
```

Once launched, use the **"3D Nav Goal"** tool in RViz to set a navigation goal for the target. Then the drones start to track the target while maintaining visibility.

https://github.com/user-attachments/assets/d20704d7-680f-41b7-9c61-90931ddff244

---

### Example 2: 4-UAV Heterogeneous Swarm Tracking

Simulates four trackers with heterogeneous LiDAR configurations (2× Avia, 1× regular Mid360, 1× downward-facing Mid360) tracking a target in the cylinder environment.

Launch the simulation by:

```bash
source devel/setup.bash
roslaunch st_planner swarm_tracking_four_heter.launch
```

Use the **"3D Nav Goal"** tool in RViz to set the goal for the target. The drones then track the target and form a tetrahedral 
distribution while avoiding occlusion.

https://github.com/user-attachments/assets/db87252b-6752-4b0c-8f14-4ce31d3bb23c

---

### Example 3: SSDF Example

A standalone example for SSDF construction in a cluttered scene, providing the SSDF update and visualization interface usage example.

Launch the example by:

```bash
source devel/setup.bash
roslaunch st_planner ssdf_example.launch
```
A cross-section of the updated SSDF:

<img width="736" height="497" alt="Image" src="https://github.com/user-attachments/assets/82010f0a-952a-4b9b-a5ee-c7d6a5613ef8" />

---

## Tips
We recommend developers to use **[rosmon](http://wiki.ros.org/rosmon)** to replace the **roslaunch**
- It is developer-friendly, particularly for multi-robot projects, 
enabling convenient management and monitoring of multiple nodes.
- **To use rosmon** :
  [Install](http://wiki.ros.org/rosmon):
  ```
  sudo apt install ros-${ROS_DISTRO}-rosmon
  source /opt/ros/${ROS_DISTRO}/setup.bash # Needed to use the 'mon launch' shortcut
  ```
  To run the example of our project, open a new terminal window in the workspace and use **rosmon**:
  ```
  source devel/setup.bash
  mon launch st_planner swarm_tracking_three_mid360.launch
  ```

## License

The source code is released under [Apache 2.0](http://www.apache.org/licenses/) license.
