# atom01_navigation

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS 2](https://img.shields.io/badge/ROS-2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Nav2](https://img.shields.io/badge/Nav2-Enabled-success.svg)](https://navigation.ros.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://releases.ubuntu.com/22.04/)
[![C++](https://img.shields.io/badge/C%2B%2B-14%2F17-blue.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.10-blue.svg)](https://www.python.org/)
[![Build](https://img.shields.io/badge/Build-Colcon-orange.svg)](https://colcon.readthedocs.io/en/released/)
[![PCL](https://img.shields.io/badge/PCL-1.12-green.svg)](https://pointclouds.org/)
[![Open3D](https://img.shields.io/badge/Open3D-0.17.0-blue.svg)](http://www.open3d.org/)
[![NumPy](https://img.shields.io/badge/NumPy-1.21-blue.svg)](https://numpy.org/)

An autonomous navigation system for mobile robots, integrating high-precision localization with ROS 2 Navigation Stack (Nav2).

[English](README.md) | [中文](README_CN.md)

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Packages](#packages)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [Acknowledgments](#acknowledgments)

## Overview

The ATOM01 Navigation System is a comprehensive robotics navigation solution built on ROS 2. It combines 3D LiDAR-based SLAM, UWB positioning, and inertial measurement for robust **3D localization and mapping** in complex environments. The system seamlessly integrates with the Nav2 navigation stack by bridging 3D poses to 2D navigation requirements, enabling autonomous **2D map navigation** for mobile robots.

**Creator**: Yongqi Zhang
**Contact**: 1205041724@qq.com

## Features

- **High-Precision Localization and Mapping**: Based on FAST-LIO 2 algorithm for real-time 3D mapping and LiDAR-Inertial odometry
- **Multi-Sensor Fusion**: Integrates Livox LiDAR, IMU, and UWB positioning systems
- **Nav2 Integration**: Seamless adapter for ROS 2 Navigation Stack compatibility
- **Real-time Performance**: Optimized for embedded systems and resource-constrained platforms
- **3D Point Cloud Processing**: Advanced point cloud registration and mapping capabilities
- **Elevation Mapping**: Global elevation map generation for terrain analysis

## System Architecture

The system utilizes a layered architecture where **3D LiDAR Localization** (robots_localization_ros2) provides high-frequency, high-precision 6DOF state estimation. An intermediate adapter layer projects these 3D poses and publishes TF transforms to support **2D Map Navigation** (Nav2) for path planning and obstacle avoidance.

```text
[ Sensor Inputs ]
      |
      v
[ robots_localization_ros2 ] (3D Localization)
  - Inputs: Livox LiDAR, IMU, UWB
  - Core: FAST-LIO 2 (LIO)
  - Outputs: 3D Odometry, Point Cloud Map
      |
      v
[ nav2_localization_adapter ] (Adapter Layer)
  - Function: Frame Transform (TF), Format Conversion
  - Outputs: map->odom TF, 2D Pose
      |
      v
[ ROS 2 Nav2 Stack ] (2D Navigation)
  - Function: Global/Local Planning, Obstacle Avoidance
  - Outputs: Velocity Control (cmd_vel)
```

## Packages

### 1. robots_localization_ros2

The core localization package based on FAST-LIO 2, providing high-precision state estimation.

**Key Features:**
- Real-time LiDAR-Inertial Odometry (LIO)
- 3D Point Cloud Mapping
- ikd-Tree based point cloud management
- Global elevation map generation
- PCD map loading and localization

**Main Nodes:**
- `robots_localization_node`: Main localization node

**Topics:**
- Subscribes: `/livox/lidar`, `/livox/imu`
- Publishes: `/odometry`, `/current_pose`, `/map`, `/path`

**For more details, please refer to:** [robots_localization_ros2/README.md](robots_localization_ros2/README.md)

### 2. nav2_localization_adapter

Bridge adapter connecting `robots_localization` (EKF) output to `Nav2` navigation stack requirements, enabling seamless navigation without AMCL when using reliable odometry fusion.

**Key Features:**
- **Seamless Integration**: Automatically publishes `map` -> `odom` and `odom` -> `base_link` transforms.
- **2D Map Generation**: Includes `pcd2pgm.py` tool to convert 3D PCD point clouds into Nav2-ready 2D occupancy grid maps.
- **Waypoint Missions**: Flexible mission execution script for loop patrols and more.
- **Multi-Environment Support**: Pre-configured Nav2 parameters for different environments.

**Main Nodes:**
- `nav2_localization_adapter_node`: Adapter node

**Topics:**
- Subscribes: `/odometry` (from robots_localization)
- Publishes: `/pose`, TF transforms (map→odom)

**For more details, please refer to:** [nav2_localization_adapter/README.md](nav2_localization_adapter/README.md)

### 3. nlink_parser_ros2

NLink UWB positioning system interface and message definitions. Includes protocol parsing and serial communication.

**Sub-packages:**
- **nlink_parser_ros2**: Main parser for NLink protocol
- **nlink_message**: Custom message definitions for NLink devices
- **serial**: Cross-platform serial port communication library

**Main Nodes:**
- `linktrack`: LinkTrack UWB sensor driver node
- `tofsense`: TOFSense laser ranging sensor driver node

**For more details, please refer to:** [nlink_parser_ros2/README.md](nlink_parser_ros2/README.md)

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 22.04
- **ROS Version**: ROS 2 Humble

### Hardware Requirements

- **LiDAR**: Livox MID-360 or compatible
- **IMU**: Integrated or external IMU (gravity unit should be 'g')
- **UWB Module**: NLink LinkTrack (optional)

### Dependencies

Please refer to the README of each package for detailed dependencies:
- [robots_localization_ros2/README.md](robots_localization_ros2/README.md)
- [nav2_localization_adapter/README.md](nav2_localization_adapter/README.md)
- [nlink_parser_ros2/README.md](nlink_parser_ros2/README.md)

#### General Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-* \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libceres-dev \
    ccache
```

#### Python Dependencies

```bash
pip install open3d numpy pyyaml Pillow scipy
```

#### Third-Party Libraries

**Sophus** (Recommended 1.22.10)
```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus/
git checkout 1.22.10
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

**fmt** (With -fPIC enabled)
```bash
git clone https://github.com/fmtlib/fmt
cd fmt
mkdir build && cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make -j$(nproc)
sudo make install
```
**Note**: You must add `add_compile_options(-fPIC)` in `CMakeLists.txt`!!

**Livox SDK2 & Driver**
Livox drivers are recommended to be installed outside the workspace. References: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) and [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

```bash
# Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Livox ROS Driver2 (recommended outside src directory)
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble
```

## Installation

### 1. Clone the Repository

Use the `--recursive` parameter to automatically clone submodules:

```bash
cd ~/
mkdir -p atom01_ws/src
cd atom01_ws/src
git clone --recursive https://github.com/Roboparty/atom01_navigation.git
```

### 2. Build the Workspace

```bash
cd ~/atom01_ws
colcon build --symlink-install
```

## Usage

Before using any commands, ensure the workspace environment variables are loaded:

```bash
cd ~/atom01_ws
source install/setup.bash
```

### 1. Mapping and Localization

For detailed instructions, please refer to [robots_localization_ros2/README.md](robots_localization_ros2/README.md).

**Brief Steps:**
1. Start LiDAR driver.
2. Modify `mapping_en` parameter in `config/robots_localization.yaml`:
   - `true`: Mapping mode
   - `false`: Localization mode
3. Launch nodes:
   ```bash
   # Mapping Mode
   # Note: Mapping and localization use the same launch file, distinguished by the mapping_en parameter in config
   ros2 launch robots_localization_ros2 localization.launch.py
   
   # Or Localization Mode
   ros2 launch robots_localization_ros2 localization.launch.py
   ```

### 2. Navigation and Map Processing

For detailed instructions, please refer to [nav2_localization_adapter/README.md](nav2_localization_adapter/README.md).

**Brief Steps:**
1. **Convert PCD to 2D Map**: Use `pcd2pgm.py` tool.
2. **Start Navigation**:
   ```bash
   ros2 launch nav2_localization_adapter navigation.launch.py map:=<path_to_map>
   ```

## Configuration

### robots_localization_ros2

Main configuration file: `config/robots_localization.yaml`

Key parameters:
- `common.lid_topic`: LiDAR point cloud topic
- `common.imu_topic`: IMU data topic
- `preprocess.lidar_type`: LiDAR type (1 for Livox)
- `mapping_en`: Enable mapping (true: Mapping, false: Localization only)
- `mapping.extrinsic_T`: LiDAR to IMU translation
- `mapping.extrinsic_R`: LiDAR to IMU rotation

### nav2_localization_adapter

For detailed configuration, please refer to: [nav2_localization_adapter/README.md](nav2_localization_adapter/README.md)

## Known Issues and Solutions

### 1. LiDAR Connection Issues
- Check IP configuration (robot should not use proxy)
- Verify LiDAR IP address matches configuration
- Ensure proper network interface settings

## Important Notes

- **Livox IMU**: Gravity unit is 'g', ensure normalization in code matches this
- **MID-360 IMU Extrinsics**: Position in LiDAR frame is [0.011, 0.0234, -0.044]
- **Point Cloud Timing**: Header time is the earliest point time; individual point time is relative offset in milliseconds
- **PCD Map**: Place PCD files in the designated map directory before launching localization mode

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

### Development Guidelines

1. Follow ROS 2 coding standards
2. Write clear commit messages
3. Add tests for new features
4. Update documentation accordingly

## Acknowledgments

- **FAST-LIO 2**: This project builds upon the FAST-LIO 2 algorithm
- **ikd-Tree**: Efficient point cloud management structure
- **Livox Technology**: For LiDAR hardware and driver support
- **Nav2**: ROS 2 Navigation Stack
- **ROS 2 Community**: For the excellent robotics middleware

---

**Note**: This is an active development project. Features and APIs may change. Please check the repository for the latest updates.
