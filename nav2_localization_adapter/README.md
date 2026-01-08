# ATOM01 Navigation Adapter

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Nav2](https://img.shields.io/badge/Nav2-Ready-green.svg)](https://navigation.ros.org/)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![License](https://img.shields.io/badge/license-GPL--3.0-yellow.svg)](https://opensource.org/licenses/GPL-3.0)

[English](README.md) | [中文](README_CN.md)

## Overview

This repository provides the navigation adapter and utility tools for the ATOM01 robot. It serves as a bridge between `robots_localization` (EKF) and the `Nav2` navigation stack, enabling seamless navigation without AMCL when using reliable odometry fusion. It handles the necessary TF transforms (`map` -> `odom` -> `base_link`) and provides robust tools for 2.5D map generation from point clouds.

**Maintainer**: Yongqi Zhang  
**Contact**: 1205041724@qq.com

**Key Features:**

- **Seamless Integration**: Automatically publishes `map` -> `odom` (static/identity) and `odom` -> `base_link` transforms based on fused odometry, satisfying Nav2 requirements.
- **2D Map Generation**: Includes `pcd2pgm.py`, a powerful tool that converts 3D PCD point clouds into an elevation map (intermediate format), calculates traversable areas, and finally generates Nav2-ready 2D occupancy grid maps (.pgm). It supports terrain analysis to correctly handle slopes and steps.
- **Waypoint Missions**: A flexible mission execution script (`waypoint_mission.py`) that supports looping, pass-through, and stop-at-waypoint modes using Nav2 actions.
- **Multi-Environment Support**: Pre-configured Nav2 parameters (MPPI, DWB) for different environments like gardens and indoor rooms.

## Installation

This package is a standard ROS 2 package.

- **Dependencies**:
  - ROS 2 (Humble/Jazzy)
  - Nav2 (`nav2_bringup`, `nav2_msgs`, etc.)
  - `robots_localization`
  - Python dependencies for scripts: `open3d`, `numpy`, `pyyaml`

- **Install Nav2**:

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-*
```

- **Install Python Dependencies** (for pcd2pgm tool):

```bash
pip install open3d numpy
```

- **Build**:

```bash
colcon build --packages-select nav2_localization_adapter --symlink-install
source install/setup.bash
```

## Usage

### 1. Generate Map from PCD (2.5D Support)
Convert 3D point cloud into an elevation map (intermediate format), calculate traversable areas, and finally generate Nav2-ready 2D occupancy grid maps (.pgm). Accounts for robot climbability (slopes/steps).

```bash
# Basic usage
python3 src/nav2_localization_adapter/scripts/pcd2pgm.py --input map.pcd --resolution 0.05

# Advanced usage (specify output directory, map name, and terrain parameters)
python3 src/nav2_localization_adapter/scripts/pcd2pgm.py \
    --input map.pcd \
    --output /path/to/output_dir \
    --map_name my_map \
    --max_slope_angle 30 \
    --max_step_height 0.15 \
    --z_min 0.2 \
    --z_max 2.0
```

### 2. Navigation

#### 2.1 Normal Navigation
Launch the complete navigation stack, including the map server, adapter node, and Nav2.
(Note: `map` and `params_file` arguments are optional if you modify the launch file defaults directly)

```bash
ros2 launch nav2_localization_adapter navigation.launch.py map:=/path/to/your/map.yaml params_file:=/path/to/nav2_params.yaml
```

#### 2.2 Automated Waypoint Mission Navigation
Execute a multi-point navigation mission defined in `config/waypoints.yaml`.

```bash
# 1. Launch navigation with waypoint support (using waypoint_navigation.launch.py)
ros2 launch nav2_localization_adapter waypoint_navigation.launch.py

# 2. Run mission script
ros2 run nav2_localization_adapter waypoint_mission.py
```

## Configuration

- **Adapter Params**: `config/adapter_params.yaml` - Configure TF frequencies and topic names.
- **Nav2 Params**: `config/nav2_params_*.yaml` - Tuned parameters for MPPI and DWB controllers.
- **Mission**: `config/waypoints.yaml` - Define waypoints and loop modes.

## References

* [Nav2](https://navigation.ros.org/) - The ROS 2 Navigation Stack.
* [ETH Elevation Mapping](https://github.com/ETHZ-RobotX/elevation_mapping) - Reference for elevation mapping.
