# ATOM01 Navigation Adapter

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Nav2](https://img.shields.io/badge/Nav2-Ready-green.svg)](https://navigation.ros.org/)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![License](https://img.shields.io/badge/license-GPL--3.0-yellow.svg)](https://opensource.org/licenses/GPL-3.0)

[English](README.md) | [中文](README_CN.md)

## 概述 (Overview)

本仓库提供了 ATOM01 机器人的导航适配器及实用工具。它作为 `robots_localization` (EKF) 与 `Nav2` 导航协议栈之间的桥梁，在使用可靠里程计融合时无需 AMCL 即可实现无缝导航。该功能包处理了必要的 TF 变换 (`map` -> `odom` -> `base_link`)，并提供了从点云生成 2.5D 地图的强大工具。

**维护者**: Yongqi Zhang  
**联系方式**: 1205041724@qq.com

**核心功能:**

- **无缝集成**: 自动发布 `map` -> `odom` (静态/单位矩阵) 和 `odom` -> `base_link` 变换，满足 Nav2 的 TF 树要求，无需运行 AMCL。
- **2D 地图生成**: 包含 `pcd2pgm.py` 工具，将 3D PCD 点云转化成高程图（中间格式），计算可通行区域最后转化成 Nav2 导航支持的 2D 占用栅格地图 (.pgm)。该工具支持地形分析，能够正确处理斜坡和台阶等地形的可通行性。
- **航点任务**: 灵活的任务执行脚本 (`waypoint_mission.py`)，支持通过 Nav2 Action 执行循环巡逻、途经点导航和站点停靠等模式。
- **多环境支持**: 针对花园、室内等不同环境，预配置了 MPPI 和 DWB 控制器的 Nav2 参数文件。

## 安装指南 (Installation)

这是一个标准的 ROS 2 功能包。

- **依赖项**:
  - ROS 2 (Humble)
  - Nav2 (`nav2_bringup`, `nav2_msgs`, etc.)
  - `robots_localization`
  - Python 依赖 (用于脚本): `open3d`, `numpy`, `pyyaml`

- **安装 Nav2**:

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-*
```

- **安装 Python 依赖** (用于 pcd2pgm 工具):

```bash
pip install open3d numpy
```

- **编译**:

```bash
colcon build --packages-select nav2_localization_adapter --symlink-install
source install/setup.bash
```

## 使用方法 (Usage)

### 1. 从 PCD 生成地图 (支持 2.5D)
将 3D 点云转化成高程图（中间格式），计算可通行区域最后转化成 Nav2 导航支持的 2D 占用栅格地图 (.pgm)。支持根据机器人的爬坡能力（斜坡/台阶）判断可通行区域。

```bash
# 基础用法
python3 src/nav2_localization_adapter/scripts/pcd2pgm.py --input map.pcd --resolution 0.05

# 进阶用法 (指定输出目录、地图名称及地形参数)
python3 src/nav2_localization_adapter/scripts/pcd2pgm.py \
    --input map.pcd \
    --output /path/to/output_dir \
    --map_name my_map \
    --max_slope_angle 30 \
    --max_step_height 0.15 \
    --z_min 0.2 \
    --z_max 2.0
```

### 2. 导航 (Navigation)

#### 2.1 普通导航
启动完整的导航栈，包括地图服务器、适配器节点和 Nav2。
(注: 如果已在 launch 文件中修改默认参数，则无需指定 `map` 和 `params_file`)

```bash
ros2 launch nav2_localization_adapter navigation.launch.py map:=/path/to/your/map.yaml params_file:=/path/to/nav2_params.yaml
```

#### 2.2 自动化航点任务导航
执行定义在 `config/waypoints.yaml` 中的多点导航任务。

```bash
# 1. 启动支持航点的导航 (使用 waypoint_navigation.launch.py)
ros2 launch nav2_localization_adapter waypoint_navigation.launch.py

# 2. 运行任务脚本
ros2 run nav2_localization_adapter waypoint_mission.py
```

## 配置说明 (Configuration)

- **适配器参数**: `config/adapter_params.yaml` - 配置 TF 发布频率和话题名称。
- **Nav2 参数**: `config/nav2_params_*.yaml` - 针对 MPPI 和 DWB 控制器调整的参数。
- **任务配置**: `config/waypoints.yaml` - 定义航点列表和循环模式。

## 参考资料 (References)

* [Nav2](https://navigation.ros.org/) - ROS 2 导航协议栈。
* [ETH Elevation Mapping](https://github.com/ETHZ-RobotX/elevation_mapping) - 高程图相关参考。
