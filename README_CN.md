# atom01_navigation

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS 2](https://img.shields.io/badge/ROS-2-blue.svg)](https://docs.ros.org/en/humble/index.html)

一个先进的移动机器人自主导航系统，集成了高精度定位与 ROS 2 导航堆栈（Nav2）。

[English](README.md) | [中文](README_CN.md)

## 目录

- [概述](#概述)
- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [功能包](#功能包)
- [前置要求](#前置要求)
- [安装](#安装)
- [使用方法](#使用方法)
- [配置说明](#配置说明)
- [贡献指南](#贡献指南)
- [致谢](#致谢)

## 概述

ATOM01 导航系统是一个基于 ROS 2 构建的综合性机器人导航解决方案。它结合了基于 3D LiDAR 的 SLAM、UWB 定位和惯性测量，为复杂环境中的机器人提供鲁棒的 3D 定位能力。同时，系统通过适配器将 3D 定位信息转换为 2D 导航所需的格式，与 Nav2 导航堆栈无缝集成，实现了基于 3D 定位和 2D 地图的自主导航功能。

**创建者**: Yongqi Zhang
**联系方式**: 1205041724@qq.com

## 功能特性

- **高精度定位**：基于 FAST-LIO 2 算法的实时激光雷达-惯性里程计
- **多传感器融合**：集成 Livox 激光雷达、IMU 和 UWB 定位系统
- **Nav2 集成**：与 ROS 2 导航堆栈无缝兼容的适配器
- **实时性能**：针对嵌入式系统和资源受限平台优化
- **3D 点云处理**：先进的点云配准和建图能力
- **高程地图**：用于地形分析的全局高程地图生成

## 系统架构

系统采用分层架构，底层利用 **3D 雷达定位** (robots_localization_ros2) 进行高频率、高精度的 6DOF 状态估计；中间层通过适配器模块将 3D 位姿投影并发布 TF 变换；上层对接 **2D 地图导航** (Nav2)，实现路径规划与避障。

```text
[ 传感器输入 ]
      |
      v
[ robots_localization_ros2 ] (3D 定位)
  - 输入: Livox LiDAR, IMU, UWB
  - 核心: FAST-LIO 2 (LIO)
  - 输出: 3D 里程计, 点云地图
      |
      v
[ nav2_localization_adapter ] (适配层)
  - 功能: 坐标系变换 (TF), 格式转换
  - 输出: map->odom TF, 2D Pose
      |
      v
[ ROS 2 Nav2 Stack ] (2D 导航)
  - 功能: 全局/局部规划, 避障
  - 输出: 速度控制 (cmd_vel)
```

## 功能包

### 1. robots_localization_ros2

基于 FAST-LIO 2 的核心定位功能包，提供高精度状态估计。

**主要功能：**
- 实时激光雷达-惯性里程计（LIO）
- 基于 ikd-Tree 的点云管理
- 全局高程地图生成
- PCD 地图加载与定位

**主要节点：**
- `robots_localization_node`: 核心定位节点

**话题 (Topics)：**
- 订阅: `/livox/lidar`, `/livox/imu`, `/nlink_linktrack_nodeframe5` (UWB)
- 发布: `/odometry`, `/current_pose`, `/map`, `/path`

**更多详细信息请参考：** [robots_localization_ros2/README.md](robots_localization_ros2/README.md)

### 2. nav2_localization_adapter

作为 `robots_localization` (EKF) 与 `Nav2` 导航协议栈之间的桥梁，在使用可靠里程计融合时无需 AMCL 即可实现无缝导航。

**主要功能：**
- **无缝集成**: 自动发布 `map` -> `odom` 和 `odom` -> `base_link` 变换。
- **2D 地图生成**: 包含 `pcd2pgm.py` 工具，将 3D PCD 点云转化成 Nav2 支持的 2D 占用栅格地图。
- **航点任务**: 灵活的任务执行脚本，支持循环巡逻等模式。
- **多环境支持**: 预配置了针对不同环境的 Nav2 参数文件。

**主要节点：**
- `nav2_localization_adapter_node`: 适配器节点

**话题 (Topics)：**
- 订阅: `/odometry` (来自 robots_localization)
- 发布: `/amcl_pose`, TF 变换 (map→odom)

**更多详细信息请参考：** [nav2_localization_adapter/README_CN.md](nav2_localization_adapter/README_CN.md)

### 3. nlink_parser_ros2

NLink UWB 定位系统接口和消息定义。包含协议解析与串口通信功能。

**子功能包：**
- **nlink_parser_ros2**: NLink 协议的主解析器
- **nlink_message**: NLink 设备的自定义消息定义
- **serial**: 跨平台串口通信库

**主要节点：**
- `linktrack`: LinkTrack UWB 传感器驱动节点
- `tofsense`: TOFSense 激光测距传感器驱动节点

**话题 (Topics)：**
- 发布: `/nlink_linktrack_nodeframe0`, `/nlink_linktrack_nodeframe5`, `/nlink_tofsense_frame0`

**更多详细信息请参考：** [nlink_parser_ros2/README.md](nlink_parser_ros2/README.md)

## 前置要求

### 系统要求

- **操作系统**：Ubuntu 22.04
- **ROS 版本**：ROS 2 Humble

### 硬件要求

- **激光雷达**：Livox MID-360 或兼容型号
- **IMU**：集成式或外置 IMU（重力单位应为 'g'）
- **UWB 模块**：NLink LinkTrack（可选）

### 依赖项

请参考各个功能包的 README 安装详细依赖。

- [robots_localization_ros2/README.md](robots_localization_ros2/README.md)
- [nav2_localization_adapter/README_CN.md](nav2_localization_adapter/README_CN.md)
- [nlink_parser_ros2/README.md](nlink_parser_ros2/README.md)

#### 通用依赖

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

#### Python 依赖 (参考 [nav2_localization_adapter/README_CN.md](nav2_localization_adapter/README_CN.md))

```bash
pip install open3d numpy pyyaml
```

#### 第三方库编译 (参考 [robots_localization_ros2/README.md](robots_localization_ros2/README.md))

**Sophus** (推荐 1.22.10)
```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus/
git checkout 1.22.10
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

**fmt** (需开启 -fPIC)
```bash
git clone https://github.com/fmtlib/fmt
cd fmt
mkdir build && cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make -j$(nproc)
sudo make install
```
**注意**：必须在 `CMakeLists.txt` 中添加 `add_compile_options(-fPIC)` ！！

**Livox SDK2 & Driver**
Livox 驱动推荐安装在工作空间之外，参考链接：[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) 和 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

```bash
# Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Livox ROS Driver2 (建议放在 src 目录外)
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble
```

## 安装

### 1. 克隆仓库

使用 `--recursive` 参数自动克隆子模块：

```bash
cd ~/
mkdir -p atom01_ws/src
cd atom01_ws/src
git clone --recursive <仓库地址> atom01_navigation
```

### 2. 安装依赖

```bash
cd ~/atom01_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译工作空间

推荐使用 Ninja 编译：

```bash
cd ~/atom01_ws
colcon build --symlink-install --cmake-args -G Ninja
```

## 使用方法

在使用任何命令前，请确保已加载工作空间环境变量：

```bash
cd ~/atom01_ws
source install/setup.bash
```

### 1. 建图与定位

详细操作请参考 [robots_localization_ros2/README.md](robots_localization_ros2/README.md)。

**简要步骤：**
1. 启动雷达驱动。
2. 修改 `config/robots_localization.yaml` 中的 `mapping_en` 参数：
   - `true`: 建图模式
   - `false`: 定位模式
3. 启动节点：
   ```bash
   # 建图模式
   ros2 launch robots_localization_ros2 mapping.launch.py
   
   # 或 定位模式
   ros2 launch robots_localization_ros2 localization.launch.py
   ```

### 2. 导航与地图处理

详细操作请参考 [nav2_localization_adapter/README_CN.md](nav2_localization_adapter/README_CN.md)。

**简要步骤：**
1. **PCD 转 2D 地图**：使用 `pcd2pgm.py` 工具转换地图。
2. **启动导航**：
   ```bash
   ros2 launch nav2_localization_adapter navigation.launch.py map:=<地图路径>
   ```

## 配置说明

### robots_localization_ros2

主配置文件：`config/robots_localization.yaml`

关键参数：
- `common.lid_topic`：激光雷达点云话题
- `common.imu_topic`：IMU 数据话题
- `preprocess.lidar_type`：激光雷达类型（Livox 为 1）
- `mapping_en`：建图使能 (true: 建图, false: 纯定位)
- `mapping.extrinsic_T`：LiDAR 到 IMU 的平移
- `mapping.extrinsic_R`：LiDAR 到 IMU 的旋转

### nav2_localization_adapter

详细配置请参考包内文档：[nav2_localization_adapter/README_CN.md](nav2_localization_adapter/README_CN.md)

## 已知问题与解决方案

### 1. 雷达连接问题
- 检查 IP 配置（机器人不应使用代理）
- 验证雷达 IP 地址与配置匹配
- 确保网络接口设置正确

## 重要说明

- **Livox IMU**：重力单位为 'g'，确保代码中的归一化与此匹配
- **MID-360 IMU 外参**：在 LiDAR 坐标系中的位置为 [0.011, 0.0234, -0.044]
- **点云时间戳**：Header 时间是最早点的时间；单个点的时间是相对于 header 时间的偏移量（单位：毫秒）
- **PCD 地图**：在启动定位模式前，将 PCD 文件放置在指定的地图目录中

## 贡献指南

欢迎贡献！请随时提交 Pull Request。对于重大更改，请先开启 issue 讨论您想要更改的内容。

### 开发指南

1. 遵循 ROS 2 编码标准
2. 编写清晰的提交信息
3. 为新功能添加测试
4. 相应地更新文档

## 致谢

- **FAST-LIO 2**：本项目基于 FAST-LIO 2 算法开发
- **ikd-Tree**：高效的点云管理结构
- **Livox Technology**：提供激光雷达硬件和驱动支持
- **Nav2**：ROS 2 导航堆栈
- **ROS 2 社区**：提供优秀的机器人中间件

---

**注意**：这是一个活跃开发的项目。功能和 API 可能会发生变化。请查看仓库获取最新更新。
