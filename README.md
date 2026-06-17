# Radar SLAM Docker

> 基于车载毫米波雷达的实时 SLAM（同时定位与建图）系统，支持 Continental ARS620 和 SR75 雷达，提供从 CAN FD 硬件驱动、点云预处理、前后端 SLAM 优化、回环检测到雷达-激光雷达外参标定的完整流水线。

[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Noetic-green.svg)](https://wiki.ros.org/noetic)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED.svg)](https://www.docker.com/)

**作者**: Glory Huang ([@gloryhry](https://github.com/gloryhry))

---

## 目录

- [系统架构](#系统架构)
- [硬件要求](#硬件要求)
- [软件依赖](#软件依赖)
- [快速开始（Docker）](#快速开始docker)
- [手动构建](#手动构建)
- [ROS 包说明](#ros-包说明)
  - [radar_canfd_driver](#radar_canfd_driver)
  - [radar_slam](#radar_slam)
  - [radar_lidar_static_calibration](#radar_lidar_static_calibration)
- [启动方式](#启动方式)
- [配置说明](#配置说明)
- [标定流程](#标定流程)
- [测试](#测试)
- [项目结构](#项目结构)
- [CI/CD](#cicd)
- [常见问题](#常见问题)
- [许可证](#许可证)
- [引用](#引用)

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────────────┐
│                          硬件层 (Hardware)                            │
│                                                                      │
│  Continental ARS620 (CAN FD)          SR75 毫米波雷达 (以太网)         │
│         │                                      │                      │
│         ▼                                      ▼                      │
│  ZLG USBCANFD-200U 适配器               以太网数据流                    │
└──────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────────────────┐
│                     驱动层 (radar_canfd_driver)                       │
│                                                                      │
│  radar_canfd_node:  CAN FD 帧 → ARS620 协议解码 → RDI/OD 点云        │
│  radar_merger_node: 前雷达 + 后雷达点云融合（可配置外参变换）           │
│                                                                      │
│  发布话题:                                                            │
│  /ars620_front/rdi_points    — 前雷达原始检测点云                      │
│  /ars620_rear/rdi_points     — 后雷达原始检测点云                      │
│  /ars620_front/od_points     — 前雷达目标跟踪点云                      │
│  /ars620_rear/od_points      — 后雷达目标跟踪点云                      │
│  /merged_cloud               — 前后融合点云                            │
└──────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    过滤层 (radar_slam 可执行文件)                      │
│                                                                      │
│  sr75 节点:                                                           │
│    极坐标网格滤波 → MAD 速度异常滤除 → DBSCAN 聚类滤波 → pointXYZVvar  │
│                                                                      │
│  ars620 节点:                                                         │
│    质量阈值 → MAD 速度异常滤除 → SNR/RCS 阈值过滤                      │
│                                                                      │
│  dual_ars620 节点:                                                    │
│    前后话题时间同步 → 坐标变换 → 合并 → 多级滤波                       │
│                                                                      │
│  发布话题: /RadarCombined (自定义 pointXYZVvar 格式)                   │
└──────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    SLAM 核心 (radar_slam 库)                           │
│                                                                      │
│  ┌──────────┐    ┌───────────┐    ┌──────────────┐                  │
│  │  Frame   │───▶│ FrontEnd  │───▶│   BackEnd    │                  │
│  │ (单帧点云)│    │ (ICP/NDT  │    │ (g2o 位姿图  │                  │
│  │          │    │  扫描匹配) │    │  优化)       │                  │
│  └──────────┘    └─────┬─────┘    └──────┬───────┘                  │
│                        │                 │                           │
│                        ▼                 ▼                           │
│                   ┌─────────┐     ┌────────────┐                    │
│                   │ Stream  │     │ LoopClosing│                    │
│                   │(帧序列) │◀───▶│(回环检测)  │                    │
│                   └─────────┘     └────────────┘                    │
│                        │                 │                           │
│                        ▼                 ▼                           │
│                   ┌──────────────────────────┐                      │
│                   │       Map (地图)          │                      │
│                   │   关键帧存储 & 地图发布    │                      │
│                   └──────────────────────────┘                      │
│                                                                      │
│  IMU 集成（可选）:                                                    │
│  ImuBuffer → ImuStaticInitializer → ImuPreintegrator                 │
│       → 位姿预测 & 预积分约束边 (g2o EdgeImuPreintegration)           │
└──────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────────────────┐
│                  外参标定 (radar_lidar_static_calibration)            │
│                                                                      │
│  • 方法一：手动选点 → 伪逆最小二乘（静态标定）                         │
│  • 方法二：ICP 帧间匹配 → 里程计对比（动态标定）                       │
│                                                                      │
│  输出: 雷达-激光雷达外参变换矩阵 (6-DOF)                              │
└──────────────────────────────────────────────────────────────────────┘
```

### 核心设计

- **基于流的增量式架构 (Stream-based)**：连续的雷达帧被组织为"流"（Stream），每个流内部通过 ICP/NDT 估计帧间运动，流作为位姿图顶点。
- **g2o 后端优化**：使用通用图优化库 g2o 构建位姿图，支持 SE(3) 顶点、位姿边和 IMU 预积分边。
- **ScanContext 回环检测**：使用 ScanContext 全局描述子进行位置识别，结合 ICP/NDT 几何验证实现鲁棒回环检测。
- **自定义点类型 `pointXYZVvar`**：扩展 PCL 点类型，包含位置、径向速度、方位角、距离方差、速度方差、角度方差、RCS 和 SNR 等信息。

---

## 硬件要求

| 组件 | 规格 |
|------|------|
| **雷达传感器** | Continental ARS620（CAN FD 接口）或 SR75 毫米波雷达 |
| **CAN 适配器** | ZLG USBCANFD-200U（AR620 专用） |
| **IMU（可选）** | 支持标准 ROS `sensor_msgs/Imu` 消息的 IMU |
| **计算平台** | x86_64 架构，支持 Docker 的 Linux 系统 |
| **最低 CPU** | 4 核，建议 8 核以上 |
| **最低内存** | 8 GB，建议 16 GB 以上 |

---

## 软件依赖

### 核心依赖

| 依赖 | 版本 | 用途 |
|------|------|------|
| **ROS** | Noetic | 分布式通信框架 |
| **PCL** | ≥ 1.7 | 点云处理（ICP/NDT 配准、滤波、可视化） |
| **Eigen3** | — | 线性代数、刚体变换 |
| **Sophus** | — | 李群 SE(3) 操作封装 |
| **g2o** | 含 slam3d 类型 | 图优化后端（位姿图 + IMU 预积分） |
| **OpenCV** | — | 图像处理、BEV 投影（标定） |
| **Boost** | — | 通用工具库 |
| **SuiteSparse** | — | g2o 稀疏求解器后端 |
| **spdlog** | — | 结构化日志 |
| **fmt** | — | 格式化字符串 |
| **freeglut** | — | PCL 可视化后端 |
| **googletest** | — | 单元测试框架 |

### ROS 包依赖

- `roscpp` — ROS C++ 客户端库
- `std_msgs` — 标准消息类型
- `sensor_msgs` — 传感器消息类型（PointCloud2, Imu）
- `message_filters` — 多话题时间同步
- `pcl_ros` — PCL-ROS 桥接（点云格式转换）
- `cv_bridge` — OpenCV-ROS 图像桥接
- `tf2` / `tf2_ros` — 坐标变换
- `visualization_msgs` — 可视化标记

---

## 快速开始（Docker）

### 1. 前置条件

确保已安装：
- [Docker](https://docs.docker.com/engine/install/) ≥ 20.10
- [Docker Compose](https://docs.docker.com/compose/install/) ≥ 1.29
- ZLG USBCANFD-200U 驱动（若使用 ARS620）

### 2. 构建镜像

```bash
# 克隆仓库
git clone https://github.com/gloryhry/radar_slam_docker.git
cd radar_slam_docker

# 构建 Docker 镜像
docker compose build
```

构建采用多阶段方式：
- **builder 阶段**：安装编译依赖，执行 `catkin_make install`
- **runtime 阶段**：仅装运行时依赖，从 builder 复制 install 目录

### 3. 启动容器

```bash
# 启动完整 ARS620 双雷达系统
docker compose up -d

# 进入容器交互式终端
docker compose exec radar_slam bash
```

容器启动后自动执行 `radar_canfd.launch` 启动 CAN FD 驱动。

### 4. 在容器内运行 SLAM

```bash
# 单 ARS620 雷达 SLAM
roslaunch radar_slam ars620.launch

# 双 ARS620 雷达 SLAM（前后雷达）
roslaunch radar_slam dual_ars620.launch

# SR75 雷达 SLAM
roslaunch radar_slam sr75.launch
```

### 5. 数据录制

```bash
# 录制所有雷达和 IMU 数据
roslaunch radar_canfd_driver dual_record.launch
```

录制的 rosbag 保存在容器内 `/rosbag/` 目录，对应宿主机的 `./rosbag/`。

---

## 手动构建

如果不使用 Docker，可以直接在 Ubuntu 20.04 + ROS Noetic 上构建。

### 安装依赖

```bash
# 安装 ROS Noetic（完整桌面版）
# 参考: https://wiki.ros.org/noetic/Installation/Ubuntu

# 安装系统依赖
sudo apt-get update
sudo apt-get install -y \
    cmake gcc g++ \
    libeigen3-dev \
    libboost-all-dev \
    libopencv-dev \
    libpcl-dev \
    libsuitesparse-dev \
    libspdlog-dev \
    libfmt-dev \
    freeglut3-dev \
    python3-catkin-tools

# 安装 ROS 包依赖
sudo apt-get install -y \
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-message-filters \
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-visualization-msgs

# 安装 Sophus
git clone https://github.com/strasdat/Sophus.git
cd Sophus && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# 安装 g2o（含 slam3d 类型）
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o && mkdir build && cd build
cmake .. -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF
make -j$(nproc) && sudo make install
```

### 克隆并构建

```bash
# 克隆主仓库
git clone https://github.com/gloryhry/radar_slam_docker.git
cd radar_slam_docker

# 克隆子模块（外部依赖包）
cd src
git clone https://github.com/gloryhry/radar_slam.git
git clone https://github.com/gloryhry/radar_lidar_static_calibration.git
cd ..

# 构建
source /opt/ros/noetic/setup.bash
catkin_make install -DCMAKE_BUILD_TYPE=Release

# 或使用 catkin-tools
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build

# 加载工作空间
source install/setup.bash
```

---

## ROS 包说明

### radar_canfd_driver

CAN FD 硬件驱动包，负责从 Continental ARS620 雷达读取原始数据并发布为 ROS 点云消息。

| 组件 | 说明 |
|------|------|
| **radar_canfd_node** | 主驱动节点。通过 `dlopen` 动态加载 `libcontrolcanfd.so`，打开 ZLG USBCANFD-200U 设备，配置 CH0（前雷达）和 CH1（后雷达），启动接收线程，将 ARS620 CAN FD 帧解码为 RDI（原始检测）/ OD（目标跟踪）点云并发布。支持 Y 轴翻转以适应传感器安装方向。 |
| **radar_merger_node** | 独立融合节点。订阅前后雷达话题，将后雷达点云通过 6-DOF 外参变换到前雷达坐标系，合并后发布。数据驱动模式，无固定频率。 |

**CAN 协议支持**：
- 支持 Motorola 位编码格式
- 解码字段：距离（12-bit）、径向速度（11-bit）、RCS（9-bit）、方位角（10-bit）、俯仰角（7-bit）、SNR 等
- 支持多帧周期组装（超时和部分发布机制）

**话题发布**：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ars620_front/rdi_points` | `sensor_msgs/PointCloud2` | 前雷达原始检测点云 |
| `/ars620_rear/rdi_points` | `sensor_msgs/PointCloud2` | 后雷达原始检测点云 |
| `/ars620_front/od_points` | `sensor_msgs/PointCloud2` | 前雷达目标跟踪点云 |
| `/ars620_rear/od_points` | `sensor_msgs/PointCloud2` | 后雷达目标跟踪点云 |
| `/merged_cloud` | `sensor_msgs/PointCloud2` | 前后融合点云 |

**主要启动文件**：
- `radar_canfd.launch` — 主启动（CH0 + CH1）
- `radar_merge.launch` — 独立融合
- `dual_record.launch` — 双雷达 + IMU + rosbag 录制

---

### radar_slam

雷达 SLAM 核心包，实现完整的前端-后端-回环检测流水线。

#### 可执行文件

| 可执行文件 | 功能 |
|-----------|------|
| **radarslam** | SLAM 主节点。订阅 `/radarslam_input/PointCloud2` 和 IMU 话题，串联 Map、FrontEnd、BackEnd、LoopClosing 四大组件。后台运行位姿优化和回环检测线程。发布地图点云、子图点云、当前位姿、轨迹路径和临时点云。 |
| **sr75** | SR75 转换/滤波器。订阅 `radar_enhanced_pcl`（旧格式 `sensor_msgs/PointCloud`），执行极坐标网格滤波（每扇区保留最近点）、MAD 速度异常滤除、DBSCAN 聚类滤波，转换后发布到 `RadarCombined`。 |
| **ars620** | 单 ARS620 转换器。订阅 `/ars620/rdi_points`，执行质量阈值、MAD 速度过滤、SNR/RCS 阈值过滤。 |
| **dual_ars620** | 双 ARS620 转换器。使用 `message_filters::ApproximateTime` 同步前后雷达话题，将后雷达变换到前雷达坐标系，合并并过滤。 |
| **match_test** | 配准调试工具。 |

#### 核心模块

| 模块 | 头文件 | 功能 |
|------|--------|------|
| **Frame** | `frame.h` | 单帧点云容器。包含自定义 `pointXYZVvar` 点云、时间戳、SE3 位姿、配准评分和相对运动量。 |
| **Stream** | `stream.h` | 帧链表。包含帧间 ICP/NDT 相对运动估计、离群点剔除和 ScanContext 描述子计算。 |
| **FrontEnd** | `front_end.h` | 前端处理。新帧到达时创建/追加流、执行扫描匹配、管理关键帧选取策略（评分阈值、流长度限制、最大未匹配次数）、支持 IMU 位姿预测。 |
| **BackEnd** | `back_end.h` | g2o 位姿图后端优化器。管理 SE3 顶点、速度/偏置顶点、位姿边和 IMU 预积分边，后台线程持续优化。 |
| **LoopClosing** | `loopClosing.h` | 回环检测与校正。使用 ICP/NDT 匹配和 ScanContext 检索，可配置回环步长（每 N 帧检测一次）、检测范围（距离阈值）、配准评分限制。 |
| **Map** | `map.h` | 地图存储。关键流有序映射 + 流缓存双端队列，支持局部地图提取。 |
| **ImuBuffer** | `imu_buffer.h` | 线程安全 IMU 测量缓冲区。支持静态初始化（估计重力方向和 IMU 偏置）。 |
| **ImuPreintegrator** | `imu_preintegration.h` | IMU 预积分器。在两关键帧之间预积分加速度和角速度测量，生成 15-DOF 约束边。 |
| **ScanContext** | `scancontext/Scancontext.h` | 基于扇形区域划分的全局描述子，用于地点识别。使用环形键（Ring Key）快速检索候选，通过距离函数 + 列偏移对齐实现旋转不变匹配。 |

#### 自定义点类型 `pointXYZVvar`

```cpp
struct pointXYZVvar {
    float x, y, z;          // 笛卡尔坐标
    float vel_rad;          // 径向速度
    float oritation;        // 方位角
    float range_var;        // 距离方差
    float vel_rad_var;      // 速度方差
    float ang_var;          // 角度方差
    float rcs;              // 雷达散射截面
    float snr;              // 信噪比
};
```

#### 滤波器

| 滤波器 | 说明 |
|--------|------|
| **极坐标网格滤波** | 将点云按角度划为扇区，每个扇区仅保留最近点，实现环境等距降采样。适用于 SR75。 |
| **MAD 速度异常滤除** | 基于中位数绝对偏差（Median Absolute Deviation）检测速度离群点，去除多径反射等引起的异常测量。 |
| **DBSCAN 聚类滤波** | 基于密度的空间聚类，去除孤立噪点。 |
| **SNR/RCS 阈值过滤** | 保留高信噪比和强反射目标。 |

**主要启动文件**：
- `sr75.launch` — 加载 `sr75` + `radarslam` + 静态 TF
- `ars620.launch` — 加载 `ars620` + `radarslam` + static TF
- `dual_ars620.launch` — 加载 `dual_ars620` + `radarslam` + static TF
- `match.launch` — 仅运行 `match_test`

---

### radar_lidar_static_calibration

雷达与激光雷达外参标定包，提供静态和动态两种标定方法。

#### 可执行文件

| 可执行文件 | 功能 |
|-----------|------|
| **radar_lidar_static_calibration_node** | 数据采集节点。订阅雷达和激光雷达话题，使用 `message_filters` 同步，按空格键保存成对的 PCD 文件。 |
| **calibration** | 静态标定。加载保存的雷达-激光雷达 PCD 对，在 PCL 可视化窗口中通过 Shift+点击选取对应点，使用伪逆法（最小二乘）估计仿射变换参数。 |
| **radar_match** | 动态标定。订阅雷达和里程计话题，对连续雷达帧执行 ICP 匹配，计算雷达-里程计变换，将结果写入 `calibration.csv`。 |
| **select_point** | 调试工具。加载 PCD 文件，用户点击选取点，输出坐标信息。 |

#### 标定方法

**方法一：静态标定（手动选点）**

```
雷达 PCD                       激光雷达 PCD
    │                                │
    ▼                                ▼
BEV 投影 (0.05 m/pixel)        BEV 投影 (0.05 m/pixel)
    │                                │
    ▼                                ▼
PCL Visualizer — 用户 Shift+点击选取对应点（至少 4 组）
    │
    ▼
伪逆法 → 6-DOF 外参变换矩阵
```

**方法二：动态标定（ICP 自动匹配）**

```
连续雷达帧 → ICP 帧间匹配 → 雷达里程计
里程计数据 → 里程计位姿    → 对比 → 雷达-里程计变换
```

**主要启动文件**：
- `calibration.launch` — 标定数据采集 (`/RadarDetectionImageNear_0`)
- `calibration_front.launch` — 前雷达标定 (`/ars620_front/rdi_points`)
- `calibration_back.launch` — 后雷达标定 (`/ars620_rear/rdi_points`)
- `match.launch` — 动态标定 (里程计话题 `/localization/ackermanpos`)
- `record.launch` — 全量数据录制 (tf, LiDAR, IMU, 雷达)

> 详细标定说明请参考 [`src/radar_lidar_static_calibration/README.md`](src/radar_lidar_static_calibration/README.md)

---

## 启动方式

### ARS620 系列

```bash
# 单 ARS620（前雷达）
roslaunch radar_slam ars620.launch

# 双 ARS620（前+后雷达，自动融合）
roslaunch radar_slam dual_ars620.launch

# 仅 CAN FD 驱动（不运行 SLAM）
roslaunch radar_canfd_driver radar_canfd.launch

# 仅点云融合（使用已有话题）
roslaunch radar_canfd_driver radar_merge.launch

# 完整录制（驱动 + IMU + rosbag）
roslaunch radar_canfd_driver dual_record.launch
```

### SR75 系列

```bash
# 完整 SR75 SLAM
roslaunch radar_slam sr75.launch
```

在启动之前，确保 SR75 的雷达数据已发布到对应话题。

### 坐标系关系

```
map (SLAM 地图坐标系)
│
└── odom (里程计坐标系)
    │
    └── base_link (车辆基座)
        │
        ├── ars620_front (前雷达)
        ├── ars620_rear  (后雷达)
        ├── imu          (IMU)
        └── lidar        (激光雷达，可选)
```

所有静态 TF 在对应的 launch 文件中定义，可根据实际安装位置修改。

---

## 配置说明

所有 SLAM 参数通过 YAML 文件配置，位于 `radar_slam/config/` 目录。

### 配置文件概览

| 文件 | 适用场景 | 特点 |
|------|---------|------|
| `radar_slam.yaml` | SR75 | 较短流长度（5），较大的 ICP 对应距离（2.5m），启用 RANSAC 离群剔除 |
| `ars620_slam.yaml` | ARS620 | 较长流长度（60），较紧的 ICP 对应距离（1.5m），禁用 RANSAC，更高的配准评分阈值 |

### 关键参数说明

#### 前端（FrontEnd）

```yaml
front_end:
  score_threshold: 3.0          # 关键帧选取评分阈值（越大越严格）
  stream_length: 60             # 最大流长度（超过后强制创建新流）
  max_unmatch_count: 10         # 最大未匹配帧数（超过后丢弃当前流）
  method: icp                   # 配准方法：icp / icp_nonlinear / ndt
```

#### ICP 配准

```yaml
icp:
  max_correspondence_distance: 1.5  # 最大对应点距离（太大可能误匹配，太小可能不收敛）
  transformation_epsilon: 1e-8      # 收敛阈值
  ransac_iterations: 0              # RANSAC 迭代次数（0 = 禁用）
```

#### 非线性 ICP（加权）

```yaml
icp_nonlinear:
  weight_scale: 20.0             # 距离加权缩放因子
  min_weight: 0.2                # 最小权重
```

#### NDT（正态分布变换）

```yaml
ndt:
  resolution: 5.0               # 体素分辨率（米）
  step_size: 0.1                # 优化步长
```

#### 回环检测（LoopClosing）

```yaml
loop_closing:
  loop_step: 0                  # 每 N 个关键帧检测一次（0 = 每个都检测）
  diff_num: 1                   # 与当前帧的最小索引间隔
  detect_area: 20.0             # 检测范围（米，超过此距离不检测回环）
  fitness_score_limit: 300      # 配准评分上限
  scan_context:
    enabled: true               # 启用 ScanContext
    num_candidates: 10          # 候选数量
```

#### 后端（BackEnd）

```yaml
back_end:
  regist_submaps: 10            # 配准子图数量
  method: icp                   # 后端配准方法
  max_correspondence: 1.0       # 后端 ICP 最大对应距离
  with_visualization: false     # 启用 PCL 可视化（影响性能）
```

#### IMU

```yaml
imu:
  enabled: false                # 启用 IMU 预积分
  topic: /imu/data              # IMU 话题
  static_init_duration: 1.0     # 静态初始化时长（秒）
```

### 滤波参数（针对不同雷达的启动参数）

**SR75** (`sr75.launch`)：
- `rcs_threshold` — RCS 阈值
- `mad_threshold` — MAD 离群阈值
- `dbscan_eps` — DBSCAN 邻域半径
- `angle_threshold_x/y` — 角度滤除范围

**ARS620** (`ars620.launch` / `dual_ars620.launch`)：
- `quality_threshold` — 检测质量阈值
- `mad_threshold` — MAD 速度滤除阈值
- `snr_threshold` — SNR 下限
- `rcs_threshold` — RCS 下限
- `rear_x/y/z/a/b/c` — 后雷达外参 (仅 dual_ars620)

---

## 标定流程

### 静态标定流程

1. **采集数据**：启动数据采集节点，确保雷达和激光雷达同时可见标定场景（建议使用角反射器）
   ```bash
   roslaunch radar_lidar_static_calibration calibration_front.launch
   ```

2. **保存点云对**：按空格键保存同步的雷达-激光雷达 PCD 对（建议采集 5-10 对）

3. **手动选点标定**：
   ```bash
   rosrun radar_lidar_static_calibration calibration
   ```
   在可视化窗口中，**Shift + 左键** 依次选取雷达和激光雷达中的对应点，选取至少 4 组后按 `q` 键计算并输出外参矩阵。

4. **验证结果**：将得到的外参填入 launch 文件的 TF 配置中。

### 动态标定流程

```bash
# 运行雷达帧间匹配
roslaunch radar_lidar_static_calibration match.launch

# 结果保存在 calibration.csv 中
# 格式: timestamp, x, y, z, roll, pitch, yaw
```

> 详细信息请参考 [`src/radar_lidar_static_calibration/README.md`](src/radar_lidar_static_calibration/README.md)

---

## 测试

项目包含 10 个基于 Google Test 的单元测试，覆盖以下模块：

```bash
# 在 catkin workspace 中构建并运行测试
catkin_make run_tests

# 或使用 catkin-tools
catkin build --catkin-make-args run_tests
```

| 测试文件 | 测试模块 |
|---------|---------|
| `test_sr75_velocity_filter.cpp` | SR75 MAD 速度滤除 |
| `test_sr75_dbscan_filter.cpp` | SR75 DBSCAN 聚类滤波 |
| `test_front_end_initial_pose.cpp` | 前端初始位姿配置 |
| `test_imu_preintegration.cpp` | IMU 预积分数学实现 |
| `test_imu_buffer.cpp` | IMU 缓冲区和静态初始化 |
| `test_icp_nonlinear.cpp` | 非线性 ICP 配置 |
| `test_back_end_pose_edge_information.cpp` | 后端位姿边信息矩阵 |
| `test_key_stream_publish_gate.cpp` | 关键帧发布去重逻辑 |
| `test_registration_visualizer.cpp` | 可视化节流和队列逻辑 |
| `test_scancontext.cpp` | ScanContext 描述子计算 |

---

## 项目结构

```
radar_slam_docker/
├── README.md                         # 本文件
├── DOCKERFILE                        # 多阶段 Docker 构建文件
├── docker-compose.yaml               # Docker Compose 编排配置
├── .gitignore
├── .dockerignore
│
├── data/                             # 标定数据挂载目录
│   └── .gitkeep
├── rosbag/                           # rosbag 录制挂载目录
│   └── .gitkeep
│
├── radar_slam/                       # 运行时配置/启动文件覆盖卷
│   ├── config/
│   │   ├── radar_slam.yaml           # SR75 SLAM 参数
│   │   └── ars620_slam.yaml          # ARS620 SLAM 参数
│   └── launch/
│       ├── sr75.launch               # SR75 主启动
│       ├── ars620.launch             # 单 ARS620 启动
│       ├── dual_ars620.launch        # 双 ARS620 启动
│       ├── match.launch              # 匹配测试启动
│       └── test.launch               # 通用测试启动
│
├── install/                          # 预编译 catkin install 目录
│   ├── setup.bash / setup.zsh
│   ├── lib/                          # 共享库
│   └── share/                        # 安装后包资源
│
├── src/                              # 源代码（ROS 包）
│   ├── radar_canfd_driver/           # ARS620 CAN FD 驱动 + 点云融合
│   │   ├── include/ars620_driver/    # ARS620 协议解码器
│   │   ├── include/sr75/             # ZLG CAN FD API 头文件
│   │   ├── src/                      # 驱动 + 融合节点实现
│   │   ├── launch/                   # 启动文件
│   │   └── lib/                      # libcontrolcanfd.so 预编译库
│   │
│   ├── radar_slam/                   # 雷达 SLAM 核心
│   │   ├── include/                  # 所有头文件
│   │   ├── src/                      # 主节点 + 转换器实现
│   │   ├── test/                     # 10 个 gtest 单元测试
│   │   └── AGENTS.md                 # Agent 开发指南
│   │
│   └── radar_lidar_static_calibration/  # 雷达-激光雷达外参标定
│       ├── include/                  # 标定核心头文件
│       ├── src/                      # 标定 + 数据采集实现
│       ├── launch/                   # 标定启动文件
│       ├── data/                     # 标定示例数据
│       ├── README.md                 # 标定包详细文档
│       └── install_dependencies.sh   # 依赖安装脚本
│
└── .github/
    └── workflows/
        └── docker-image.yml          # CI: Docker 镜像构建与发布
```

---

## CI/CD

项目使用 GitHub Actions 自动构建和发布 Docker 镜像。

**触发条件**：
- 推送到 `master` 分支
- 推送版本标签（`v*`）
- 手动触发（`workflow_dispatch`）
- 每日定时构建（UTC 19:17）

**流水线步骤**：
1. 解析外部仓库（`radar_slam`, `radar_lidar_static_calibration`）的最新 SHA，用于缓存键
2. 使用 `actions/cache` 缓存 Docker 层，避免无变更时重复构建
3. 克隆外部源码仓库（`--depth 1`）
4. 使用 `DOCKER_BUILDKIT=1` 构建多阶段 Docker 镜像
5. 版本标签推送时：将镜像打包为 gzip 压缩包，创建 GitHub Release 并上传构建产物

---

## 常见问题

<details>
<summary><b>Q: CAN 设备打开失败？</b></summary>

确保 ZLG USBCANFD-200U 已正确连接并被系统识别：
```bash
lsusb | grep -i zlg
# 或
ls /dev/ttyUSB*
```
检查 `libcontrolcanfd.so` 是否在 `LD_LIBRARY_PATH` 中。
</details>

<details>
<summary><b>Q: 雷达点云为空或数据异常？</b></summary>

1. 检查 CAN 波特率是否与雷达配置一致（ARS620 默认 500kbps 仲裁域，2Mbps 数据域）
2. 确认 Y 轴翻转参数（`flip_y`）是否与传感器安装方向匹配
3. 使用 `rostopic echo /ars620_front/rdi_points` 检查是否有数据发布
</details>

<details>
<summary><b>Q: SLAM 定位漂移严重？</b></summary>

1. 降低前端 ICP 的 `max_correspondence_distance`（使匹配更严格）
2. 增大 `score_threshold`（更严格的关键帧选取）
3. 启用 IMU 预积分（`imu.enabled: true`）
4. 检查回环检测参数是否合理（`loop_step`, `fitness_score_limit`）
</details>

<details>
<summary><b>Q: 如何选择 ICP / 非线性 ICP / NDT？</b></summary>

| 方法 | 适用场景 |
|------|---------|
| **ICP** | 通用场景，计算效率高，需要较好的初始位姿 |
| **非线性 ICP** | 存在较大初始误差时，距离加权提高鲁棒性 |
| **NDT** | 结构化环境（有墙壁、建筑物等），对初始位姿容忍度更高 |

建议优先使用 ICP，效果不佳时尝试 NDT 或非线性 ICP。
</details>

<details>
<summary><b>Q: Docker 容器中无法访问 GUI（PCL 可视化）？</b></summary>

在宿主机上执行：
```bash
xhost +local:docker
```
然后在 `docker-compose.yaml` 中添加环境变量 `DISPLAY` 和挂载 `/tmp/.X11-unix`。
</details>

<details>
<summary><b>Q: 如何添加新的雷达型号支持？</b></summary>

1. 在 `radar_canfd_driver` 中实现对应 CAN 协议解码器
2. 在 `radar_slam` 中添加对应的滤波/转换可执行文件
3. 创建对应的 launch 文件和 YAML 配置文件

遵循现有架构的模块化设计即可扩展。
</details>

---

## 许可证

本项目基于 **GNU General Public License v3.0 (GPLv3)** 开源。

详见 [LICENSE](src/radar_lidar_static_calibration/LICENSE) 文件。

---

### 相关项目

- [radar_slam](https://github.com/gloryhry/radar_slam) — 雷达 SLAM 核心算法
- [radar_lidar_static_calibration](https://github.com/gloryhry/radar_lidar_static_calibration) — 雷达-激光雷达外参标定工具


