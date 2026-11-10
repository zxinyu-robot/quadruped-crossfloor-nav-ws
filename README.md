# Quadruped Cross-Floor Navigation Workspace

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)

> 基于 ROS 2 Humble 的 Unitree Go2 跨楼层自主导航工作空间，覆盖任务编排、空间状态运行时、局部规划和安全执行。

## 项目状态

| 项目 | 状态 |
| --- | --- |
| 维护状态 | Active |
| 验证平台 | Unitree Go2 + Livox Mid-360 |
| 当前验证 | 云端 vkernel 多楼层 BT 闭环；真机在线建图与规划 |
| 尚未完成 | 真机跨楼层完整闭环、其他四足平台验证 |
| 数据集 | Elevator-LIO 数据集不入库（约 18.9 GB） |

## 项目简介

核心的任务编排、空间状态运行时与控制指令适配链路为自研实现。Go2 相关实现集中在 Forwarders 和 Sport API 适配层，迁移到其他平台时主要替换该层。

仓库已从 `MotionSLAM_ws` 更名为 `quadruped-crossfloor-nav-ws`；ROS 包、Docker 镜像和脚本仍沿用 `motionslam` 前缀，以兼容已部署的构建和启动流程。

## 核心能力

- 度量级定位、全局与局部规划。
- 行为树任务编排与任务运行时。
- Forwarders 与 Unitree Sport API 控制适配。
- 三空间模型：Metric、Symbolic、World。
- 云端虚拟设备内核中的 11 航点闭环演示（PCT → BT → SCAN → Forwarder）。
- 真机在线建图与规划；跨楼层真机闭环仍在验证。

执行约束：数据可以跨空间翻译，但执行只发生在度量空间；世界空间不得直接下发速度。完整定义见 [三空间架构](docs/架构/架构_端侧系统架构.md)。

## 案例 B：跨楼层自主导航展示

本节集中展示案例 B 的相关素材：动态环境中的足式机器人跨楼层长航时自主导航闭环。系统以 Unitree Go2 + Livox Mid-360 为验证平台，覆盖端边协同、空间状态、任务契约、行为树编排、局部规划和安全执行。

### 云端闭环演示

![云端跨楼层闭环演示](docs/media/cross_floor_demo.webp)

- [完整视频](docs/media/cross_floor_demo.mp4)
- 云端 vkernel 中已跑通多楼层 11 航点 BT 闭环：PCT → BT → SCAN → Forwarder，SubgoalReached 11/11。

### 真机在线与 Elevator-LIO 场景回放

![真机在线跨楼层与场景回放](docs/media/cross_floor_with_elevator_lio.webp)

- [完整回放视频](docs/media/cross_floor_with_elevator_lio.mp4)
- 前半段：Go2 + Livox Mid-360 真机在线建图与规划。
- 后半段：解析 Elevator-LIO 真实 ROS1 bag（`Office9.bag`）并使用 Open3D scan-to-map ICP 回放场景。
- 数据集回放是场景重建，不是官方 LIO 结果；完整导航栈在数据集上的回放基线仍在进行中。

### 数据状态流转时序图

![数据状态流转时序图](docs/media/state_flow_seq.png)

该图展示案例 B 中从任务意图、全局 / 局部规划、安全动作到设备执行与状态回流的链路。它作为本仓库的导航系统展示，不再放在运行时底座仓库的主展示区。

## 快速开始

### 获取代码和依赖

```bash
git clone --recursive https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws.git
cd quadruped-crossfloor-nav-ws
git submodule update --init
```

第三方版本以 [THIRD_PARTY.md](THIRD_PARTY.md) 为准。

### 构建和启动

环境：Jetson Orin NX、Unitree Go2、Livox Mid-360、ROS 2 Humble（Docker，host 网络）。

```bash
docker build -t motionslam:humble docker/
cp MID360_config.example.json MID360_config.json
./docker/run_container.sh
./scripts/dev/build_ws.sh
./scripts/motionslam nav start
./scripts/motionslam nav watch
./scripts/motionslam nav stop
```

启动前配置 `GO2_IP`、`EDGE_HOST`、`DEMO_PROFILE` 和可选 `WITH_FOXGLOVE` / `WITH_EDGE_UPLINK` 等开关。Jetson 系统时间必须正确，否则构建可能因 apt/gmake 失败。

## 核心运行流程

```text
Super-LIO → PCT global guidance → Behavior Tree → SCAN local planner
          → Forwarders → Unitree Sport APIs → Go2
```

1. Super-LIO 提供位姿估计。
2. PCT 提供全局路径指引。
3. 行为树负责任务调度与执行。
4. SCAN 负责局部连续路径规划。
5. Forwarders 适配控制指令。
6. Unitree Sport APIs 驱动硬件。

## 项目结构与边界

- `motionslam_bringup`：行为树编排。
- `runtime/task_runtime`：任务运行时。
- Forwarders：控制指令和 Sport API 适配。
- `config/`：真机与云端参数基线。
- `scripts/`：构建、运维和导航入口。
- `docs/`：架构、演示和验证说明。

工程集成包括 Super-LIO、PCT、SCAN-Planner、unitree_ros2 和 livox_ros_driver2；第三方代码和许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 配置注意事项

- 真机以 `config/demo_scan_planner.yaml` 为准，云端 overlay 不得回灌真机。
- 跨楼层部署前应检查相对机器人 / 分层地图的障碍物高度过滤。
- 透明障碍使用类别无关的 `virtual_obstacle_merge` 管道。
- 真实机器人操作必须由现场人员确认安全条件和急停路径。

## 关联项目

- [ForkSpatial-Runtime-Engine](https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine)：空间状态契约与运行时。
- [legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)：定位故障恢复与多楼层导航。
- [swarm-adapter-foundation](https://github.com/zxinyu-robot/swarm-adapter-foundation)：多机状态和通信策略基础设施。
- [CleanRobot-Sim2Real-skill](https://github.com/zxinyu-robot/CleanRobot-Sim2Real-skill)：复合清洁机器人操作技能学习。

## 参考工作

- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)
- [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)
- [PCT_planner](https://github.com/VectorRobotics/PCT_planner)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## 许可证

许可证以仓库中的 [LICENSE](LICENSE) 为准.
