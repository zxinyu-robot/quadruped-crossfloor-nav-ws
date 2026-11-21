# Quadruped Cross-Floor Navigation Workspace

[![Original code license](https://img.shields.io/badge/Original%20code-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)

> 基于 ROS 2 Humble 的 Unitree Go2 跨楼层自主导航工作空间。仓库已从 `MotionSLAM_ws` 更名而来，ROS 包与脚本仍沿用 `motionslam` 前缀。

## 项目状态

| 项目 | 状态 |
| --- | --- |
| 维护状态 | Active |
| 验证平台 | Unitree Go2 + Livox Mid-360 |
| **度量空间** | 已实现并验证：云端 vkernel 多楼层 BT 闭环 11/11；真机在线建图与规划 |
| **符号空间** | 骨架完整：BT 编排 + `TaskIntent` / `ExecutionFeedback` 契约；缺拒绝 / 策略回放的硬机制 |
| **世界空间** | **空层**：仅 `/demo/mission/semantic_goal` 意图入口；VLM、因果地图、多机意图不在本仓 |
| **端侧接缝** | 原生 CycloneDDS 直连无桥接，双向互通 4/4；**Humble 容器内运行待验证** |
| 尚未完成 | 真机跨楼层完整闭环、其他四足平台、世界侧禁速度的强制机制 |

## 三空间架构

```text
世界空间  World     语音 / VLM / 因果地图 / 上游世界模型 / 仿真 / 多机意图
    ↕ ② 决策翻译（任务、拒绝、策略回放；禁止下发速度）
符号空间  Symbolic  技能名 / 模式 / 事件 / 观测标签 / session·代数 / Policy
    ↕ ① 行为翻译（占据→可通行/可探索/可归档；轨迹→技能结果）
度量空间  Metric    点云·位姿·tomogram·B-spline·Sport / 本体控制器
    ↕ ③ 数据交互（版本化物料或观测 ↔ 世界；不是原始路径 dump）
```

| 翻译面 | 硬约束 |
|--------|--------|
| **① 行为翻译** | 失败回程用稳定类型（`reached` / `stuck` / `plan_fail` / `link_lost`），不是日志字符串 |
| **② 决策翻译** | **禁止下发速度** |
| **③ 数据交互** | **不是原始路径 dump** |

**OS0**：执行只发生在度量空间。完整定义见 [三空间架构](docs/架构/架构_端侧系统架构.md)。

## 运行流程

```text
[世界]  边端任务 JSON / 探索模式（共用同一技能表与度量内核）
   │ ② TaskIntent 契约准入 → StateGate
[符号]  BT 调度技能（ExploreFrontier / Navigate）
   │ ① 技能 → 度量目标
[度量]  Super-LIO → PCT → SCAN → Forwarders → Sport API → Go2
   │ ① 回程：轨迹 / 占据 → 技能终态
[符号]  reached / stuck / plan_fail / link_lost → 恢复阶梯
```

**BT 是调度者，不是 SCAN 的前一级**：PCT / SCAN 是技能内部的度量算子。回程只回技能终态，不回折线点；物料上行走 ③，与执行回程是两条独立通路。

## 端侧运行时（runtime_core/）

`runtime_core/` 原为独立仓库 `Autonomous-Navigation-for-Legged-Robots`，已并入本仓并停止独立维护。

| 空间 | 位置 | 契约 / 接口 |
| --- | --- | --- |
| **符号空间** | `runtime_core/`、`src/motionslam_bringup`、`runtime/task_runtime` | `motionslam.task_intent.v1` / `motionslam.execution_feedback.v1` |
| **度量空间** | `src/`（PCT / SCAN / Forwarders）、`scripts/` | `/demo/mission/semantic_goal`、`/demo/mission/event` |

接缝是 **DDS（RTPS）**：原生 CycloneDDS 参与者直连，同机可走 iceoryx（实测同样载荷下 lo 上 UDP 从 273 MB 降到 4 KB）。**接缝只承载符号量**，不得传几何量或速度指令。实测记录与踩坑见 [runtime_core/docs/dds-interop.md](runtime_core/docs/dds-interop.md)。

容器：`docker build -t motionslam:humble docker/` → `./docker/run_container.sh`；进容器后 `./scripts/dev/verify_dds.sh` 一条命令自检整条链路。

```bash
cd runtime_core
python3 scripts/gen_contract.py --check && python3 scripts/gen_dds_types.py --check
cmake -S . -B build -DLEGGED_NAV_BUILD_TESTS=ON && cmake --build build -j
ctest --test-dir build --output-on-failure
pip install -e ".[dev]" && pytest -q          # 91 项
```

## 快速开始

环境：Jetson Orin NX + Unitree Go2 + Livox Mid-360，ROS 2 Humble（Docker，host 网络）。

```bash
git clone --recursive https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws.git
cd quadruped-crossfloor-nav-ws && git submodule update --init
cp MID360_config.example.json MID360_config.json
./docker/run_container.sh
./scripts/dev/build_ws.sh
./scripts/motionslam nav start     # watch / stop
```

启动前配置 `GO2_IP`、`EDGE_HOST`、`DEMO_PROFILE`（可选 `WITH_FOXGLOVE` / `WITH_EDGE_UPLINK`）。Jetson 系统时间必须正确，否则 apt/gmake 会失败。

## 演示

| 云端闭环 | 真机在线 + Elevator-LIO 回放 |
| --- | --- |
| ![云端跨楼层闭环](docs/media/cross_floor_demo.webp) | ![真机在线跨楼层](docs/media/cross_floor_with_elevator_lio.webp) |

- 云端 vkernel：多楼层 11 航点 BT 闭环，SubgoalReached 11/11（[视频](docs/media/cross_floor_demo.mp4)）
- 真机：Go2 + Livox Mid-360 在线建图与规划；后半段为 Elevator-LIO `Office9.bag` 的 Open3D 场景回放，**是场景重建、不是官方 LIO 结果**（[视频](docs/media/cross_floor_with_elevator_lio.mp4)）
- 数据状态流转时序图：[state_flow_seq.png](docs/media/state_flow_seq.png)
- 整体演示视频（意图 → TaskIntent → 导航栈 → 反馈）待重新录制

## 配置注意事项

- 真机以 `config/demo_scan_planner.yaml` 为准，云端 overlay 不得回灌真机。
- 跨楼层部署前检查相对机器人 / 分层地图的障碍物高度过滤。
- 透明障碍走类别无关的 `virtual_obstacle_merge` 管道。
- 现场必须有急停路径与安全确认。

## 关联项目与参考工作

**关联**：[ForkSpatial-Runtime-Engine](https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine)（通用空间平台，提供 ①②③ 三个翻译面的实现骨架，两侧可替换）、[legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)、[swarm-adapter-foundation](https://github.com/zxinyu-robot/swarm-adapter-foundation)、[CleanRobot-Sim2Real-skill](https://github.com/zxinyu-robot/CleanRobot-Sim2Real-skill)。

**参考**：[Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)、[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)、[PCT_planner](https://github.com/VectorRobotics/PCT_planner)、[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)、[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)。第三方代码与许可见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 作者与署名

- **主导与系统设计**：zxinyu（zxiangsheng2314@gmail.com）——导航栈、三空间架构、行为树编排、Forwarder 与真机适配层。
- `runtime_core/` 前身为 `Autonomous-Navigation-for-Legged-Robots`，已并入本仓，演进历史保留在其提交记录中。
- 契约单一来源、DDS 类型生成链路与实验脚本/文档在 GitHub Copilot 辅助下完成；所有结论可由 `runtime_core/scripts/` 重跑复现，未经复现的数字不写入文档。
- 第三方组件版权归各自作者。

## 许可证

自研代码 [MIT](LICENSE)。`patches/` 与 `third_party/` 不在此授权范围，依各自来源确定；详见 [THIRD_PARTY.md](THIRD_PARTY.md)。
