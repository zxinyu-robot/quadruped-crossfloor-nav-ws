# Quadruped Cross-Floor Navigation Workspace

[![Original code license](https://img.shields.io/badge/Original%20code-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)

> 基于 ROS 2 Humble 的 Unitree Go2 跨楼层自主导航工作空间，覆盖任务编排、空间状态运行时、局部规划和安全执行。

## 项目状态

| 项目 | 状态 |
| --- | --- |
| 维护状态 | Active |
| 验证平台 | Unitree Go2 + Livox Mid-360 |
| **度量空间** | 已实现并验证：云端 vkernel 多楼层 BT 闭环 11/11；真机在线建图与规划 |
| **符号空间** | 有骨架：BT 编排 + 任务契约（`TaskIntent` / `ExecutionFeedback`），缺「拒绝 / 策略回放」的硬机制 |
| **世界空间** | **空层**：仅 `/demo/mission/semantic_goal` 意图入口；VLM、因果地图、多机意图不在本仓 |
| **③ 数据交互** | 最弱：`session` / `context_generation` 有约定，无代码强制 |
| 端侧运行时接缝 | 原生 CycloneDDS 直连，**无桥接节点**；单机双向互通 4/4 通过 |
| 接缝验证边界 | 构建/契约/互通/QoS/长稳已在原生环境验证；**Humble 容器内运行待验证** |
| 尚未完成 | 真机跨楼层完整闭环、其他四足平台验证、世界侧禁速度的强制机制 |
| 数据集 | Elevator-LIO 数据集不入库（约 18.9 GB） |

## 项目简介

核心的任务编排、空间状态运行时与控制指令适配链路为自研实现。Go2 相关实现集中在 Forwarders 和 Sport API 适配层，迁移到其他平台时主要替换该层。

仓库已从 `MotionSLAM_ws` 更名为 `quadruped-crossfloor-nav-ws`；ROS 包、Docker 镜像和脚本仍沿用 `motionslam` 前缀，以兼容已部署的构建和启动流程。

## 核心能力

- 度量级定位、全局与局部规划。
- 行为树任务编排与任务运行时。
- Forwarders 与 Unitree Sport API 控制适配。
- 三空间模型：Metric、Symbolic、World（见下节）。
- 云端虚拟设备内核中的 11 航点闭环演示（BT 调度 → PCT 全局指引 / SCAN 局部规划 → Forwarder）。
- 真机在线建图与规划；跨楼层真机闭环仍在验证。

## 三空间架构

```text
世界空间  World     语音 / VLM / 因果地图 / 上游世界模型 / 仿真 / 多机意图
    ↕ ② 决策翻译（任务、拒绝、策略回放；禁止下发速度）
符号空间  Symbolic  技能名 / 模式 / 事件 / 观测标签 / session·代数 / Policy
    ↕ ① 行为翻译（占据→可通行/可探索/可归档；轨迹→技能结果）
度量空间  Metric    点云·位姿·tomogram·B-spline·Sport / 本体控制器
    ↕ ③ 数据交互（版本化物料或观测 ↔ 世界；不是原始路径 dump）
```

| 翻译面 | 输入 → 输出 | 硬约束 |
|--------|------------|--------|
| **① 行为翻译** | 占据 / 轨迹 → 可通行 / 技能结果 | 失败回程使用稳定类型（`reached` / `stuck` / `plan_fail` / `link_lost`），不是日志字符串 |
| **② 决策翻译** | 任务 / 拒绝 / 策略回放 | **禁止下发速度** |
| **③ 数据交互** | 版本化物料或观测 | **不是原始路径 dump** |

**OS0**：数据可以跨空间翻译，**执行只发生在度量空间**。完整定义与编号对应（`OS1` = ①、`OS2` = ②、`OS3` = ③）见 [三空间架构](docs/架构/架构_端侧系统架构.md)。

## 核心运行流程

一次完整闭环要跨三个空间。关键约束：**符号空间只调度技能，不调度折线点**；几何计算全部留在度量空间。

```text
[世界空间]  边端任务 JSON / 探索模式（两种意图源，共用同一技能表与同一度量内核）
                │ ② 决策翻译：TaskIntent 契约准入 → StateGate
                ▼
[符号空间]  BT 编排 → 选技能（ExploreFrontier / Navigate）→ 目标与约束下发
                │ ① 行为翻译：技能 → 度量目标（几何量不得回写符号面）
                ▼
[度量空间]  Super-LIO 位姿 → PCT 全局指引 → SCAN 局部连续规划 → Forwarders → Sport API → Go2
                │ ① 行为翻译回程：占据 / 轨迹 → 技能终态
                ▼
[符号空间]  reached / stuck / plan_fail / link_lost → 恢复阶梯 → 终态判定
```

**单次 `Navigate` 技能内部的度量侧数据流**（即上面第三行的展开，不是全局串联顺序）：

1. **Super-LIO** 提供位姿估计。
2. **PCT** 提供全局路径指引，作为该技能的全局约束。
3. **SCAN** 在 tomogram 上做局部连续路径规划（B-spline）。
4. **Forwarders** 把路径适配为控制指令。
5. **Unitree Sport APIs** 驱动 Go2。

**三条容易搞错的地方**：

- **BT 不是 SCAN 的前一级，而是调度者**：BT 选技能、盯子目标与超时；PCT / SCAN 是技能内部的度量算子。把它们画成一条直线会掩盖「失败回哪一层重决策」这件事。
- **回程只回技能终态**（① 行为翻译），不回折线点。折线是度量空间的瞬时产物，出了度量空间就没有意义。
- **物料上行走 ③**（点云 / BEV / Token 带 `session` / `context_generation`），与执行回程是两条独立的通路，不要混成一条。

## 端侧运行时（runtime_core/）

`runtime_core/` 原为独立仓库 `Autonomous-Navigation-for-Legged-Robots`，
**已并入本仓并停止独立维护**（该仓库仅保留归档说明）。它落在符号空间，与 BT 编排同层：

| 空间 | 位置 | 负责 | 契约 / 接口 |
| --- | --- | --- | --- |
| **符号空间** | `runtime_core/`、`src/motionslam_bringup`、`runtime/task_runtime` | 视觉 → 语义观测 → TaskIntent 契约准入 → StateGate → 技能编排与派发 → 执行反馈 | `motionslam.task_intent.v1` / `motionslam.execution_feedback.v1` |
| **度量空间** | `src/`（PCT / SCAN / Forwarders）、`scripts/` | 全局指引 → 局部连续规划 → cmd_vel forwarder → 平台 | `/demo/mission/semantic_goal`、`/demo/mission/event` |

两层接缝是 **DDS（RTPS）**：`runtime_core` 作为原生 CycloneDDS 参与者直接与导航栈通信，
不经过任何桥接节点。同机可用 iceoryx 共享内存通路，实测同样载荷下 lo 上 UDP 字节从 273 MB
降到 4 KB。

**这条接缝只承载符号量**（`TaskIntent` / `ExecutionFeedback` / mission event）。世界侧的内容必须经
**② 决策翻译**变成符号意图，才能进入本接缝；它**不得**成为几何量或速度指令的通路。
任务运行时不下发速度，执行权始终留在度量空间。

实测记录、踩坑清单与策略调整全部收敛在**一份**文档：
[runtime_core/docs/dds-interop.md](runtime_core/docs/dds-interop.md)（按时序组织）。

> 该文档记录了本链路最值得记住的一条：**`dds_write` 返回成功不代表数据已交付**。
> 一次没有建立匹配关系的运行曾产出过看似正常的接收数字，因此现在基准程序内置
> **Match Gate**（`matched < 1` 直接判 `INVALID` 并拒绝出数），
> 并区分 `READY` / `MATCHED` / `RECEIVING` / `IDLE` / `DONE` / `NO_DATA` 状态。
> 修复有效性问题之前得出的四条性能结论已全部撤回重测。

容器化：`docker/Dockerfile` 已加入 DDS 链路依赖（`ros-humble-cyclonedds` 自带 `libddsc`+`idlc`，
iceoryx 为其硬依赖），`docker/cyclonedds.xml` 已开启同机共享内存；
进容器后 `./scripts/dev/verify_dds.sh` 一条命令跑完整链路自检。

`runtime_core/` 自带构建与测试（CMake + CTest、pytest），契约常量的单一来源是
`runtime_core/contracts/runtime_contract.json`：

```bash
cd runtime_core
python3 scripts/gen_contract.py --check        # 契约生成物同步
python3 scripts/gen_dds_types.py --check       # DDS 类型生成物同步
cmake -S . -B build -DLEGGED_NAV_BUILD_TESTS=ON && cmake --build build -j
ctest --test-dir build --output-on-failure     # 231 项契约检查
pip install -e ".[dev]" && pytest -q           # 91 项
```

> 整体演示视频（视觉意图 → TaskIntent → 导航栈 → 执行反馈）待重新录制后补充；
> 原先独立仓库的 ARKitScenes 回放视频已随迁移下线。

## 案例 B：跨楼层自主导航展示

本节集中展示案例 B 的相关素材：动态环境中的足式机器人跨楼层长航时自主导航闭环。系统以 Unitree Go2 + Livox Mid-360 为验证平台，覆盖端边协同、空间状态、任务契约、行为树编排、局部规划和安全执行。

### 云端闭环演示

![云端跨楼层闭环演示](docs/media/cross_floor_demo.webp)

- [完整视频](docs/media/cross_floor_demo.mp4)
- 云端 vkernel 中已跑通多楼层 11 航点 BT 闭环：BT 调度 → PCT 全局指引 / SCAN 局部规划 → Forwarder，SubgoalReached 11/11。

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

上述命令不会自动取得历史快照中的第三方组件；版本及许可核查状态见 [THIRD_PARTY.md](THIRD_PARTY.md)。

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

## 项目结构与边界

按三空间归位，每层标注实现与状态：

**度量空间（实现最实）**

- `scan_planner`（SCAN-Planner）：tomogram、B-spline 局部连续路径规划。
- Super-LIO：位姿估计；PCT：全局路径指引。
- Forwarders：控制指令与 Unitree Sport API 适配。
- `config/`：真机与云端参数基线（真机以 `config/demo_scan_planner.yaml` 为准）。

**符号空间（骨架完整）**

- `motionslam_bringup`：行为树编排与原子技能表。
- `runtime/task_runtime`：ROS 侧任务运行时。
- `runtime_core/`：ROS-free 端侧任务运行时（TaskIntent 契约 / 准入 / 门控 / 技能 / Forwarder）。
- `scripts/`：构建、运维与导航入口。

**世界空间（仅接口）**

- `/demo/mission/semantic_goal`：意图入口；`/demo/mission/event`：事件回程。
- VLM、因果地图、上游世界模型、多机意图均不在本仓；仿真在云端 `vkernel_sim`。

工程集成包括 Super-LIO、PCT、SCAN-Planner、unitree_ros2 和 livox_ros_driver2；第三方代码和许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 配置注意事项

- 真机以 `config/demo_scan_planner.yaml` 为准，云端 overlay 不得回灌真机。
- 跨楼层部署前应检查相对机器人 / 分层地图的障碍物高度过滤。
- 透明障碍使用类别无关的 `virtual_obstacle_merge` 管道。
- 真实机器人操作必须由现场人员确认安全条件和急停路径。

## 关联项目

- [ForkSpatial-Runtime-Engine](https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine)：通用空间平台，提供 ① 行为翻译 / ② 决策翻译 / ③ 数据交互 三个面的实现骨架，世界侧与度量侧均可替换。
- [legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)：定位故障恢复与多楼层导航。
- [swarm-adapter-foundation](https://github.com/zxinyu-robot/swarm-adapter-foundation)：多机状态和通信策略基础设施。
- [CleanRobot-Sim2Real-skill](https://github.com/zxinyu-robot/CleanRobot-Sim2Real-skill)：复合清洁机器人操作技能学习。

## 参考工作

- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)
- [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)
- [PCT_planner](https://github.com/VectorRobotics/PCT_planner)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## 作者与署名

- **项目主导与系统设计**：zxinyu（zxiangsheng2314@gmail.com）。导航栈、三空间架构、行为树编排、
  Forwarder 与真机适配层的设计口径与实现由作者主导；云端多楼层闭环与真机在线建图/规划均为本仓自有工作。
- **原独立仓库并入**：`runtime_core/` 前身为 `Autonomous-Navigation-for-Legged-Robots`，
  已整体并入本仓并停止独立维护，原仓库的演进过程保留在 `runtime_core/` 的提交历史中。
- **工程化与实验辅助**：契约单一来源、DDS 类型生成链路、互通/QoS/SHM/长稳实验脚本与记录文档，
  在 GitHub Copilot 辅助下完成；所有实验结论均可由 `runtime_core/scripts/` 下的脚本重跑复现，
  未经复现的数字不写入文档。
- **第三方组件**：Super-LIO、PCT、SCAN-Planner、unitree_ros2、livox_ros_driver2 等版权归各自作者，
  许可与版本核查见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 许可证

本仓库的自研代码采用 [MIT](LICENSE)。`patches/` 中针对第三方项目的补丁以及 `third_party/` 中的代码不属于此授权范围，其版权和许可依各自来源确定。外部依赖仍适用其各自的许可证；具体版本、已知问题和待核实事项见 [THIRD_PARTY.md](THIRD_PARTY.md)。本仓库的 MIT 许可证不改变组合或分发第三方组件时的许可义务。
