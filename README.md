# Quadruped Cross-Floor Navigation Runtime

[![Original code license](https://img.shields.io/badge/Original%20code-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)

> 端侧自主导航运行时：**符号层任务运行时 + 度量层导航执行栈**，两侧以原生 DDS 直连，无桥接节点。
> 两个平面都在本仓内演进：导航执行栈是 `main` 主干，端侧任务运行时经 `feat/runtime-core-overlay` 分支线并入 `main`，代码位于 `runtime_core/`。

## 项目定位

**一个系统，两个平面**：上层管「该做什么、能不能做、做完了没有」，下层管「怎么走、走不走得过去」。两层的边界是**契约**，不是函数调用。

- **符号平面**（`runtime_core/` + BT 编排）：语义观测 → `TaskIntent` 契约准入 → `StateGate` 门控 → 技能编排 → 执行反馈
- **度量平面**（导航执行栈）：Super-LIO 位姿 → PCT 全局指引 → SCAN 局部连续规划 → Forwarders → Sport API → Go2
- **接缝**：原生 CycloneDDS 直连（同机可走 iceoryx 共享内存），**只承载符号量**

**为什么收在同一仓**：任务运行时与导航栈最初靠 TCP 薄转发器连接，接缝不可验。两条分支线合到 `main` 后，两侧在同一容器、同一 Domain 内直连，接缝才有可复现的测试。

**明确不做**：端侧大模型实时推理；从世界侧下发速度；多机协同（在关联仓）。

**验证到哪**：云端 vkernel 多楼层闭环 11/11；真机在线建图与规划已通；DDS 接缝双向互通 4/4。**真机跨楼层完整闭环未完成。**

## 架构：两个平面一条接缝

```text
┌─ 符号平面（runtime_core/）──────────────────────────────────────────────────────┐
│  语义观测 → TaskIntent 契约准入 → StateGate → 技能编排 → Forwarder → 执行反馈    │
│  契约单一来源 contracts/runtime_contract.json；.msg → IDL → idlc 类型确定性生成  │
└───────────────────────────────┬─────────────────────────────────────────────────┘
                                │  DDS（CycloneDDS，无桥接；同机 iceoryx SHM）
                                │  只载 TaskIntent / ExecutionFeedback / mission event
┌───────────────────────────────┴─────────────────────────────────────────────────┐
│  度量平面（导航执行栈）                                                          │
│  Super-LIO 位姿 → PCT 全局指引 → SCAN 局部规划(tomogram+B-spline) → Forwarders  │
│  → Unitree Sport API → Go2                                                      │
└─────────────────────────────────────────────────────────────────────────────────┘
```

**三空间归属**：

```text
世界空间  World     语音 / VLM / 因果地图 / 上游世界模型 / 仿真 / 多机意图
    ↕ ② 决策翻译（任务、拒绝、策略回放；禁止下发速度）
符号空间  Symbolic  技能名 / 模式 / 事件 / 观测标签 / session·代数 / Policy
    ↕ ① 行为翻译（占据→可通行/可探索/可归档；轨迹→技能结果）
度量空间  Metric    点云·位姿·tomogram·B-spline·Sport / 本体控制器
    ↕ ③ 数据交互（版本化物料或观测 ↔ 世界；不是原始路径 dump）
```

| 翻译面 | 硬约束 | 本仓实现位置 |
|--------|--------|--------------|
| **① 行为翻译** | 失败回程用稳定类型（`reached`/`stuck`/`plan_fail`/`link_lost`），不是日志字符串 | `runtime_core/src/feedback.cpp`、BT 到达判定 |
| **② 决策翻译** | **禁止下发速度** | `runtime_core/src/admission.cpp`、`src/motionslam_bringup` |
| **③ 数据交互** | **不是原始路径 dump** | 物料上行带 `session` / `context_generation` |

**OS0**：执行只发生在度量空间。定义与编号对应见 [三空间架构](docs/架构/架构_端侧系统架构.md)。

> **BT 是调度者，不是 SCAN 的前一级**：PCT / SCAN 是技能内部的度量算子。把三者画成直线会掩盖「失败该回哪一层重决策」。

## 技术组成

| 类别 | 组件 | 本仓位置 | 用途 |
| --- | --- | --- | --- |
| 符号运行时 | `runtime_core/`（C++20 + Python 对照实现，ROS-free） | 本仓分支线 | 契约准入、门控、技能编排、反馈 |
| 编排 | `motionslam_bringup`（BT）· `runtime/task_runtime` | 本仓主干 | 任务调度与原子技能表 |
| 通信接缝 | CycloneDDS 0.10.5 · iceoryx（同机 SHM） | 本仓接缝工程 | 两平面直连，无桥接 |
| 类型链路 | rosidl 导出 → `transform_rosidl_idl.py` → `idlc` | 本仓接缝工程 | `.msg` 单一来源，生成物可校验 |
| 定位 | Super-LIO · Livox Mid-360 | 本仓主干 | 位姿估计 |
| 规划 | PCT_planner（全局）· SCAN-Planner（tomogram + B-spline 局部） | 本仓主干 | 全局指引与连续局部规划 |
| 本体 | unitree_ros2 · Forwarders · Sport API | 本仓主干 | 控制适配与硬件驱动 |
| 构建与部署 | CMake · CTest · pytest · colcon · Docker（host 网络） | 主干 + 分支线 | 端侧容器化 |
| 工具 | Foxglove Bridge · Open3D | 本仓主干 | 可视化；Elevator-LIO 场景回放 |

第三方版本与许可核查见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 演示

| 云端闭环 | 真机在线 + Elevator-LIO 回放 |
| --- | --- |
| ![云端跨楼层闭环](docs/media/cross_floor_demo.webp) | ![真机在线跨楼层](docs/media/cross_floor_with_elevator_lio.webp) |

- 云端 vkernel：多楼层 11 航点 BT 闭环，SubgoalReached 11/11（[视频](docs/media/cross_floor_demo.mp4)）
- 真机：Go2 + Livox Mid-360 在线建图与规划；后半段为 Elevator-LIO `Office9.bag` 的 Open3D 场景回放，**是场景重建、不是官方 LIO 结果**（[视频](docs/media/cross_floor_with_elevator_lio.mp4)）
- 数据状态流转时序图：[state_flow_seq.png](docs/media/state_flow_seq.png)

## 测试报告

**符号平面**（`runtime_core/`，无需 ROS）

| 项 | 结果 | 证据 |
| --- | --- | --- |
| C++ 测试 | `ctest` **2/2**（开 `-DLEGGED_NAV_BUILD_DDS=ON`；默认 OFF 时 1/1） | `runtime_core/tests/` |
| Python 测试 | `pytest` **91** 项 | `runtime_core/tests/` |
| 契约一致性 | 契约与 DDS 类型生成物 `in sync`（5 msg） | `gen_contract.py --check`、`gen_dds_types.py --check` |

**接缝**

| 项 | 结果 | 证据 |
| --- | --- | --- |
| 双向互通（原生 ↔ ROS） | **4/4 通过**，envelope / probe × 双向，字段逐项断言 | `scripts/dds_interop_check.py` |
| QoS 兼容性矩阵 | **7/7** 符合 DDS requested/offered | `scripts/dds_qos_matrix.sh` |
| SHM 实际生效 | 131 MB 载荷：SHM 关 lo 上 UDP 273.6 MB → SHM 开 **4.2 KB** | `scripts/dds_shm_proof.sh` |
| 长稳 64 KiB @ 10 Hz × 20 min × 4 轮 | 零丢失（11960/11960），p50 **1108–1134 µs**，p95 2026–2064 µs | `scripts/dds_soak_repeat.sh` |
| 接收退出语义 T4（100+1s+100）× 5 | 5/5 无提前退出，`idle=2000 ms` | `scripts/dds_sub_exit_cases.sh` |

**度量平面**

| 项 | 结果 | 证据 |
| --- | --- | --- |
| 云端多楼层闭环 | 11 航点 11/11，`mission_succeeded` | 演示视频 + `scripts/dev/cloud_sim_multifloor.sh` |
| 真机在线建图与规划 | 已通（跨楼层完整闭环未完成） | 演示视频 |

**未验证**（不写成已验证）：Humble 容器内复跑上述接缝数字、arm64 / Jetson 实机、≥2 小时长稳、writer-only / reader-only 深度拆分。

接缝的全部时序记录、踩坑与策略调整收敛在一份文档：[runtime_core/docs/dds-interop.md](runtime_core/docs/dds-interop.md)。

> 该文档记录的教训：**`dds_write` 返回成功不代表数据已交付**。曾有一批"性能数据"来自没有建立匹配关系的运行，现基准程序内置 **Match Gate**（`matched < 1` 判 `INVALID` 并拒绝出数），相关四条结论已撤回重测。

## 快速开始

### A. 符号平面自检（无需真机、无需 ROS、无需 Docker）

```bash
git clone --recursive https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws.git
cd quadruped-crossfloor-nav-ws/runtime_core

python3 scripts/gen_contract.py --check       # 契约生成物同步
python3 scripts/gen_dds_types.py --check      # DDS 类型生成物同步
cmake -S . -B build                           # LEGGED_NAV_BUILD_TESTS 默认 ON
cmake --build build -j
ctest --test-dir build --output-on-failure    # 默认 1/1
pip install -e ".[dev]" && pytest -q          # 91 项
```

要一并编译 `dds/` 下的原生类型支持（需 `find_package(CycloneDDS)` 能找到 `libddsc`）：

```bash
cmake -S . -B build -DLEGGED_NAV_BUILD_DDS=ON && cmake --build build -j
ctest --test-dir build --output-on-failure    # 2/2
```

### B. 度量平面（Docker + 真机）

前置：Jetson Orin NX（或同架构 Linux）、Docker、Go2 与 Mid-360 同网段可达、系统时间正确（时间错误会导致 apt/gmake 失败）。

```bash
# 1) 取代码（含子模块）
git clone --recursive https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws.git
cd quadruped-crossfloor-nav-ws
git submodule update --init

# 2) 雷达网络配置（真机必需；位置由 sensors.launch.py 决定）
cp src/motionslam_bringup/config/MID360_config.example.json \
   src/motionslam_bringup/config/MID360_config.json
$EDITOR src/motionslam_bringup/config/MID360_config.json   # 填现场雷达 / 主机 IP

# 3) 构建镜像（上下文是 docker/）
docker build -t motionslam:humble docker/

# 4) 容器内编译工作区（宿主机没有 ROS，必须在容器里编）
./docker/run_container.sh ./scripts/dev/build_ws.sh

# 5) 起栈 → 看日志 → 停栈（宿主机执行，nav start 会自己拉起容器）
export GO2_IP=<机器狗 IP>          # Foxglove 绑定用
export EDGE_HOST=<边端 IP>         # 可选；不用边端上行可不设
./scripts/motionslam nav start
./scripts/motionslam nav watch     # 关键事件；-f 实时跟 /demo/mission/event
./scripts/motionslam nav stop      # 急停并停栈
```

常用环境变量：`DEMO_PROFILE`（默认 `demo_short`）、`WITH_FOXGLOVE`（默认 `true`）、`WITH_EDGE_UPLINK`（默认 `false`）、`WITH_PCT_PLANNER`（默认 `true`）。完整清单见 `scripts/nav/start_demo_scan_nav.sh` 顶部。

### C. 接缝验证（容器内，可选）

```bash
./docker/run_container.sh ./scripts/dev/verify_dds.sh
```

一条命令跑完：构建 → 契约一致性 → DDS 类型一致性 → C++ 测试 → 双向互通 → QoS 矩阵。

### 注意

- `nav start` 需要镜像与 `install/` 已就绪，否则先报「镜像不存在」。
- 真机参数基线以 `src/motionslam_bringup/config/demo_scan_planner.yaml` 为准，云端 overlay 不得回灌真机。
- 透明障碍走类别无关的 `virtual_obstacle_merge` 管道；跨楼层部署前检查分层地图的障碍物高度过滤。
- 现场必须有急停路径与安全确认。

## 作者与署名

- **主导与系统设计**：zxinyu（zxiangsheng2314@gmail.com）——导航栈、三空间架构、行为树编排、端侧任务运行时、Forwarder 与真机适配层。
- **分支线归并**：端侧任务运行时经 `feat/runtime-core-overlay` 分支线并入 `main`（PR #19），代码位于 `runtime_core/`，演进历史见本仓提交记录；本仓由 `MotionSLAM_ws` 更名而来，ROS 包与脚本沿用 `motionslam` 前缀。
- **工程化与实验辅助**：契约单一来源、DDS 类型生成链路、契约/DDS/长稳实验脚本与记录文档在 GitHub Copilot 辅助下完成；所有结论可由 `runtime_core/scripts/` 与 `scripts/` 重跑复现，未经复现的数字不写入文档。
- **关联**：[ForkSpatial-Runtime-Engine](https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine)（通用空间平台，提供 ①②③ 三个翻译面的实现骨架）、[legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)、[swarm-adapter-foundation](https://github.com/zxinyu-robot/swarm-adapter-foundation)。
- **参考**：[Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)、[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)、[PCT_planner](https://github.com/VectorRobotics/PCT_planner)、[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)、[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)。

## 许可证

自研代码 [MIT](LICENSE)。`patches/` 中针对第三方项目的补丁与 `third_party/` 中的代码不在此授权范围，版权和许可依各自来源确定；外部依赖仍适用其各自许可证，具体版本与待核实事项见 [THIRD_PARTY.md](THIRD_PARTY.md)。
