# Quadruped Cross-Floor Navigation Workspace

[![Original code license](https://img.shields.io/badge/Original%20code-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)

> Unitree Go2 + Livox Mid-360 的跨楼层自主导航工作空间。仓库由 `MotionSLAM_ws` 更名而来，ROS 包与脚本沿用 `motionslam` 前缀。

## 项目定位

**是什么**：端侧导航执行栈 + 符号层任务编排。以 Jetson Orin NX 为端侧算力，驱动 Go2 完成跨楼层长航时自主导航。

**在三空间里的位置**：

- 占据 **符号空间**（BT 编排、技能名、任务契约）与 **度量空间**（定位、全局指引、局部连续规划、控制适配）；
- **世界空间只留接口**：`/demo/mission/semantic_goal` 收意图，VLM、因果地图、多机意图都在仓外。

**明确不做**：端侧大模型实时推理；从世界侧下发速度；多机协同（在关联仓）。

**验证到哪**：云端 vkernel 多楼层 BT 闭环 11/11；真机在线建图与规划已通；**真机跨楼层完整闭环未完成**。

## 架构设计

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

运行流程：世界侧意图 → ② `TaskIntent` 准入 → 符号侧 BT 调度技能 → ① 技能下发达度量目标 → `Super-LIO → PCT → SCAN → Forwarders → Sport API → Go2` → ① 回程只回技能终态 → 符号侧走恢复阶梯。

> **BT 是调度者，不是 SCAN 的前一级**：PCT / SCAN 是技能内部的度量算子。把三者画成直线会掩盖「失败该回哪一层重决策」。

## 技术组成

| 类别 | 组件 | 用途 |
| --- | --- | --- |
| 运行时 | ROS 2 Humble · C++20 · Python 3 | 节点、launch、任务运行时 |
| 通信 | CycloneDDS 0.10.5 · iceoryx（同机 SHM） | `runtime_core` 与导航栈的 DDS 接缝，无桥接节点 |
| 构建 | CMake · colcon · Docker（host 网络） | 端侧容器化部署 |
| 定位 | Super-LIO · Livox Mid-360 | 位姿估计 |
| 规划 | PCT_planner（全局） · SCAN-Planner（tomogram + B-spline 局部） | 全局指引与连续局部规划 |
| 本体 | unitree_ros2 · Forwarders · Sport API | 控制指令适配与硬件驱动 |
| 工具 | Foxglove Bridge · Open3D | 可视化；Elevator-LIO 场景回放 |

第三方版本与许可核查见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 演示

| 云端闭环 | 真机在线 + Elevator-LIO 回放 |
| --- | --- |
| ![云端跨楼层闭环](docs/media/cross_floor_demo.webp) | ![真机在线跨楼层](docs/media/cross_floor_with_elevator_lio.webp) |

- 云端 vkernel：多楼层 11 航点 BT 闭环，SubgoalReached 11/11（[视频](docs/media/cross_floor_demo.mp4)）
- 真机：Go2 + Livox Mid-360 在线建图与规划；后半段为 Elevator-LIO `Office9.bag` 的 Open3D 场景回放，**是场景重建、不是官方 LIO 结果**（[视频](docs/media/cross_floor_with_elevator_lio.mp4)）
- 数据状态流转时序图：[state_flow_seq.png](docs/media/state_flow_seq.png)

## 测试报告

| 项 | 结果 | 证据 |
| --- | --- | --- |
| 云端多楼层闭环 | 11 航点 11/11，`mission_succeeded` | 演示视频 + `scripts/dev/cloud_sim_multifloor.sh` |
| DDS 双向互通（原生 ↔ ROS） | **4/4 通过**（envelope / probe × 双向，字段逐项断言） | `scripts/dds_interop_check.py` |
| QoS 兼容性矩阵 | **7/7** 符合 DDS requested/offered | `scripts/dds_qos_matrix.sh` |
| SHM 实际生效 | 131 MB 载荷：SHM 关 lo 上 UDP 273.6 MB → SHM 开 **4.2 KB** | `scripts/dds_shm_proof.sh` |
| 长稳 64 KiB @ 10 Hz × 20 min × 4 轮 | 零丢失（11960/11960），p50 **1108–1134 µs**，p95 2026–2064 µs | `scripts/dds_soak_repeat.sh` |
| 接收退出语义 T4（100+1s+100）× 5 | 5/5 无提前退出，`idle=2000 ms` | `scripts/dds_sub_exit_cases.sh` |
| `runtime_core` 单元测试 | ctest **2/2**、pytest **91** | `runtime_core/tests/` |
| 生成物一致性 | 契约与 DDS 类型 `in sync` | `gen_contract.py --check`、`gen_dds_types.py --check` |
| CI | `main` 最新 run conclusion = **success** | GitHub Actions |

**未验证**（不写成已验证）：Humble 容器内复跑上述数字、arm64 / Jetson 实机、≥2 小时长稳、writer-only / reader-only 深度拆分。

全部实验的时序记录、踩坑清单与策略调整收敛在一份文档：[runtime_core/docs/dds-interop.md](runtime_core/docs/dds-interop.md)。

## 快速开始

### A. 端侧运行时自检（无需真机、无需 ROS、无需 Docker）

```bash
git clone --recursive https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws.git
cd quadruped-crossfloor-nav-ws/runtime_core

python3 scripts/gen_contract.py --check       # 契约生成物同步
python3 scripts/gen_dds_types.py --check      # DDS 类型生成物同步
cmake -S . -B build -DLEGGED_NAV_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure    # 2/2
pip install -e ".[dev]" && pytest -q          # 91 项
```

编译 `dds/` 下的原生类型支持需要 CycloneDDS（`find_package(CycloneDDS)`），
未安装时可跳过，上面四步仍可跑通。

### B. 导航栈（Docker + 真机）

前置：Jetson Orin NX（或同架构 Linux）、Docker、Go2 与 Mid-360 同网段可达、系统时间正确（时间错误会导致 apt/gmake 失败）。

```bash
# 1) 取代码（含子模块）
git clone --recursive https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws.git
cd quadruped-crossfloor-nav-ws
git submodule update --init

# 2) 雷达网络配置（真机必需）
#    文件位置由 sensors.launch.py 决定：包内 config/MID360_config.json
cp src/motionslam_bringup/config/MID360_config.example.json \
   src/motionslam_bringup/config/MID360_config.json
$EDITOR src/motionslam_bringup/config/MID360_config.json   # 填现场雷达 / 主机 IP

# 3) 构建镜像（上下文是 docker/）
docker build -t motionslam:humble docker/

# 4) 容器内编译工作区（宿主机没有 ROS，必须在容器里编）
./docker/run_container.sh ./scripts/dev/build_ws.sh

# 5) 起栈 → 看日志 → 停栈（这三条在宿主机执行，nav start 会自己拉起容器）
export GO2_IP=<机器狗 IP>          # Foxglove 绑定用
export EDGE_HOST=<边端 IP>         # 可选；不用边端上行可不设
./scripts/motionslam nav start
./scripts/motionslam nav watch     # 关键事件；-f 实时跟 /demo/mission/event
./scripts/motionslam nav stop      # 急停并停栈
```

常用环境变量：`DEMO_PROFILE`（默认 `demo_short`）、`WITH_FOXGLOVE`（默认 `true`）、`WITH_EDGE_UPLINK`（默认 `false`）、`WITH_PCT_PLANNER`（默认 `true`）。完整清单见 `scripts/nav/start_demo_scan_nav.sh` 顶部。

### C. 容器内验证 DDS 接缝（可选）

```bash
./docker/run_container.sh ./scripts/dev/verify_dds.sh
```

一条命令跑完：构建 → 契约一致性 → DDS 类型一致性 → C++ 测试 → 双向互通 → QoS 矩阵。

### 注意

- `nav start` 需要镜像与 `install/` 都已就绪，否则会先报「镜像不存在」。
- 真机参数基线以 `src/motionslam_bringup/config/demo_scan_planner.yaml` 为准，云端 overlay 不得回灌真机。
- 透明障碍走类别无关的 `virtual_obstacle_merge` 管道；跨楼层部署前检查分层地图的障碍物高度过滤。
- 现场必须有急停路径与安全确认。

## 作者与署名

- **主导与系统设计**：zxinyu（zxiangsheng2314@gmail.com）——导航栈、三空间架构、行为树编排、Forwarder 与真机适配层。
- `runtime_core/` 前身为 `Autonomous-Navigation-for-Legged-Robots`，已并入本仓，演进历史保留在其提交记录中。
- 契约单一来源、DDS 类型生成链路与实验脚本/文档在 GitHub Copilot 辅助下完成；所有结论可由 `runtime_core/scripts/` 重跑复现，未经复现的数字不写入文档。
- 关联：[ForkSpatial-Runtime-Engine](https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine)（通用空间平台，提供 ①②③ 三个翻译面的实现骨架）、[legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)、[swarm-adapter-foundation](https://github.com/zxinyu-robot/swarm-adapter-foundation)。
- 参考：[Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)、[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)、[PCT_planner](https://github.com/VectorRobotics/PCT_planner)、[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)、[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)。

## 许可证

自研代码 [MIT](LICENSE)。`patches/` 中针对第三方项目的补丁与 `third_party/` 中的代码不在此授权范围，版权和许可依各自来源确定；外部依赖仍适用其各自许可证，具体版本与待核实事项见 [THIRD_PARTY.md](THIRD_PARTY.md)。
