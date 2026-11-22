# Fork Spatial Runtime

[![CI](https://github.com/zxinyu-robot/fork-spatial-runtime/actions/workflows/ci.yml/badge.svg)](https://github.com/zxinyu-robot/fork-spatial-runtime/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)

> 面向弱网 / 断网自主机器人的语义状态管理、时序行为记忆与端侧持续学习调度架构。
>
> A semantic-state, behavior-memory and edge continual-learning runtime for autonomous robots under weak or no connectivity.

## 项目状态

| 项目 | 状态 |
| --- | --- |
| 维护状态 | Active |
| 实现范围 | C++20、后端无关、可单元测试 |
| 当前验证 | Linux/macOS CI；无需第三方依赖的单元测试 |
| 生产后端 | 尚未接入 GTSAM、Ceres 或 g2o |

## 项目简介

项目将状态表征、历史记忆和在线决策组织成一个端侧闭环，而不是三个彼此独立的算法模块。模型推理、调度和学习都在端侧运行；云端只负责预训练、数据分析和策略分发。

| 模块 | 职责 | 关键产物 |
|---|---|---|
| Token 编码与资源管理 | 几何观测 → 语义价值 Token；管理来源、版本、时效、置信度与淘汰 | 带语义属性和资源引用的 Token |
| ESN 行为时序记忆 | 融合连续场景 Token、历史行为与执行反馈 | 动态记忆状态 `x_t` |
| 端侧在线学习与决策 | 本地推理、策略选择、任务调度；候选策略验证后发布并可回滚 | 行为决策、策略版本 |

语义价值不等于几何安全许可：可行性由规划与度量安全监督否决。ESN 动态记忆也不等于长期经验库，长期经验与策略更新归端侧学习模块。

## 核心能力

- `RuleBasedTokenEncoder` + `TokenStore`：语义编码与资源生命周期管理。
- `EchoStateMemory`：确定性稀疏储备池、washout、快照恢复和重置规则。
- `EdgePolicyManager`：active / candidate 双策略、旁路比较、发布与回滚。
- 空间状态契约：`AssetRef`、`Stamp`、`LocalizationHealth`、`ReasonCode`、`ExecutionLease`。
- `MissionOrchestrator`：机器人无关的子目标派发、到达判定、恢复阶梯和终态。
- 状态内核：因子批次校验、原子提交、固定延迟窗口和后端无关接口。
- 故障注入 v1：定位丢失、位姿跳变、换图、定位重置、过期、断链和租约失效等确定性场景。

## 架构与案例 C 展示

```text
World State → Token Encoder + TokenStore → EchoStateMemory
                         ↓
                EdgePolicyManager
                         ↓
      BT-Planner / Task Runtime + Safety Supervisor
                         ↓
              DecisionOutcome → state and learning feedback
```

下面的系统流程图用于说明空间状态、任务意图与安全动作之间的边界和回流关系；它属于本仓库的运行时与状态契约说明。案例 B 的机器人导航演示素材保留在 [quadruped-crossfloor-nav-ws](https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws)。

![空间状态到安全执行的数据状态流转](https://raw.githubusercontent.com/zxinyu-robot/quadruped-crossfloor-nav-ws/main/docs/media/state_flow_seq.png)

契约文档：[SpatialState](docs/contracts/spatial_state.md) · [SemanticToken](docs/contracts/semantic_token.md) · [BehaviorMemory](docs/contracts/behavior_memory.md) · [EdgePolicy](docs/contracts/edge_policy.md) · [MissionOrchestration](docs/contracts/mission_orchestration.md) · [状态内核架构](docs/architecture.md)

## 快速开始

要求：CMake 3.25+、Make 和支持 C++20 的编译器。CMake 3.20–3.24 可不用 preset：

```bash
cmake -S . -B build/default -DBUILD_TESTING=ON
cmake --build build/default
(cd build/default && ctest --output-on-failure)
```

使用 preset：

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default --output-on-failure
./build/default/examples/cpp/runtime_demo
```

预期演示输出：

```text
accepted graph version: 1
active states: 1
backend operations: 3
```

## 最小用法

```cpp
using namespace fork_spatial::runtime;

SpatialRuntime runtime(
    std::make_unique<MyOptimizerBackend>(),
    std::make_unique<FixedLagWindowPolicy>(2'000'000'000));

FactorBatch batch;
batch.state_requests.push_back({pose_key, initial_pose});

const SubmitResult result = runtime.submit(batch);
if (result) {
    const EstimateSnapshot estimate = runtime.optimize();
}
```

优化后端必须保证 `apply()` 的原子性。边缘化分为“计算先验”和“提交事务”两步，避免后端部分修改后运行时状态无法回滚。

## 项目结构

```text
src/runtime/       状态、因子、事务、边缘化和优化入口
src/cognition/     Token、ESN 行为记忆、端侧策略
src/mission/       参考执行器 / 测试夹具
src/contract/      跨仓库空间状态契约与故障注入
examples/cpp/      runtime、认知、任务和契约示例测试
```

## 当前边界与路线

尚未实现或不在当前版本范围内：GTSAM/Ceres/g2o 生产后端、ROS 2/DDS 适配器、共享内存传输、持久化、学习型 Token 编码器、离线预训练管线、多机器人子图融合、回环检测、可视化界面和实时性能 benchmark。

阶段与进度以[项目执行计划](docs/plan/项目执行计划.md)为准。当前 A、A2 已完成；B 剩故障演练接入 MotionSLAM 和 Go2 回放基线。

## 关联项目

- [quadruped-crossfloor-nav-ws](https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws)：案例 B，Go2 跨楼层自主导航和真机 / 回放演示。
- [legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)：定位故障恢复与多楼层导航。
- [swarm-adapter-foundation](https://github.com/zxinyu-robot/swarm-adapter-foundation)：多机器人弱网协同通信。

## 许可证

[MIT](LICENSE)
