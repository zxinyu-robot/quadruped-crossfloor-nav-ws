# ROADMAP（main：运行时）

> `main` 只按运行时管理。Go2 真机侧事项（真机复核、返航、动态障碍、跨楼层高度过滤、SCAN pin、云端仿真增强等）
> 见 [`platform/go2` 分支 ROADMAP](https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws/blob/platform/go2/docs/ROADMAP.md)。

## A. 对齐模块职责与契约（下一步）

当前同一职责有多份实现：

| 职责 | `runtime_core/` | `runtime/task_runtime/` | `fork_spatial/` |
|---|---|---|---|
| 任务状态机 | `legged_nav::TaskRuntime` | `motionslam::runtime::TaskRuntime` | `MissionOrchestrator` |
| 空间状态 / 门控 | `SpatialState` + `StateGate` | `contract_bridge`（调 FSR） | `SpatialState` / `LocalizationHealth` / `ExecutionLease` |
| 契约原件 | `contracts/runtime_contract.json` | — | `docs/contracts/*.md` |

另有 `docs/符号空间契约.md`、`interfaces/motionslam_msgs/` 两处契约描述。

- [ ] 一份「模块职责 + 交互契约」文档，契约原件收进一个目录（第一个 PR，只改文档）
- [ ] 任务运行时收成一份（建议 `runtime_core`）；`task_runtime` 的插件注册与 LNR 恢复桥并入
- [ ] 空间状态契约以 `fork_spatial` 为准，`StateGate` 改读其契约类型
- [ ] `MissionOrchestrator` 降为参考实现

## B. 接入平台（计划中）

- [ ] Go2 Robot Adapter：在 ROS overlay 外起原生进程，CycloneDDS 直连 `platform/go2` 现有话题，让真机链路经过准入与 StateGate
- [ ] HOLD 语义：Go2 编排器无暂停入口，HOLD 只能用 cancel 实现（终止而非暂停），需定取舍
- [ ] 世界状态代际：`WorldRef` / `ArtifactEpoch` 接进 `SpatialState`，代际变化后旧任务失效或重绑定（见 [架构_代数变化安全重绑定](架构/架构_代数变化安全重绑定.md)）

## C. 后续（未排期）

- 决策模型 + ESN 驱动的任务决策（`fork_spatial` 已有 Token / ESN / 端侧策略骨架）
- 空间信息离散化进模型
