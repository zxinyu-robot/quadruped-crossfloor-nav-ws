# 契约索引

本仓所有跨模块契约的原件位置。原件不集中搬放：`runtime_contract.json` 由 `gen_contract.py` 生成 C++/Python 常量，
`fork_spatial/docs/contracts/` 随 `git subtree pull` 同步。改契约时改原件，并在同一个 PR 里更新使用方。

| 契约 | 原件 | 权威范围 | 生成物 / 校验 |
|---|---|---|---|
| 任务意图 / 执行反馈 schema、技能白名单、禁控字段 | [`runtime_core/contracts/runtime_contract.json`](../runtime_core/contracts/runtime_contract.json) | 任务入口与反馈 | `gen_contract.py --check` |
| 空间状态（身份 / 版本 / 时间 / 可信度 / 原因码 / 租约） | [`fork_spatial/docs/contracts/spatial_state.md`](../fork_spatial/docs/contracts/spatial_state.md) + `fork_spatial/src/contract/spatial_state.h` | 空间状态与执行期失败原因 | `fork_spatial` ctest |
| 任务编排参考执行器 | [`fork_spatial/docs/contracts/mission_orchestration.md`](../fork_spatial/docs/contracts/mission_orchestration.md) | 仅参考实现 | `fork_spatial` ctest |
| 语义 Token / 行为记忆 / 端侧策略 | [`fork_spatial/docs/contracts/`](../fork_spatial/docs/contracts) 下 `semantic_token.md`、`behavior_memory.md`、`edge_policy.md` | 认知层 | `fork_spatial` ctest |
| 跨进程消息 | [`interfaces/motionslam_msgs/msg/`](../interfaces/motionslam_msgs/msg) | DDS / ROS 话题类型 | `gen_dds_types.py --check` |
| 任务生命周期（task_runtime 版） | [`docs/符号空间契约.md`](../docs/符号空间契约.md) | 过渡，待并入 `runtime_core` | `runtime/task_runtime` ctest |

模块职责与各接口的字段 / 时钟 / 超时 / 执行权对照见 [模块职责与交互契约](../docs/架构/模块职责与交互契约.md)。
