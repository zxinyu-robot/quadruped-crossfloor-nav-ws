# MissionOrchestrationContract

任务编排层的机器人无关契约。语义从 MotionSLAM_ws 的 `demo_scan_bt_orchestrator.py`（Python BT 状态机）提炼；迁移到本仓库后，Python 版在 Go2 侧降级为过渡兜底。实现：`src/mission/mission_orchestrator.{h,cpp}`（`fork_spatial::mission`）。

## 职责边界

- 编排层负责：子目标队列推进、到达判定、无进展/超时/规划失败的恢复阶梯、任务终态。
- 不负责：几何规划（PCT/SCAN 或任何 planner）、设备控制（cmd_vel/限幅/看门狗）、空间状态生产（LIO）。
- 与决策层的关系：端侧决策模型（EdgePolicy）输出的行为策略，经适配层转成 `MissionRequest` 交给本层执行；本层的恢复阶梯也是决策模型失效时的最低安全兜底。

## 状态机

```text
Idle ──submit──▶ WaitReady ──(状态源热身: 窗口内消息数 ≥ ready_min 且 settle)──▶ Running
Running ──子目标到达──▶ 派发下一子目标 ──……──▶ 最终子目标到达 ──▶ Succeeded
Running ──定位失效──▶ Recovering/Wait (StopTrajectory 先行)
Running ──首次 stuck──▶ Recovering/Wait ──wait_s──▶ Running (重派发)
Running ──再次 stuck──▶ 重派发 ≤ stuck_redispatch_max
Running ──stuck 超限 / 规划失败 ≥ replan_fail_threshold──▶ Recovering/Escape
Recovering/Escape: backup(escape_backup_m) → sidestep(escape_sidestep_m) → Running (重派发)
Recovering ──PlanFailHold 次数 > max_plan_fail_recovery──▶ Failed
任意 ──cancel──▶ Canceled
```

## 恢复阶梯（对齐 Python ExecPhase）

- **首次 stuck** → `RecoveryWait`（停 `recovery_wait_s`）→ 重派发当前子目标。
- **再次 stuck** → 直接重派发，累计 ≤ `stuck_redispatch_max`。
- **stuck 超限 / `on_plan_failed()` 累计 ≥ `replan_fail_threshold`** → `PlanFailHold`：`StopTrajectory` + escape 阶梯。
- **Escape 阶梯**：`escape:backup`（沿 −yaw 后退 `escape_backup_m`）→ `escape:sidestep`（垂直 +yaw 侧移 `escape_sidestep_m`）。每步的完成条件是位移 > `progress_dist_delta_m` **或** 超时 `escape_step_timeout_s`。escape 目标作为 `DispatchSubgoal` 命令下发，`reason` 标 `escape:*`，与正常子目标走同一条规划链路；`z = max(odom.z, 0.3)`。
- **Escape 完成** → `plan_fail_recovery_done` → 清计数并重派发当前子目标。`PlanFailHold` 累计次数 > `max_plan_fail_recovery` → `Failed`。
- **`on_pose_corrected()`**：位姿图优化/重定位收敛后的无感纠偏（Go2: `/lio/backend/pgo_state` OPTIMIZING→APPLIED）→ 重新派发当前子目标。
- **漂移评估与 ICP 重定位**（Python 的 DriftICP）不进本层：是 LIO 侧语义，由适配层处理后经 `localization_jump` 信号回流。

## 关键规则（必须遵守）

1. **到达判定三条件同时成立**：`|pose − subgoal| ≤ reach_tolerance_m`、速度 ≤ `reach_stable_speed_mps`、已执行 ≥ `min_execute_before_reach_s`。不得仅凭位置更新判定到达。
2. **只有最终子目标到达才算任务成功**；中间子目标到达立即派发下一个。
3. **定位失效时必须先停旧轨迹**（`StopTrajectory`）再进恢复，不得继续跟踪过期数据。
4. **恢复阶梯有上限**：重派发 ≤ `stuck_redispatch_max`，总恢复次数 ≤ `max_plan_fail_recovery`，超限转 `Failed`——失败要能说"不"，不能无限等。
5. 所有 `MissionCommand` 和 `MissionEvent` 携带 `subgoal_index`，适配层把它接到 `task_id → plan_id → traj_id` 版本链上，防止执行过期指令。

## 默认参数来源

`OrchestratorConfig` 默认值照抄 Python 编排器的现有参数（`subgoal_reach_m=0.45`、`stuck_timeout_s=60`、escape 距离等），保证迁移后行为可比。新增的 `reach_stable_speed_mps` 是文档契约里"速度稳定"条件的落地，Python 版目前没有。

**`reach_stable_speed_mps` 真机调优建议**：默认 0.08 m/s 经 mock 闭环验证；真机 odom twist 噪声更大，建议 0.05–0.15 m/s 区间内调——过小会因噪声迟迟判不到到达（直至 stuck），过大会提前判到达。适配层应对 `speed_mps` 做低通滤波（如 EMA α≈0.3）再喂入。
