# Spatial State Contract（空间状态契约）

## 定位

本契约是 `fork-spatial-runtime`（下称 FSR）与 `MotionSLAM_ws`、`legged-navigation-runtime`（下称 LNR）
三个仓库共用的 **身份 · 版本 · 时间 · 可信度 · 原因码** 定义。它只定义字段与消费规则，
不定义传输（DDS/SHM/UDP 在其上另行生成 IDL）。头文件：`src/contract/spatial_state.h`。

职责边界（与 `mission_orchestration.md` 一致，并按契约对齐结论收紧）：

- **执行权在 MotionSLAM 的 BT / 规划器 / 闭环 / 安全监督。** FSR 只产出"状态证据 + 策略建议"，
  不向 Sport 下发运动指令。`mission::MissionOrchestrator` 定位为参考执行器 / 测试夹具，不再扩功能。
- **Go2 的位姿、地图、Frontier 是派生状态**，经 `cognition::GeometricObservation` 一侧的适配器进入
  认知层；只有需要增量因子 / 优化事务的地方才接 `runtime::SpatialRuntime`。
- **先旁路运行（只记录不控制）再进决策路径**：Runtime 的建议先只记录到 fragment log，与 BT 实际决策并排比对；
  正确率达标后再允许进入受限行为选择。

## 1. 身份与版本链 `AssetRef`

| 字段 | 含义 | MotionSLAM 来源 | LNR | FSR 旧字段 |
|---|---|---|---|---|
| `robot_id` / `session_id` | 本次运行上下文 | `session_id` / `NavigationContextState.session_id` | — | `ResourceRef.robot_id/session_id` |
| `asset_id` | 资产身份，建议 `building/floor/map/tile` | `building_id + floor_id + map_id + tile_id` | — | `ResourceRef.asset_id` |
| `revision` | 资产**内容**版本 | `map_version` / `layer_version` | — | `ResourceRef.revision` |
| `generation` | 上下文**代数**；变化 ⇒ 旧状态/子目标/轨迹全部作废 | `generation` / `context_generation` | 楼层/地图切换 | `ResourceRef.generation` |
| `localization_epoch` | 定位有效周期；变化只作废位姿锚定状态 | `localization_epoch`（适配层补齐） | 定位重置 | `ResourceRef.localization_epoch` |
| `pose_anchored` | 是否绑定位姿锚点；决定是否受定位代数变化影响 | `pose_anchored` | — | `ResourceRef.pose_anchored` |
| `frame_id` | 坐标解释，必填无默认 | `frame_id`（Go2 默认 `world`） | — | `ResourceRef.frame_id`（原默认 `map`） |

四个字段不可互替：

- `revision` 变但 `generation` 不变 ⇒ 局部更新，消费方可继续（`same_generation()` 为真）。
- `generation` 变 ⇒ `TokenStore::invalidate_generation()`，**TTL 未到也作废**；`admit()` 返回 `Reject / AssetStale`。
- `localization_epoch` 变且状态带位姿锚定 ⇒ `admit()` 返回 `Reverify`，重新定位/重规划后才能继续；不带位姿锚定的资产不因定位重置整体作废。
- `frame_id` 为空 ⇒ `validate()` 拒收，不允许隐式 world/map。

`AssetRef` 另有可选 `asset_type`、`content_hash`。**Token 流不是身份**：`SemanticToken.token_id` 与
`asset_id` 分离，编码器 / 量化参数变化不改变资产身份。

## 1b. 增量同步 `DeltaRef` / `dispose()`

增量必须声明基线：`DeltaRef{asset_id, base_revision, target_revision, generation, codec_id, checksum}`。

```
asset / generation 不匹配      → StaleGeneration（丢弃，重同步）
target_revision <= 本地 revision → AlreadyApplied（重复投递，幂等）
base_revision == 本地 revision   → Apply
否则（本地落后 / 区间交叠）      → MissingDeltas（补齐缺失增量或请求快照；上报 ResyncRequired）
```

本地在 40、收到 `delta(base=42→43)` 时不得盲目应用——这是三个仓库目前都没有定义的规则。
`codec_id` 只是同步层内部的编码选择（原始差分 / 量化 / Token 流），资产层不感知。

## 1c. 执行令牌与 Intent Execution Contract（系统令牌，不是语义 Token）

绑定机器人的可追溯链：每条命令可查谁签发、第几任、基于哪版状态、何时过期。

```
ExecutionLease{task_id, owner_id, robot_id, epoch, issued_ns, expiry_ns,
               start_deadline_ns, execution_timeout_ns, clock, max_skew, state: AssetRef}
robot_id 不符                 → WrongRobot
epoch < 本地已接受的 epoch     → StaleEpoch（掉线的旧执行者还在发命令，拒绝）
clock 域不符                  → ClockMismatch
过期                          → Expired
state 与本地不同代/定位锚定不符 → StateMismatch（基于旧地图/旧位姿签发的命令，拒绝）
否则                          → Accept
```

`epoch` 是关键：接管一次加一；随机 UUID / 访问令牌做不到"拒绝旧任执行者"。多节点接管的协商机制后置，字段先进契约，
否则 Go2 回放日志里没有 owner/epoch，可追溯链断在起点。

幂等规则：`intent_id` 用于跨系统追踪；`idempotency_key` 用于去重。同键同内容返回原任务/原结果；同键不同内容返回 `IdempotencyConflict`。

反馈状态：`ACCEPTED / RUNNING / BLOCKED / RECOVERING / SUCCEEDED / FAILED / CANCELLED / PREEMPTED`。

## 1d. 语义 Token 的混合表示（三条规则）

1. **连续量保留数值**（台阶高度、位姿、速度、协方差）：走 SHM 给局部规划/控制，不离散化；Token 里只放引用或摘要。
2. **离散量用 Token**（地形类别、风险等级、事件、任务意图）：词表先当契约定（枚举），带 `SemanticToken.vocab_version`，不先学。
3. **稀疏结构用索引**（栅格块、关键帧、Frontier）：Token 引用 `AssetRef` + 索引，不内嵌数据。

`SemanticToken` 还要记录 `source`、`verification_status`、`evidence_ref` 和 `vocab_version`：下游模型必须能区分来源、是否已验证、依据哪条日志/恢复证据，以及使用哪一版词表。

编解码器只在异步上行链路（Token + 增量 → 边缘/云），永不进控制回路。Token 化是否值得由数字决定：
带宽、重建误差、新鲜度、过期拒绝率、P50/P95/P99 决策时延。

## 2. 时间 `Stamp`

`stamp_ns` + `valid_until_ns`（0 = 不设）+ `ClockDomain{Monotonic, WallUtc}`，统一 **ns**。
LNR 的 ms 字段在适配层乘 1e6；Go2 信封的 ns-or-ms 二选一在适配层归一后标注 `clock`。

## 3. 原因码 `ReasonCode`（唯一一套）

替代 `mission::FailureCode`（现为别名）与 `DecisionOutcome.failure_code` 字符串。

| ReasonCode | MotionSLAM 事件 | LNR |
|---|---|---|
| `NoWaypoints` | `mission_rejected` | — |
| `PlanFailed` / `PlanFailedRepeated` | `plan_fail_assess` / `plan_fail_hold` | — |
| `ReplanTimeout` | — | — |
| `Stuck` | `escape_start/step/done` | — |
| `LocalizationLost` | `/lio/relocation/converged=false` | `Tracking→Searching` |
| `LocalizationJump` | `drift_icp_start`, `/lio/backend/drift_estimate` | `pose_jump_m > maximum_pose_jump_m` |
| `RelocalizationTimeout` | — | `search_timeout_ms` 到期 → `SafeStop` |
| `FloorTransition` | `NavigationContextState.ready=false` | 楼层验证期 |
| `AssetStale` | `context_generation` 不匹配拒收 | — |
| `LinkLost` | 边端 TTL/session 校验失败 | — |
| `SafetyVeto` | `glass_slow_mode`、`glass_trap`、度量层否决（`detail` 存原事件名） | — |
| `ResyncRequired` | 增量基线缺失 / 校验失败（`dispose()` 返回 `MissingDeltas`） | — |
| `Timeout` | 任务超时 | — |
| `Canceled` | `mission_canceled` | — |

`glass_trap` 不单列：它是度量层否决的一种，归 `SafetyVeto` 并把原事件名放 `detail`。

## 4. 可信度 `LocalizationHealth` 与授权 `ExecutionAuthority`

| `LocalizationMode` | 来源映射 | `ExecutionAuthority` | 执行层行为 |
|---|---|---|---|
| `Normal3D` | Go2 `relocation_converged && stance_ok`；LNR `Tracking` | `Full` | 正常派发 |
| `Degraded2D` | Go2 `glass_slow_mode` / 2D 退化 | `Restricted` | 限速限域，继续推进（`SlowModeOn`） |
| `Relocalizing` | Go2 `drift_icp_start`；LNR `Searching/Aligning` | `Suspended` | `StopTrajectory`，任务保留 |
| `Verifying` | Go2 `relocation_fitness` 待确认；LNR `Verifying` | `Suspended` | 同上，等 `healthy_frames` |
| `Hold` | Go2 `NavigationContextState.ready=false`；LNR `SafeStop` | `Revoked` | 收回授权，需显式恢复 |

数值字段：`quality`（→ `Token.confidence` / `localization_quality`）、`pose_jump_m`、
`alignment_fitness`（→ `/lio/relocation/fitness`）、`healthy_frames`、`stance_ok`（→ `/local_map/stance_ok`）、`reason`。

重定位进度：`candidate_found`（找到候选位姿）、`alignment_complete`（精配准结束）、`alignment_success`（精配准收敛）。
LNR 状态机靠这三项离开 `Searching/Aligning`（→ LNR `descriptor_candidate_found / alignment_complete / alignment_success`）；
故障注入 `LocalizationLost / PoseJump` 清零、`Recover` 置为成功。

## 5. 认知层接入点

- `TokenStore::invalidate_generation(AssetRef current)`：同 asset 下 `generation` 落后的 Token 全部 Evicted。
- `PolicyDecision` 新增 `task_id`、`context: AssetRef`、`validity: Stamp`、`degrade_reason: ReasonCode`。
  执行层据此拒绝过期 / 跨代 / 不适用的建议。
- `DecisionOutcome` 的 `failure_code` 改为 `ReasonCode`，新增 `executed_policy_version`、`executed_context`。
  与 `decision` 内记录不一致的样本不得计入 active 统计（证据链最小字段）。

## 6. 消费规则 `admit()`

`admit()` 返回处理动作和原因码，运行期与一次性准入复用同一规则：

```
generation 不匹配 → Reject / AssetStale
stamp 过期        → Reject / AssetStale
localization_epoch 不匹配且 pose_anchored → Reverify / LocalizationLost
无法确认状态      → Hold / health.reason
状态可用          → Accept / None
```

`RecoveryOutcome{trigger, action, duration_ns, cost, resumed, post_state}`记录恢复后的验证结果，供结构化经验库使用。

## 7. 验收用例（`examples/cpp/test_contract.cpp`）

| 用例 | 验证 |
|---|---|
| 定位重置 | 旧代 Token TTL 未到也被 `invalidate_generation` 作废；旧代状态 `admit` → `AssetStale` |
| 跨楼层 | 不同 asset 永不同代；`Hold` → `Revoked`，`FloorTransition` 透传 |
| 过期 Token | `valid_until` 边界；0 = 永不过期 |
| 地图版本不一致 | 仅 revision 变化仍可消费；决策/执行 `AssetRef` 可对账；空 `frame_id` 拒收 |
| 规划失败 | `mission::FailureCode == contract::ReasonCode`；fallback 必带 `degrade_reason` |
| 安全否决 | `vetoed && failure_code == SafetyVeto` |
| 执行超时 | `Timeout` / `RelocalizationTimeout` 命名；不可用 mode 无 reason 时退化为 `LocalizationLost` |
| 增量乱序 / 缺失 | 本地 40 不得应用 `delta(42→43)`；重复投递幂等；代数变化丢弃 |
| 过期执行者 | edge_a 掉线、edge_b 以 epoch 8 接管后，edge_a 的 epoch 7 命令被拒；过期 / 异机 / 旧代 / 时钟域不符同样被拒 |
| Intent 幂等 | 同键同内容返回原任务；同键不同内容返回 `IdempotencyConflict` |
| localization_epoch | 定位代数变化只要求位姿锚定 Token 重新验证，不作废普通空间资产 |
| 恢复结果 | `RecoveryOutcome` 记录动作、耗时、代价和是否满足继续条件 |
