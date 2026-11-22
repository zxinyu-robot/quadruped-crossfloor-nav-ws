# EdgePolicyContract

端侧在线学习与决策模型负责三件事：利用 Token 和 ESN 记忆生成行为策略、完成本地调度、根据反馈适应新场景。断网时照常工作。实现：`src/cognition/edge_policy.{h,cpp}`。

## 两条路径

| 路径 | 使用 | 约束 |
|---|---|---|
| 实时决策 | `decide(tokens, behavior_input, memory)`（`behavior_input` 必须与最近喂给 `memory.update()` 的向量一致） → `PolicyDecision{action, target_token, policy_version, fallback, reason}` | 只用 **active** 策略；不阻塞、不在线修改 |
| 在线学习 | `report(outcome, features)` | 只更新 **candidate** 策略，并累积经验缓冲 |

行为集合：Explore / SwitchFrontier / Wait / Recover / ReturnHome。决策输出交给任务执行层（BT-Planner / Task Runtime）落成技能序列。

## 版本管理

```text
candidate ──(旁路比对: samples ≥ min_validation_samples,
             veto_rate ≤ max_veto_rate, success_rate ≥ active)──▶ promote → active
active ──(连续失败 ≥ rollback_failure_streak)──▶ rollback → previous / 规则兜底
```

- 每次 promote 都分配新的 `policy_version`，并保留上一版本用于回滚。
- 决策和反馈都携带 `policy_version`，只有当前 active 版本的反馈才计入 active 统计。

## 兜底与否决

- 以下情况走 `rule_fallback`，并标记 `fallback=true` 和 reason：没有 active 策略、ESN 处于冲刷期、选出的动作没有可用目标。机器人侧还可以替换成已验证的 BT 策略。
- **度量层否决必须回流**：`DecisionOutcome.vetoed=true` 加上 `failure_code` 记为负奖励，防止模型反复选同一个不可达的高价值目标。

## 断网保障

模型推理、调度、学习、验证、发布全部在端侧闭环。云端只用于预训练、数据分析和策略分发，不是基本自主调度的必要依赖。
