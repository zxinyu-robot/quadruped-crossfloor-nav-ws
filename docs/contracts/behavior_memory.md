# BehaviorMemoryContract

ESN（回声状态网络）负责场景处理过程中行为策略的时序记忆。实现：`src/cognition/behavior_memory.{h,cpp}`。

## 职责边界

- ESN 只维护**近期**时序动态 \(x_t\)，回答的是“刚才发生了什么”。比如是否反复受阻、目标是否长期没有接近、切换行为后的结果如何。
- 储备池权重 `W_in` / `W_res` 由 seed 确定，固定不学习。**读出层与策略更新归 EdgePolicy**；长期经验的持久化、样本筛选和策略更新，也由 EdgePolicy 负责。

## 输入（固定维度 `kBehaviorInputDim = 12`）

| 维度 | 内容 |
|---|---|
| 0–1 | Scene traversability / risk |
| 2 | Risk token |
| 3 | Goal priority |
| 4–6 | Frontier 最高 priority / 最高信息增益 / 数量（tanh 压缩） |
| 7–10 | 上一动作、是否成功、是否被否决、进展距离（tanh） |
| 11 | bias |

## 更新

\[ x_{t+1} = (1-\alpha)\,x_t + \alpha \tanh(W_{in} u_t + W_{res} x_t) \]

其中 \(\alpha\) 是 `leak_rate`，`W_res` 按 `spectral_radius` 缩放（幂迭代估计谱半径）。更新节拍是决策节拍（建议 2–5 Hz），不进入控制周期。

## 重置规则

| 触发 | 原因码 |
|---|---|
| 任务切换 | `TaskSwitch` |
| 定位重置 / 重定位收敛 | `LocalizationReset` |
| 跨楼层切换 | `FloorChange` |

重置时把状态清零并递增 `epoch`。之后要经过 `washout_steps` 步，`warmed_up()` 才为真；冲刷期内，决策层走规则兜底。`snapshot()` / `restore()` 用于进程重启后恢复记忆。
