# SemanticTokenContract

Token 编码器把几何观测转换成带语义价值的 Token，同时管理这些状态资源的来源、有效性与生命周期。实现：`src/cognition/semantic_token.{h,cpp}`。

## Token 字段

| 字段 | 含义 |
|---|---|
| `token_id` | 稳定 ID（同一场景元素跨帧复用，便于 ESN 时序累积） |
| `kind` | Scene / Frontier / Goal / Risk / Execution |
| `attributes` | traversability、goal_relevance、information_gain、risk、priority，均为 [0,1] |
| `resource` | 资源引用：robot_id、session_id、asset_id、revision、frame_id |
| `anchor` | 目标 / frontier 锚点坐标（`resource.frame_id` 下） |
| `stamp_ns` / `ttl_ns` | 观测时间与有效期 |
| `confidence` | 置信度，由定位质量决定；低于 `min_confidence` 拒收 |
| `status` / `version` | Valid / Stale / Evicted；每次更新递增 |

## 资源生命周期

```text
upsert ─▶ Valid ──(stamp+ttl 过期)──▶ Stale ──(eviction_grace 过期)──▶ Evicted
              ▲                                   │
              └──────── 新观测 upsert ─────────────┘
定位重置 / 会话切换 ─▶ invalidate_session ─▶ Evicted
坐标系失效         ─▶ invalidate_frame   ─▶ Evicted
```

规则：
- 资源 revision 回退的 upsert 拒收（`RejectedStaleRevision`），防止旧地图覆盖新状态。
- 属性越界、TTL 非正、缺 frame_id 拒收（`RejectedInvalid`）。
- 决策只读取 `Valid` 状态的 token。

## 边界

- **语义价值 ≠ 几何安全许可**。高信息增益的 frontier 只提高探索偏好，能否到达由规划层和度量安全监督判定，被否决的结果要回流（见 EdgePolicy）。
- 编码器不做决策，也不碰几何约束。`RuleBasedTokenEncoder` 是规则化基线编码器，可以替换成学习型编码器，`ITokenEncoder` 接口不变。
