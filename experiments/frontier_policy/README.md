# frontier_policy：候选前沿选择策略评测

对 `demo_scan_stack` 的"搜索型任务"做 4 种候选前沿选择策略 + 3 组消融的可复现评测。

## 结论与口径

见 [`docs/实验/候选选择策略_20261127.md`](../../docs/实验/候选选择策略_20261127.md)。结果落在 `results/`：

- `summary.json`：各策略聚合指标（成功率 / SPL / 决策 P50·P95 / 行为切换 / 无效重规划率 / 里程）。
- `per_episode.csv`、`episodes.jsonl`：逐局指标（140 局）。
- `raw/<strategy>/<goal>/result.json`：逐局原始事件流与逐次决策（策略、`decision_ms`、候选数、`info_gain`、`semantic_prior`、距离）。

## 依赖的导航栈改动（在 `platform/go2` 分支）

| 文件 | 作用 |
|---|---|
| `src/motionslam_bringup/scripts/frontier_scorer.py` | `select_by_strategy()`：nearest / infogain / semantic / weighted（+ `weighted_no_dist|no_info|no_sem`） |
| `src/motionslam_bringup/scripts/subgraph_publisher_node.py` | `frontier_source:=grid`：点云在线 2D 占据建图 + 边界 frontier + `info_gain` |
| `src/motionslam_bringup/scripts/demo_scan_bt_orchestrator.py` | `frontier_strategy` 参数与决策埋点 |
| `src/motionslam_bringup/scripts/subgraph_snapshot.py` | `FrontierView.info_gain` 透传 |

## 运行

脚本按云端 pod 布局写死路径（`/root/exp/...`、`/ws/...`），复用时改 `WS_SETUP` 与 `MAP_YAML`：

```bash
# 单局：策略 + 目标 + ROS_DOMAIN_ID
python3 runner.py --strategy semantic --goal-id g06 --domain 61 --max-time 90 --max-visits 8

# 批量：8 路并行，main(4x20) + 消融(3x20)
LANES=8 MODE=all MAXTIME=90 bash launch_all.sh

# 聚合
python3 analysis.py <runs_dir>
```

## 指标定义

- 成功率：任意时刻进入目标 1.0 m 内（ObjectNav 惯例）。
- SPL：成功局 `L/max(P,L)` 均值；`L` 由 `mapdef.shortest_path_len()` 在地图真值栅格上 A\* 求得。
- 决策耗时：`select_by_strategy()` 单次墙钟；信息增益的建图/提取开销在 2 Hz 快照侧（实测 ~1.8 ms/次），不计入决策耗时。
- 行为切换：BT `phase` 变化次数。
- 无效重规划率：派发后出现 `pct_path_failed / replan_timeout / plan_fail_hold` 的派发占比。

## 已知局限

单 seed × 20 目标；成功判据为 1.0 m 内；决策耗时在 8 路并发下测量（上界）；仿真底座为 vkernel，无 Gazebo 物理。
