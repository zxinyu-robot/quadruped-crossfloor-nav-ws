# 证据口径

本仓库 README 不写指标数字，所有结论的分级与出处集中在本文件。等级分级（与项目工程规范一致）：真机闭环 / 真机部分 / 数据集回放 / 云端仿真 / 合成数据·单元测试 / 脱敏重构 / 计划中。

## 当前证据表

| 数字/结论 | 平台 | 等级 | 测量方法 | 出处 | 日期 |
|---|---|---|---|---|---|
| 契约校验拒绝码 14 类覆盖（schema/平台/禁区/禁控字段/参数） | 本机 CPU | 合成数据·单元测试 | pytest 参数化用例 | tests/test_task_intent.py | 2026-11-19 |
| 端到端闭环（accepted→started→progress→succeeded） | 本机 CPU | 合成数据·单元测试 | SimForwarder + TaskRuntime | tests/test_runtime.py | 2026-11-19 |
| 状态失效（过期/版本不一致/低置信度）进入 HOLDING | 本机 CPU | 合成数据·单元测试 | StateGate | tests/test_runtime.py | 2026-11-19 |
| 视觉闭环（帧→观测→意图→反馈），stub 后端 | 本机 CPU | 合成数据·单元测试 | VisionLoop + StubVLMBackend | tests/test_vision_loop.py | 2026-11-19 |
| 数据链路层（通道表/QoS/禁控 payload/pub-sub），进程内后端 | 本机 CPU | 合成数据·单元测试 | InProcessTransport | tests/test_transport.py | 2026-11-19 |
| 空间状态契约（快照/增量/有效期过滤）与 Fast-Slow 流式上下文（滑动窗口/压缩/LIO 去重/KV 复用统计） | 本机 CPU | 合成数据·单元测试 | SpatialStateStore + StreamContext | tests/test_spatial_context.py | 2026-11-19 |
| C++20 权威核心（契约/准入/门控/技能/运行时/反馈/传输/流式上下文/视觉编排），231 项检查（含 44 条 C++/Python 共用契约语料） | 本机 CPU | 合成数据·单元测试 | CTest tests/test_all.cpp | tests/test_all.cpp | 2026-11-19 |
| Cyclone DDS 端侧单机链路（SHM + 跨进程 RTPS/UDP；跨机不属于本仓范围） | Jetson/真机 | 计划中 | CycloneDDSTransport（接口就位） | src/legged_nav/transport.py | — |
| Qwen2.5-VL-7B-Instruct-AWQ 端到端闭环（ARKitScenes RGB+traj 位姿+yaw → vLLM → VisionLoop → TaskRuntime → SimForwarder → feedback） | RTX 4090 + vLLM(awq_marlin) | 数据集回放 | 15/15 帧 Navigate 成功；VLM P50 0.42s；逐帧 yaw 消费（-82°~92.8° 转弯段）；Runtime 事件 60 条；Fast-Slow fast=4/slow=11/KV 复用 0.73/里程 27.1m；本帧批次 VLM 未检出 hazard（深度门控逻辑已由本地回放验证） | scripts/e2e_arkitscenes_awq.py | 2026-11-19 |
| ARKitScenes 深度门控回放（观测回放 + lowres_depth 走廊近场门控 + traj 轴角→yaw） | 本机 CPU | 数据集回放 | 15 帧：走廊最小深度 0.94–2.22m；杯瓶 hazard 帧（走廊 1.01m > 0.9m 阈值）正确放行 Navigate 不误 Stop；yaw 逐帧变化（转弯段 87.8°）；dedup 3、slow 压缩 8、KV 复用 0.67、里程 21.3m | scripts/e2e_arkitscenes_awq.py | 2026-11-19 |
| Jetson Orin NX 真机部署（TensorRT + INT4-AWQ 吞吐实测） | Jetson Orin NX | 计划中 | QwenVLBackend（接口就位；vLLM/AWQ 路径已验证） | src/legged_nav/vlm_adapter.py | — |
| Elevator-LIO Office3 闭环回放（相机帧 → AWQ vLLM → VisionLoop → TaskRuntime；IMU/LiDAR 通道已抽取对齐，位姿不合成故 Fast-Slow 去重未启用） | RTX 4090 + vLLM(awq_marlin) | 数据集回放 | 25 帧闭环：Navigate×5 / Stop×10（hazard：obstacles、clutter、sharp corner、broken chair 等）/ 拒发×10（no_actionable 9 + low_confidence 1）；VLM P50 0.78s；Runtime 事件 68 | scripts/e2e_elevatorlio_awq.py | 2026-11-19 |
| 真机闭环（Sport API、定位、规划） | Unitree Go2 | 计划中 | — | — | — |

## 口径说明

- `SimForwarder` 记录指令流并更新仿真位姿，不产生真实运动指令；其结论不代表真机运动能力。
- ARKitScenes `.traj` 是数据集提供的相机轨迹参考，按时间戳最近邻与帧关联；未经独立外参校验，不作为绝对测量真值。实测列序为 `ts tx ty tz rx ry rz`（平移在前、轴角在后，与部分文档描述相反，以文件数值量纲为准）；yaw 由轴角经 Rodrigues 投影得到。
- 障碍判定为语义+空间两级：VLM 检出 hazard 类别后，再由同名深度图前向走廊近场最小深度（阈值 0.9m）门控——物体在桌上不影响通行时不触发 Stop；无深度图时保守保留 hazard。该门控是规则层近似，不等于模型自身的碰撞风险评估。
- `StubVLMBackend` 为预设输出的替身：链路验证 ≠ 模型视觉感知能力。真推理证据需注明实际加载权重名与量化格式（如 INT4-AWQ/TensorRT）及实测吞吐、时延。
- 配置值不等于实测值：`deadline_s`、`ttl_ns`、`min_confidence` 是门限参数，不是测得指标。
- 部署侧数据（真机时延、到达率、避障率）按平台另行标注，不得并入本仓仿真证据。
