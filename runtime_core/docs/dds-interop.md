# 单机 DDS 链路：实测记录、踩坑与策略调整（按时序）

> 本文是这条链路**唯一**的测试记录文档，按时序组织。「踩坑清单」「策略调整清单」在第 10、11 节。
> 主线环境：**ROS 2 Humble + Docker + CycloneDDS 0.10.5 + iceoryx 2.0.5**（Ubuntu 22.04 jammy）。
> 原生 Jazzy 环境只作为探索阶段的历史参考，不作为本仓验收依据。
> 所有数字都可用 `scripts/` 下脚本重跑复现；仓库内不再保留散落的报告文件。

---

## 0. 接缝的空间归属（先定边界，再谈性能）

按仓根 `docs/架构/架构_端侧系统架构.md` 的三空间约束，**这条 DDS 接缝只允许承载符号量**：

| 空间 | 负责 | 禁止 | 本接缝的角色 |
| --- | --- | --- | --- |
| 世界空间 | 因果、场景语义、策略记忆（边端 VLM / 世界模型在此） | 下 `/cmd_vel`、`/motion/command` | 只作为意图来源，不直连本接缝的度量侧 |
| **符号空间** | 技能名、任务生命周期、失败类型、仲裁结果 | 规划器直写执行速度 | **本接缝承载的就是这一层**（`TaskIntent` / `ExecutionFeedback` / mission event） |
| 度量空间 | 几何、占据、可行通行、连续轨迹、限幅速度 | — | 执行只发生在这里 |

**OS0**：翻译可跨空间，**执行只发生在度量空间**。

因此 `runtime_core` 经 DDS 交换的是**已被符号空间裁决过的意图与回执**，
它**不得**成为几何量或速度指令的通路——否则等于用 DDS 绕过 OS0。
模型本体在世界空间，但它产出的 `TaskIntent` 属于符号空间；
**adapter-BT 是符号 → 度量的唯一接缝**：把意图翻成可调度技能，把度量结果翻回符号状态。

> 口径澄清：`TrajectoryProbe` / `PosePoint` 是**传输能力探针**，
> 只用来验证「嵌套标准消息 + 自定义消息数组 + string + 原生数组」的 CDR 编解码，
> **不构成符号面契约，也不代表 runtime_core 会向导航栈传递轨迹**。
> 真正的符号面契约是 `motionslam.task_intent.v1` / `motionslam.execution_feedback.v1`。

---

## 1. P0 · 互通可行性（最先解决的是"能不能通"）

三个结论，全部来自实测端点公告而非源码猜测：

| 项 | 结论 | 证据 |
| --- | --- | --- |
| ROS topic → DDS topic | 必须加 `rt/` 前缀 | RTPS 抓包 SEDP 公告 `rt/demo/mission/event` 等，`rt/` 出现 201 次 |
| DDS type name | `pkg::msg::dds_::Type_` | 抓包 `std_msgs::msg::dds_::String_` |
| 双向互通 | Native→ROS、ROS→Native 均通过 | `matched=1` 且字段值逐项断言 |

**踩坑 A（最贵的坑）**：用 `strings librmw_cyclonedds_cpp.so | grep "rt/"` 判断前缀，得到 0 个匹配，
于是错误地得出"rmw 不加前缀"。实际上 `rt` 只有 2 个字符，**低于 `strings` 默认最短长度 4**，
且 `"rt"` 与 `"/"` 是两个独立字面量。教训：**否定结论必须用活体观测验证，不能靠二进制字符串推断**。

**踩坑 B**：type name 少写 `dds_::` 与尾下划线，端点永远匹配不上，且**没有任何报错**。

---

## 2. P1 · 类型生成链路（A1 路线）

`.msg` 是唯一人工维护的定义，其余全部生成：

```
pkg/msg/Type.msg → rosidl 导出 .idl 快照 → transform_rosidl_idl.py → idlc → 原生类型支持
```

rosidl 导出的 IDL 用 `module pkg { module msg { struct Type } }`（没有 `dds_` 和尾下划线），
直接交给 idlc 会生成错误的 `m_typename`，端点匹配不上。转换只做三件结构性改写，不做字符串替换。

**踩坑 C**：`idlc` 会把**绝对路径**写进文件头与 include guard，导致换目录生成字节不同。
归一化为 `<DDS_ROOT>` 与 `DDSC__<PKG>__<TYPE>_H` 后，`--check` 才能跨机器比对。

**踩坑 D**：`idlc` 总是输出 `DDS_TOPIC_XTYPES_METADATA`；版本 0.10.5 **没有** QoS Provider API。
`@hashid` 是 XTypes 成员 ID 用的，与 ROS 的 RIHS01 哈希不是一套机制，不能混为一谈。

---

## 3. P2 · QoS 兼容性矩阵（7/7 符合 requested/offered）

| # | writer（offered） | reader（requested） | 期望匹配 | 实测 |
| --- | --- | --- | --- | --- |
| 1 | RELIABLE / VOLATILE | RELIABLE / VOLATILE | 是 | 匹配 |
| 2 | RELIABLE / VOLATILE | BEST_EFFORT / VOLATILE | 是 | 匹配 |
| 3 | BEST_EFFORT / VOLATILE | RELIABLE / VOLATILE | 否 | 未匹配 |
| 4 | BEST_EFFORT / VOLATILE | BEST_EFFORT / VOLATILE | 是 | 匹配 |
| 5 | RELIABLE / TRANSIENT_LOCAL | RELIABLE / VOLATILE | 是 | 匹配 |
| 6 | RELIABLE / VOLATILE | RELIABLE / TRANSIENT_LOCAL | 否 | 未匹配 |
| 7 | RELIABLE / TRANSIENT_LOCAL | RELIABLE / TRANSIENT_LOCAL | 是 | 匹配 |

**关键风险**：两侧 QoS 不兼容时**没有任何报错**，只有 `matched` 悄悄为 0。因此必须把
`matched` 当断言，而不是只看进程是否存活。

**踩坑 E**：脚本没 source ROS，`libiceoryx_binding_c.so` 找不到，7 组全部 rc=127 —— 看起来像"全不匹配"。

**踩坑 F**：`transient_local` 填进了 reliability 槽位，被实现静默当成 RELIABLE，导致 durability 那一维根本没被测到。
修复：程序对非法 QoS 词直接报错退出；脚本回显被测程序**实际施加**的 QoS 并与之比对。

---

## 4. P3 · SHM 吞吐基准

| 载荷 | SHM 关 | SHM 开 |
| --- | --- | --- |
| 64 KiB | 106 MB/s · p50 118 ms | 1628–2192 MB/s · p50 3.8 ms |
| 1 MiB | 126 MB/s · p50 379 ms | 1029 MB/s · p50 2.5 ms |
| 1 KiB | 204 MB/s · p50 1.2 ms | 408 MB/s（小包收益有限） |

---

## 5. P4 · SHM 是否**实际**生效（用 UDP 流量反证）

配置里写了 `<SharedMemory><Enable>true` 只证明"被解析"，不证明"被使用"。判据用 lo 上的 UDP 字节数：

| 用例 | 载荷 | got | lo UDP pcap 字节 | UDP 包数 |
| --- | --- | --- | --- | --- |
| SHM 关 | 2000 × 64 KiB = 131 MB | 2000 | 273,603,190 | 112,638 |
| SHM 开 | 2000 × 64 KiB = 131 MB | 1805 | **4,169** | 14 |

同样 131 MB 载荷，SHM 开时 UDP 上只有 4 KB —— 数据确实走 iceoryx。差约 6.5 万倍。

**踩坑 G（重要）**：那次 `got=1805`（丢 195 条）最初被我归因为"chunk 池打空导致 RELIABLE 也救不回来"。
后续重复实验证明：**当时的 roudi mempool 只有 2000 个 131072 档 chunk，而 burst 正好 2000 条**，
所以这个观察本身成立；但**不能用它推导"RELIABLE 无效"**，因为归因是资源上限而非可靠性语义。
真正可靠的结论见第 9 节。

---

## 6. P5 · 长稳（64 KiB / 10 Hz / 20 min × 4 轮）

| 轮次 | got / sent | p50 | p95 | p99 | RSS 首→末 |
| --- | --- | --- | --- | --- | --- |
| 单独跑 | 11960/11960 | 1129 µs | 2059 µs | 2135 µs | 9656→9936 KB |
| 重复 #1 | 11960/11960 | 1111 | 2026 | 2127 | 9628→9908 |
| 重复 #2 | 11960/11960 | 1134 | 2058 | 2111 | 9548→9992 |
| 重复 #3 | 11960/11960 | 1108 | 2064 | 2116 | 9612→10056 |

时延分位数极差 ≤ 2.3%，**高度可复现**；`/dev/shm` 段 4 轮恒为 2,454,760,776 字节。

**策略调整**：单轮时我判断 RSS 台阶是"iceoryx 一次性扩容"，重复 3 轮后该解释**站不住**——
每轮都单调增长且峰值总在采样末尾、从未出现平台期。结论只能写成：
**20 分钟内以约 14–22 KB/min 单调增长，是否长期泄漏需 ≥2 小时对照才能定性。**

**踩坑 H**：采样循环写在前台 pub 之后，被阻塞 20 分钟拿不到任何 RSS 样本。改为与 pub 并行采样。

**踩坑 I**：`SUB_RC=$?` 写在 `kill "$SAMPLER"; wait "$SAMPLER"` 之后，取到的是采样器被 SIGTERM 的 143，
不是被测进程退出码——4 轮全部报 143 却收发计数精确相等，由此发现。

---

## 7. P6 · 基准有效性危机（本轮最重要的转折）

### 7.1 现象

Depth 矩阵输出里出现 `pub matched=0`，但 `sub matched=1`；`dds_write` 全部返回成功。
这意味着：**接收数字是在没有建立匹配关系的情况下产生的**。

### 7.2 据此撤回的结论

- ❌ `depth=5` 导致接收端反压
- ❌ `depth=5` 导致数据被覆盖或丢弃
- ❌ `depth=5` 的吞吐比 `depth=20/200` 差
- ❌ 基于 `got` 计算的延迟与丢包率有效

`dds_write` 成功只代表写调用成功；BEST_EFFORT 下更不代表交付。

### 7.3 四个独立根因（逐一定位，分别修复）

| 根因 | 表现 | 定位方式 | 修复 |
| --- | --- | --- | --- |
| 泄漏的重复实例污染同一 Domain | 同份代码时好时坏，`pub matched=0` | 清理残留进程后同一份代码 `matched=1`；`interop_native` 始终正常 | 每轮强制清场 + 精确进程名检查；Match Gate |
| 启动顺序死锁 | 六轮 T 用例全 `INVALID` | 订阅端等发布端、脚本等订阅端 `ready`，闭环 | `READY` 改为"端点已创建、开始监听发现"，与 `MATCHED` 分离 |
| Idle 按迭代次数判定 | `MATCHED` 后立即空转退出，`got=0` 却照常 `DONE` | `empty_wakeups` 达 6.28e6；`seconds` 为负 | 改用 `steady_clock` 墙钟；`dds_set_status_mask(DATA_AVAILABLE)` 止住空转 |
| printf 参数错位 | `matched` 字段变成 787785144 | 格式串被拆成多个**参数**而非字面量拼接 | 合并为单一格式串并逐项对齐 |

### 7.4 修复后的接收状态机

```
INIT → READY → DISCOVERING → MATCHED → RECEIVING → IDLE → DONE
                                        ↘ NO_DATA（匹配成功但 0 样本）
                                        ↘ INVALID（匹配超时）
```

规则：
- `READY` 不代表匹配；匹配单独由 `MATCHED` 把关
- **首包窗口**（默认 10 s）与**接收后空闲窗口**（默认 2 s）分离
- Idle 计时**只在收到第一个样本后启动**，空唤醒不计时
- 所有超时用 `steady_clock`，`wait_ms` 取 `min(剩余首包窗口, 剩余空闲窗口)`
- 取数取空为止，避免把 RHC 中未取走的数据误判为空闲
- `MATCHED` 但 `got=0` → `NO_DATA`，**不得**与正常 `DONE` 混淆

### 7.5 Match Gate（硬门槛）

`matched < 1` 时直接输出 `valid:false` 并返回退出码 3，**不产出任何性能数字**。
这让"跑通了但没匹配"的运行在流程上就不可能被当成数据。

---

## 8. P7 · 接收退出语义验证（T4 × 5）

用例：发 100 条 → 停 1 秒 → 再发 100 条；首包窗口 10 s、空闲窗口 2 s。

| 轮次 | got | missing | end_reason | idle_duration_ms | last_rx_ts_ms | verdict |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 200 | 0 | idle_timeout | 2000 | 1099 | 退出语义 PASS |
| 2 | 197 | 3 | idle_timeout | 2000 | 1299 | 退出语义 PASS / 数据差 3 条 |
| 3 | 200 | 0 | idle_timeout | 2000 | 1099 | 退出语义 PASS |
| 4 | 200 | 0 | idle_timeout | 2000 | 1099 | 退出语义 PASS |
| 5 | 200 | 0 | idle_timeout | 2000 | 1099 | 退出语义 PASS |

关键点：**中间 1 秒暂停从未触发提前退出**（5/5），结束都由真实空闲窗口决定，
`last_rx_ts_ms` 落在第二批数据到达之后。这才是"改的是时间语义，不是把循环变慢"的证明。

---

## 9. P8 · 有效性修复后的实测结论（目前唯一可用的性能结论）

**RELIABLE 不等于应用层送达。** 在 100 条 4 KiB 突发、读端 `KEEP_LAST(depth)` 的条件下：

| 条件 | got / 200 | missing |
| --- | --- | --- |
| depth=5，突发 100×2（无限速） | 105–178 | 最多 95 |
| depth=20，限速 1000 Hz，100+1s+100 | 200（4/5 轮） | 0–3 |

机理：RELIABLE 保证样本到达**读端缓存**，但 `KEEP_LAST(depth)` 会让**未被应用取走**的旧样本被覆盖。
队列深度直接决定"应用层最多能捞回多少条"，与可靠性语义正交。

因此部署时的参数约束是：

```
读端 KEEP_LAST 深度  >=  突发窗口内未被及时取走的样本数
iceoryx chunk 档位数 >=  历史深度 + 突发在途量
```

---

## 10. 踩坑清单（按时序）

| # | 坑 | 后果 | 识别方式 |
| --- | --- | --- | --- |
| A | 用 `strings` 判断 `rt/` 前缀 | 否定了真实存在的前缀 | `strings` 默认最短长度 4，`rt` 被丢弃 |
| B | type name 缺 `dds_::` / 尾下划线 | 静默不匹配 | 抓 SEDP 公告比对 |
| C | idlc 写入绝对路径 | 生成物不可跨目录复现 | `--check` 跨目录比对失败 |
| D | 混淆 `@hashid` 与 RIHS01 | 错误的哈希方案 | 两套机制用途不同 |
| E | 脚本未 source ROS | 7 组全 rc=127，误判"全不匹配" | `ldd` 检查缺失动态库 |
| F | QoS 词填错槽位 | 整整一维变量没被测到 | 回显实际施加值比对 |
| G | 把 chunk 池上限当成可靠性语义 | 错误结论"RELIABLE 无效" | 复核 mempool 档位数与突发量 |
| H | 采样循环被前台 pub 阻塞 | 20 分钟零样本 | 日志只有 start |
| I | `$?` 取值时机错误 | 退出码全 143 | 与收发计数矛盾 |
| J | 泄漏的重复实例 | 匹配全靠运气 | 清理后同码复现 |
| K | `READY` 同时表达"已创建"与"已匹配" | 启动死锁 | 六轮全 INVALID |
| L | Idle 按迭代次数 | 秒级内空转退出 | `empty_wakeups` 6.28e6 |
| M | printf 格式串被拆成参数 | 字段错位（`matched`=787785144） | 字段值明显非物理 |
| N | `pkill -f dds_bench` 匹配到自身命令行 | ssh 会话自杀（exit 255） | 命令无输出即中断 |
| O | 把延迟放在 `take()` 之后 | 造不出 RHC 积压，测不到反压 | 与"取数频率不足"是两件事 |

## 11. 策略调整清单（按时序）

| 阶段 | 原策略 | 调整后 | 触发原因 |
| --- | --- | --- | --- |
| 定位互通 | 读源码 / `strings` 推断 | 抓 SEDP 端点公告 | 坑 A 得出错误结论 |
| 验证互通 | 只看进程存活 | 断言 `matched` + 逐字段值 | 不匹配时无任何报错 |
| 类型定义 | 手写双份 IDL | `.msg` 单一来源 + 确定性生成 | 两份定义漂移 |
| SHM 验证 | 看 XML 里 `Enable=true` | 用 lo 上 UDP 字节反证 | 配置被解析 ≠ 被使用 |
| 长稳结论 | 单轮定性"一次性扩容" | 多轮重复 + 首尾窗口 | 单轮无法区分泄漏与扩容 |
| 性能结论 | 直接跑 Depth 矩阵 | 先过 Match Gate 再谈性能 | 坑 J/K 导致整批数据作废 |
| 超时判定 | 迭代次数 | `steady_clock` 墙钟 + 双窗口 | 坑 L |
| 数据完整性 | `sent - got` | 序列号统计（唯一/重复/乱序/区间缺失） | 在途与覆盖无法区分 |
| 变量分离 | Depth 与消费者能力混跑 | 先 Depth 单变量，再拆 writer/reader，消费能力单独一组 | 变量耦合 |
| 文档 | 每轮一份报告文件 | 单一按时序的本文档 | 记录散落、无法回溯 |

## 12. 容器化验收状态（Humble + Docker）

主线镜像为仓库根 `docker/Dockerfile`（`ros:humble`）。已核实（硬证据，非推测）：

| 项 | 结论 | 证据 |
| --- | --- | --- |
| `ros-humble-cyclonedds` | 一个 deb 同时含 `libddsc` + `idlc` + 头文件 | `dpkg-deb -c` |
| iceoryx 为硬运行时依赖 | `libddsc.so` 的 `NEEDED` 含 `libiceoryx_binding_c.so` | `readelf -d` |
| amd64 / arm64 | 两个架构均可 | 两 arch 的 deb 一致 |
| 源码构建/测试/生成物一致性 | 已按 Dockerfile 的 RUN 逐条原生复算通过 | configure/build rc=0、`ctest 2/2`、两个 `--check` |
| **镜像 `FROM` + `apt` 层** | **未验证** | 编写环境无 docker/podman |
| **Humble 容器内 SHM / 性能** | **未验证** | 需到有容器运行时的机器复跑 |

头文件布局差异（写死会踩）：humble 在 `include/dds/dds.h`，jazzy 在 `include/CycloneDDS/dds/dds.h`。
本仓用 `find_package(CycloneDDS)` + `CycloneDDS::ddsc` 由 target 自带 include 路径，两种布局都能解析，
**不要手写 `-I`**。

## 13. 未完成 / 待办

| 项 | 说明 |
| --- | --- |
| ≥2 小时长稳 | 定性 RSS 单调增长是泄漏还是一次性扩容 |
| writer-only / reader-only Depth 拆分 | 分清发送端缓存与接收端缓存各自的作用 |
| 消费能力矩阵（C 组） | 需在有效匹配前提下重跑 |
| Humble 容器内复跑全部结论 | 当前所有性能数字来自原生环境 |
| arm64（Jetson）镜像与实机 | 需真机 |
| 符号面红线校验 | 增加静态检查：DDS 载荷不得出现速度/几何字段（守住 OS0） |
