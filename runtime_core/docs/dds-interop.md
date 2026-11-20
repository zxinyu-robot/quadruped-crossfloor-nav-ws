# 单机 DDS 链路：Native CycloneDDS ↔ ROS 2 Jazzy 互通

> 环境实测：CycloneDDS **0.10.5** · rmw_cyclonedds_cpp **2.2.4** · ROS 2 **Jazzy** · Ubuntu 24.04
> 状态：**双向互通已验证**（含 CDR 反序列化）；跨机、SHM、大载荷未验证

本文记录 Native DDS（不依赖 `rcl`/`rclcpp`/`rmw`）直接与 ROS 2 导航栈通信的实测结论、
类型生成链路，以及**踩过的坑**。

---

## 1. 实测结论（2026-11-21）

| 项 | 结果 | 证据 |
| --- | --- | --- |
| ROS topic → DDS topic 映射 | **加 `rt/` 前缀** | RTPS 抓包 SEDP 公告：`rt/demo/mission/event` 等 6 个，`rt/` 出现 201 次 |
| DDS type name | `pkg::msg::dds_::Type_` | 抓包：`std_msgs::msg::dds_::String_` |
| Native → ROS | **PASS** | `matched_subscribers=1`；`ros2 topic echo` 收到的字段值完全正确 |
| ROS → Native | **PASS** | `ros2 topic pub` 发 `schema=2/channel=99/payload=[65,66,67]/stamp=12345`，Native 收到 `"ABC"` |
| XTypes 元数据不对称 | **不影响匹配** | idlc descriptor 带 `DDS_TOPIC_XTYPES_METADATA`，rmw 侧无静态 descriptor，仍匹配成功 |
| 显式 XCDR1 声明 | **不必要** | 未做任何 data representation 设置即互通 |
| RIHS01 | 与 DDS 匹配**无关** | 全程未使用 rihs，匹配成功 |

### 踩过的两个坑（都会导致 `matched_subscribers=0`）

1. **topic 名少了 `rt/` 前缀**。`ros2 topic echo /foo` 实际订阅的是 DDS 主题 `rt/foo`。
   用 `strings librmw_cyclonedds_cpp.so | grep "rt/"` 判断前缀是**错的**（`"rt"` 与 `"/"` 是
   两个独立字面量，且 `rt` 只有 2 字符，低于 `strings` 默认最短长度 4）。必须看实际端点公告。
2. **type name 不带 `dds_::` / 尾下划线**。见下节。

> 方法论：任何"匹配不上/看不到"的否定结论，先验证探测工具本身（本次 `builtin` reader 误用
> 就产生了假的"发现失败"结论）。

---

## 2. 一条命令复现（MVP）

```bash
# 1) 构建（含 DDS 类型支持与实验工具）
cmake -S . -B build -DLEGGED_NAV_BUILD_TESTS=ON -DLEGGED_NAV_BUILD_DDS=ON
cmake --build build -j

# 2) 跑双向互通实验（会自动调用 ros2 CLI 作为对端）
source /opt/ros/jazzy/setup.bash
python3 scripts/dds_interop_check.py \
    --native build/dds/interop_native \
    --ws-setup /ws/install/setup.bash     # 有自定义消息时必填
```

输出：`dds/reports/dds_interop_report.md` 与 `dds_interop_report.json`（含环境、逐项结果、期望值）。

### 覆盖的两种载荷

| 类型 | 覆盖点 |
| --- | --- |
| `motionslam_msgs/msg/RuntimeEnvelope` | 基础标量 + `uint8[]` 变长序列 |
| `motionslam_msgs/msg/TrajectoryProbe` | 嵌套 `std_msgs/Header`、嵌套自定义消息数组 `PosePoint[]`、`string`、`float64[]`、`bool` |

每次运行都会断言**字段值**（不止"能连上"）：

| 方向 | 断言内容 |
| --- | --- |
| Native → ROS | `ros2 topic echo --once` 输出含 `schema_version/channel_id`、`proposal_id/frame_id/坐标` |
| ROS → Native | Native 打印的 JSON 含 ROS 端发布的具体值（如 `"payload": "Hi"`、`"kind": 3`） |

### 最近一次结果

```
环境：ROS jazzy · rmw_cyclonedds_cpp · domain 0 · ros-jazzy-cyclonedds 0.10.5
结论：4/4 通过
  native_to_ros  envelope  PASS
  ros_to_native  envelope  PASS
  native_to_ros  probe     PASS
  ros_to_native  probe     PASS
```

## 3. 类型生成链路（A1 路线）

`.msg` 是唯一人工维护的定义，其余全部是**生成物**：

```
pkg/msg/Type.msg                                 作者编辑
  └─ rosidl_adapter ──► dds/rosidl/pkg/msg/Type.idl    导出快照（生成，入库）
        └─ scripts/transform_rosidl_idl.py ──► dds/idl/pkg/msg/Type.idl
              └─ idlc ──► dds/gen/pkg/Type.{c,h}         CycloneDDS 原生类型支持
                    └─ 断言 m_typename == "pkg::msg::dds_::Type_"
```

### 为什么要转换

rosidl 导出的 IDL 是：

```idl
module pkg { module msg { struct Type { ... }; }; };
```

而 ROS 在 DDS 上公告的是 `pkg::msg::dds_::Type_`。直接 `idlc` 编译会生成错误的 `m_typename`，
端点永远匹配不上。转换只做三件结构性改写（不做字符串替换）：

1. 删除 `@verbatim(...)` 注解
2. `struct Type` → `module dds_ { struct Type_ }`（含嵌套引用 `pkg::msg::X` → `pkg::msg::dds_::X_`）
3. `#include` 保留，靠 `-I` 解析

### 可复现性处理

`idlc` 会把**绝对路径**写进文件头注释与 include guard，导致换目录生成字节不同。
生成器做了两处归一化：

- 路径 → `<DDS_ROOT>`
- include guard → `DDSC__<PKG>__<TYPE>_H`

因此 `--check` 可以跨机器、跨目录比对。

---

## 4. 使用

```bash
# 1) 刷新 ROSIDL 导出快照（需要 ROS 环境）
source /opt/ros/jazzy/setup.bash
python3 scripts/gen_dds_types.py --refresh-rosidl --msg motionslam_msgs/msg/RuntimeEnvelope

# 2) 生成 IDL + 类型支持（需要 idlc）
python3 scripts/gen_dds_types.py --msg motionslam_msgs/msg/RuntimeEnvelope

# 3) CI 校验（无 idlc 时自动降级为「转换层 + manifest 哈希」校验）
python3 scripts/gen_dds_types.py --check
```

### 构建与测试

```bash
cmake -S . -B build -DLEGGED_NAV_BUILD_TESTS=ON -DLEGGED_NAV_BUILD_DDS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`LEGGED_NAV_BUILD_DDS=ON` 需要 `libddsc`（`find_package(CycloneDDS)`）。
`dds/tests/test_dds_type.cpp` 断言：

- `m_typename == "motionslam_msgs::msg::dds_::RuntimeEnvelope_"`
- `m_nops == 5`、`m_nkeys == 0`
- 同进程 writer/reader 匹配 + `schema_version/channel_id/payload` 往返一致

### 与 ROS 联调（需 ROS 环境）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
source /ws/install/setup.bash

# ROS → Native：先起 Native subscriber，再 ros2 topic pub
ros2 topic pub --once /runtime/envelope motionslam_msgs/msg/RuntimeEnvelope \
  "{schema_version: 2, channel_id: 99, payload: [65,66,67], stamp_ns: 12345}"
```

---

## 5. 未验证 / 待办

| 项 | 状态 |
| --- | --- |
| 跨机（不同主机）互通 | 未验证 |
| `string` 字段与嵌套消息 | **已验证**（`TrajectoryProbe`：Header + PosePoint[] + string） |
| 大载荷（点云 / RGB 帧） | 未验证；这是引入 SHM 的前提 |
| SHM（Cyclone 0.10 走 iceoryx） | 未验证 |
| `partition` 隔离 | 未验证；首期建议用 `ROS_DOMAIN_ID` |
| `@appendable` / XCDR2 | 未验证；首期固定 `@final` + XCDR1 |
| QoS 组合（BEST_EFFORT / KEEP_LAST 深度） | 只验证了 RELIABLE + KEEP_LAST 10 |

## 6. 版本前置条件

| 项 | 要求 | 说明 |
| --- | --- | --- |
| CycloneDDS | 0.10.5 | `idlc`、`dds_topic_descriptor_t` 带 `type_information`/`type_mapping` |
| QoS Provider API | **不可用** | 0.10.5 无 `dds_create_qos_provider`/`dds_qos_from_profile`，QoS 只能用 `dds_qset_*` 设置 |
| idlc | 随 `ros-jazzy-cyclonedds` | 只在生成阶段需要，运行期不需要 |
