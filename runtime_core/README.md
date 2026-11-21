# runtime_core：端侧任务运行时

> 本目录是 [quadruped-crossfloor-nav-ws](https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws)
> 的一部分。原独立仓库 `Autonomous-Navigation-for-Legged-Robots` 已并入本仓并停止独立维护；
> 许可证随仓库根 [LICENSE](../LICENSE)（MIT）。

面向四足机器人端侧导航运行时，提供从视觉语义观测、任务意图准入、技能编排到执行反馈的闭环。
核心采用 C++20（不依赖 ROS），另有 Python 对照实现；工程分级 **P0**。

相关文档：

- [运行时与导航栈的两层关系](../README.md#端侧运行时叠加runtime_core)
- [单机 DDS 链路实测记录](docs/dds-interop.md)
- [证据表](docs/evidence.md)
- [第三方依赖与许可](THIRD_PARTY.md)

## 演示

原独立仓库的 ARKitScenes 回放视频已随迁移下线，将由本仓重新录制的整体演示替代
（视觉意图 → TaskIntent → 导航栈 → 执行反馈 的完整链路）。

回放脚本仍在 [`scripts/e2e_arkitscenes_awq.py`](scripts/e2e_arkitscenes_awq.py) 与
[`scripts/e2e_elevatorlio_awq.py`](scripts/e2e_elevatorlio_awq.py)，可自行复现；
指标口径见 [docs/evidence.md](docs/evidence.md)。

**演示口径：数据集回放 + Stub 语义观测 + 仿真执行，非真机导航。**

## 目录

- `include/legged_nav/`、`src/*.cpp`：C++ 核心接口与实现（权威实现）
- `src/legged_nav/`：Python 对照实现
- `contracts/runtime_contract.json`：契约常量唯一来源（生成 `contract_gen.hpp` / `contract_gen.py`）
- `dds/`：由 `.msg` 确定性生成的 CycloneDDS 原生类型支持，见 [docs/dds-interop.md](docs/dds-interop.md)
- `tests/`：C++ 综合测试、契约对照语料、Python 单元/端到端测试
- `scripts/`：构建、契约/DDS 代码生成、数据集回放
- `docs/`：证据表、模型权重配置、DDS 互通记录
