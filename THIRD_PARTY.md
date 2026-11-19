# 第三方依赖（迁移用）

> 下表记录历史依赖快照，不代表这些版本均可从所列上游获取，也不代表当前仓库通过 Git 子模块追踪它们。

| 组件 | 远程 | 分支 | commit |
|------|------|------|--------|
| Super-LIO | https://github.com/Liansheng-Wang/Super-LIO.git | ros2 | `25ff025ef03fd203511e0232d813cf66dfe77294` |
| livox_ros_driver2 | https://github.com/Livox-SDK/livox_ros_driver2.git | — | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` |
| unitree_ros2 | https://github.com/unitreerobotics/unitree_ros2.git | — | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` |
| PCT_planner | https://github.com/VectorRobotics/PCT_planner.git | main | `0cf4827eb0e3374345b836b74ad6ea7d913b0edf` |
| SCAN-Planner | https://github.com/wuyi2121/SCAN-Planner.git | ros2-community | `d62de0844c526b9a66e4ac33733a9b8ed84943ad` |
| legged-terrain-state (vendored) | https://github.com/zxinyu-robot/legged-terrain-state.git | main | `third_party/legged_terrain_state`（MIT，LICENSE 随附） |

## 许可与版本核查

- **Super-LIO：** 上游 `ros2` 分支的许可证为 GPL-3.0；上表的固定提交截至 2026-11-19 无法从所列上游读取，实际使用版本的许可证仍需核实。
- **SCAN-Planner：** 上游 `ros2-community` 分支的许可证为 Apache-2.0；下文所述 Jetson 本地提交不在上游，需保留并核实实际使用版本及其许可。
- **PCT_planner：** 上表的固定版本 README 声明 GPLv2，`package.xml` 则声明 Apache-2.0，且该版本没有根目录许可证全文。两处声明存在冲突，GPL 版本是否允许后续版本也尚未确认；使用或分发前应向权利人确认，不能仅依据其中一处确定授权。
- **unitree_ros2：** 上表的固定版本为 BSD-3-Clause，保留其版权和许可证声明。
- **livox_ros_driver2：** 上表的固定版本包含多部分许可，包括 MIT 和 Apache-2.0；保留并遵守上游完整许可证及适用的声明。
- **legged-terrain-state：** 本仓库内的副本保留其 [MIT 许可证](third_party/legged_terrain_state/LICENSE)。

第三方项目及其修改不因本仓库的 MIT 许可证而改变许可。涉及 GPL 代码与其他组件的组合或分发时，需根据具体许可版本、集成方式和分发范围评估兼容性；源码可在 GitHub 上访问也属于需要考虑的分发情形。

## 本仓 patch

- `patches/scan-planner-motionslam.patch` — 应用到 SCAN-Planner 后于 `scan_planner_ws` 编译
- 注：SCAN-Planner 的 pin `d62de08` 是 Jetson 本地提交、不在上游仓库；云端引导（`scripts/dev/cloud_bootstrap.sh`）用 `ros2-community` 分支头 + patch（其 `closed_loop_controller.cpp` 一个 hunk 与上游当前实现冲突，可跳过）
- `patches/pct-planner-motionslam.patch` — PCT 可选对照改动

这些补丁包含针对上游文件的修改及上下文，不属于根目录 MIT 授权的范围。SCAN 补丁应按实际上游版本的许可处理；PCT 补丁的许可边界待上述冲突澄清后确认。

## 说明

- 本 GitHub 仓为**脱敏私人备份**，不是狗端唯一 remote。
- 雷达配置请从 `MID360_config.example.json` 复制为 `MID360_config.json` 并填现场 IP。
- 详细切分原则见 `docs/开发参考/私人GitHub备份清单.md`。
