# 第三方依赖与并入子树

`main` 只含运行时。Go2 真机链路的第三方依赖（Super-LIO、livox_ros_driver2、unitree_ros2、PCT_planner、SCAN-Planner、legged-terrain-state）、本仓 patch 及许可核查见 [`platform/go2` 分支 THIRD_PARTY.md](https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws/blob/platform/go2/THIRD_PARTY.md)。

## 并入本仓的自研子树

| 目录 | 来源 | commit | 方式 | 许可 |
|------|------|--------|------|------|
| `runtime_core/` | https://github.com/zxinyu-robot/Autonomous-Navigation-for-Legged-Robots.git | `fe5bdb8` | `git subtree --squash` | MIT |
| `fork_spatial/` | https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine.git | `88e75b9` | `git subtree --squash` | MIT（[LICENSE](fork_spatial/LICENSE)） |

同步上游改动：`git subtree pull --prefix=fork_spatial https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine.git main --squash`。

## 构建时拉取

| 组件 | 远程 | commit | 用途 | 许可 |
|------|------|--------|------|------|
| legged-navigation-runtime（LNR） | https://github.com/zxinyu-robot/legged-navigation-runtime.git | `a5a8b2e` | `runtime/task_runtime` 的 `MOTIONSLAM_WITH_LNR=ON` 恢复桥（CMake FetchContent） | MIT |

`runtime_core/` 自身的开发依赖见 [runtime_core/THIRD_PARTY.md](runtime_core/THIRD_PARTY.md)。
