# 案例 B：足式机器人跨楼层长航时自主导航

**案例整理与署名：zxinyu**

## 目标

面向动态、非结构化环境中的 Unitree Go2 四足机器人跨楼层目标导航，统一定位状态、任务契约、规划接口和故障恢复语义。

## 系统组成

- Go2 + Livox Mid-360 验证平台；
- ROS 2 Humble 工作空间；
- Super-LIO 定位；
- 全局引导与 SCAN 局部规划；
- 行为树任务编排；
- 任务运行时、空间状态和控制指令适配；
- Go2 转发器与 Sport API 适配层。

开源规划和定位组件在本项目中进行工程集成；任务编排、空间状态运行时和控制指令适配属于本项目的系统集成边界。

## Demo 与验证结果

- 云端仿真中的多楼层任务闭环：11/11 成功；
- Go2 真机已运行在线建图与规划；
- 可通过仓库现有 `docs/media/cross_floor_demo.webp` 查看跨楼层演示；
- Elevator-LIO 数据集回放用于补充验证和回归。

当前状态不等同于所有平台、所有楼层或所有网络条件下的性能保证。跨平台迁移仍需替换硬件转发器并重新验证。

## 相关接口

空间状态契约通过 [ForkSpatial-Runtime-Engine](https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine) 对接；定位失效恢复插件来自 [legged-navigation-runtime](https://github.com/zxinyu-robot/legged-navigation-runtime)。这些仓库各自维护实现与验证记录。
