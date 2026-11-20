# DDS 互通实验报告

- 时间：2026-11-21T06:30:48+08:00
- 环境：ROS jazzy · rmw_cyclonedds_cpp · domain 0
- CycloneDDS：ii  ros-jazzy-cyclonedds 0.10.5-1noble.20260225.142613 amd64        Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation.
- 结论：**4/4 通过**

| 方向 | 类型 | 结果 | 细节 |
| --- | --- | --- | --- |
| native_to_ros | envelope | PASS | 全部字段匹配 |
| ros_to_native | envelope | PASS | 全部字段匹配 |
| native_to_ros | probe | PASS | 全部字段匹配 |
| ros_to_native | probe | PASS | 全部字段匹配 |
