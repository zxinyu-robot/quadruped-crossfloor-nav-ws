# 端侧运行时 ⇄ 导航栈 的 DDS 信封（自定义类型，单机 Cyclone DDS + SHM）
# payload 为 UTF-8 JSON 字节；channel_id 见 docs/架构/运行时DDS链路.md
uint32 schema_version
uint32 channel_id
uint8[] payload
int64 stamp_ns
