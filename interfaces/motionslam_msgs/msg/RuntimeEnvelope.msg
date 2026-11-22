# 端侧运行时 ⇄ 导航栈 的 DDS 信封（自定义类型，单机 Cyclone DDS + SHM）
# 按三空间约束（OS0），本信封只承载符号量：TaskIntent / ExecutionFeedback / mission event。
# 不得承载几何量或速度指令——执行只发生在度量空间，符号面不直写执行。
# payload 为 UTF-8 JSON 字节，schema 见 runtime_core/contracts/runtime_contract.json；
# 链路实测与踩坑记录见 runtime_core/docs/dds-interop.md。
uint32 schema_version
uint32 channel_id
uint8[] payload
int64 stamp_ns
