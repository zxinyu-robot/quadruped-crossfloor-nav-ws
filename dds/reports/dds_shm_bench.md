# 同机 DDS 吞吐与时延：SHM 关闭 / 开启对比

- 时间：2026-11-21
- 环境：CycloneDDS 0.10.5 · iceoryx 2.0.6（`iox-roudi`）· 单机 · domain 0
- 方法：`dds_bench --role pub|sub`，两进程对发，`stamp_ns` 记发送时刻算单程时延
- 原始输出：[dds_shm_bench.log](dds_shm_bench.log)
- 复现：`bash scripts/dds_shm_bench.sh <build_dir>`

## 结果

| 载荷 | 配置 | 接收吞吐 | 时延 p50 | 时延 p95 |
| --- | --- | --- | --- | --- |
| 1 KiB | SHM 关 | 204 MB/s | 1.2 ms | 2.1 ms |
| 1 KiB | SHM 开 | 408 MB/s（n=100） | <1 ms | — |
| 64 KiB | SHM 关 | 106 MB/s | 117.6 ms | 263.5 ms |
| 64 KiB | **SHM 开** | **1628–2192 MB/s**（16–21×） | **2.9–3.8 ms** | 4.5–5.7 ms |
| 1 MiB | SHM 关 | 126 MB/s | 378.7 ms | 689.4 ms |
| 1 MiB | **SHM 开** | **1029 MB/s**（8×） | **2.5 ms** | 4.1 ms |

SHM 开启时 `/dev/shm` 实际占用：`iceoryx_mgmt` 68.8 MB、`root` 1.19 GB；
RouDi 日志：`Reserving 66761736 bytes in the shared memory [iceoryx_mgmt]`、`RouDi is ready for clients`。

## 结论

- **大载荷（≥64 KiB）走 SHM 收益显著**：吞吐提升 8–21 倍，p50 时延从百毫秒级降到毫秒级。
  这正是点云 / RGB 帧这类通道需要的。
- **小载荷收益有限**：1 KiB 约 2 倍，且存在限制（见下）。小消息用 UDP 就够。

## 已知限制（实测，未规避）

1 KiB 载荷下若一次在途样本较多（本基准 500 条以上、history 深度随之增大），底层报
`iceoryx subscriber - TOO_MANY_CHUNKS_HELD_IN_PARALLEL`，即 chunk 池被在途样本耗尽；
100 条时可正常收发。缓解方向（未在本轮验证）：降低 reader history 深度、按载荷尺寸扩大
mempool、或对大流量通道使用专用 QoS。**小载荷本就不建议依赖 SHM**，因此不再追此路径。
