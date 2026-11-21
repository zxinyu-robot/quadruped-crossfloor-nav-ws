# 模型权重配置

| 项 | 值 |
|---|---|
| 模型 | Qwen2.5-VL-7B-Instruct |
| 量化格式 | INT4-AWQ（`Qwen/Qwen2.5-VL-7B-Instruct-AWQ`） |
| 加载引擎 | vLLM 0.31.0，`quantization="awq_marlin"` |
| 推理参数 | `max_model_len=4096`、`gpu_memory_utilization=0.85`、`limit_mm_per_prompt={"image":1,"video":0}`、sampling `temperature=0` `max_tokens=160` |
| 实测显存 | 6.67 GiB（RTX 4090） |
| 实测延迟 | ARKitScenes P50 0.42s/帧；Elevator-LIO P50 0.78s/帧 |
| 体积 | ~6.5 GB（safetensors×2） |
| 下载源 | ModelScope `Qwen/Qwen2.5-VL-7B-Instruct-AWQ`（快照目录 `Qwen--Qwen2.5-VL-7B-Instruct-AWQ/snapshots/master`） |
| 存放位置 | 远程机数据盘 `/root/gpufree-data/ms/models/`，脚本引用 `/root/ms/models/`（symlink） |

权重不进 git 仓；部署侧实现 `VLMBackend.observe()` 后按本表加载。Jetson Orin NX 上改用 TensorRT + AWQ，吞吐实测为计划中项。
