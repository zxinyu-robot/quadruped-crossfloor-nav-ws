#!/usr/bin/env bash
# 一条命令跑 DDS 最小实验沙箱（不装导航栈）。宿主机需要 docker。
#
#   ./docker/run_dds_sandbox.sh selftest
#   ./docker/run_dds_sandbox.sh qos-matrix
#   ./docker/run_dds_sandbox.sh shm-proof --size 65536 --count 2000
#   ./docker/run_dds_sandbox.sh soak --size 65536 --rate 10 --duration 1200
#
# --shm-size 不是可选项：实测 iceoryx 段要占 2.4 GB 量级，而 Docker 默认 /dev/shm
# 只有 64 MB。这里同时给 --ipc=host 与 --shm-size，保证两种隔离方式下都够用；
# 主线容器（platform/go2 分支 docker/run_container.sh）用的是 --ipc host，共享宿主机 /dev/shm。
set -euo pipefail
IMAGE=${IMAGE:-runtime-core-dds:humble}
ROOT=$(cd "$(dirname "$0")/.." && pwd)

docker build -f "$ROOT/docker/Dockerfile.dds" -t "$IMAGE" "$ROOT"

exec docker run --rm -it \
    --ipc=host \
    --shm-size=4g \
    --network=host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "$IMAGE" "$@"
