#!/usr/bin/env bash
# 容器入口：按第一个参数选择要跑的实验。
set -euo pipefail
set +u
. /opt/ros/humble/setup.bash
set -u

cd /opt/runtime_core
cmd="${1:-selftest}"; shift || true

BENCH=build/dds/dds_bench

case "$cmd" in
  selftest)
    ctest --test-dir build --output-on-failure
    python3 scripts/gen_contract.py --check
    python3 scripts/gen_dds_types.py --check
    ;;
  interop)
    python3 scripts/dds_interop_check.py --native build/dds/interop_native "$@"
    ;;
  qos-matrix)
    bash scripts/dds_qos_matrix.sh "$BENCH"
    ;;
  shm-proof)
    bash scripts/dds_shm_proof.sh --bench "$BENCH" "$@"
    ;;
  soak)
    bash scripts/dds_soak.sh --bench "$BENCH" "$@"
    ;;
  shell)
    exec bash
    ;;
  *)
    echo "未知子命令: $cmd（可选 selftest|interop|qos-matrix|shm-proof|soak|shell）" >&2
    exit 2
    ;;
esac
