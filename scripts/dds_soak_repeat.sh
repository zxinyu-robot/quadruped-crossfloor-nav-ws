#!/usr/bin/env bash
# soak 重复测试驱动：同配置连跑 N 轮，逐轮保存日志，便于判断
# 「RSS 台阶」是一次性扩容还是泄漏、时延分位数是否可复现。
#
# 用法：bash scripts/dds_soak_repeat.sh --rounds 3 --duration 1200 --size 65536 --rate 10 --tag 64k
set -uo pipefail

ROUNDS=3
DURATION=1200
SIZE=65536
RATE=10
TAG=run
SHM=on
BENCH=build/dds/dds_bench
while [ $# -gt 0 ]; do
    case "$1" in
        --rounds) ROUNDS=$2; shift 2 ;;
        --duration) DURATION=$2; shift 2 ;;
        --size) SIZE=$2; shift 2 ;;
        --rate) RATE=$2; shift 2 ;;
        --tag) TAG=$2; shift 2 ;;
        --shm) SHM=$2; shift 2 ;;
        --bench) BENCH=$2; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

LOG="dds/reports/soak_${TAG}.log"
: > "$LOG"
echo "{\"event\": \"repeat_plan\", \"rounds\": $ROUNDS, \"duration_s\": $DURATION, \"size\": $SIZE, \"rate_hz\": $RATE, \"tag\": \"$TAG\"}" | tee -a "$LOG"

for r in $(seq 1 "$ROUNDS"); do
    echo "{\"event\": \"round_start\", \"round\": $r}" | tee -a "$LOG"
    bash scripts/dds_soak.sh --bench "$BENCH" --size "$SIZE" --rate "$RATE" \
        --duration "$DURATION" --shm "$SHM" 2>&1 | sed "s/^/{\"round\": $r, \"line\": /; s/$/}/" | tee -a "$LOG"
    # 上一轮结束后留出时间让 iceoryx 段回收，否则下一轮基线不可比
    sleep 20
done
echo "{\"event\": \"repeat_done\", \"rounds\": $ROUNDS}" | tee -a "$LOG"
