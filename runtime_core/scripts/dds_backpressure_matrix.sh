#!/usr/bin/env bash
# 反压矩阵：把 History Depth、Reliability、消费者能力三个变量拆开，
# 观察「缓存过大 → 积压」与「缓存过小 → 写端阻塞」两端的实际形态。
#
# A 组：Depth × Reliability（消费者固定慢速）
# C 组：Depth/Reliability 固定，只调消费者处理耗时
#
# 用法：bash scripts/dds_backpressure_matrix.sh [--count 200] [--size 65536] [--bench path]
set -uo pipefail

COUNT=200
SIZE=65536
BENCH=build/dds/dds_bench
while [ $# -gt 0 ]; do
    case "$1" in
        --count) COUNT=$2; shift 2 ;;
        --size) SIZE=$2; shift 2 ;;
        --bench) BENCH=$2; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done
[ -x "$BENCH" ] || { echo "找不到 $BENCH" >&2; exit 2; }

if [ -z "${ROS_DISTRO:-}" ]; then
    for s in /opt/ros/*/setup.bash; do
        if [ -f "$s" ]; then set +u; . "$s" >/dev/null 2>&1; set -u; break; fi
    done
fi
if ldd "$BENCH" 2>/dev/null | grep -q 'not found'; then
    for d in /opt/ros/*/lib/*/ /opt/ros/*/lib/; do
        [ -d "$d" ] && LD_LIBRARY_PATH="$d${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    done
    export LD_LIBRARY_PATH
fi

idx=0
json_of() { sed -n 's/.*\({"role": "'"$1"'".*}\)$/\1/p' "$2" | tail -1; }

run_case() {  # group depth reliability consumer_delay_ms
    local group=$1 depth=$2 rel=$3 delay=$4
    idx=$((idx + 1))
    local topic="rt/bp/$group$idx"
    local sublog pubout subout
    sublog=$(mktemp)

    timeout 300 "$BENCH" --role sub --count "$COUNT" --size "$SIZE" --topic "$topic" \
        --depth "$depth" --reliability "$rel" --consumer-delay-ms "$delay" \
        --await-match-ms 8000 --idle-ms 15000 >"$sublog" 2>&1 &
    local subpid=$!
    sleep 3

    pubout=$(timeout 300 "$BENCH" --role pub --count "$COUNT" --size "$SIZE" --topic "$topic" \
        --depth "$depth" --reliability "$rel" --await-match-ms 8000 2>/dev/null | tail -1)
    wait "$subpid" 2>/dev/null
    subout=$(json_of sub "$sublog")
    rm -f "$sublog"

    pubout=${pubout:-{\}}
    subout=${subout:-{\}}

    python3 - "$group" "$depth" "$rel" "$delay" "$pubout" "$subout" <<'PY'
import json, sys
group, depth, rel, delay, pubraw, subraw = sys.argv[1:7]
try:
    pub = json.loads(pubraw)
except Exception:
    pub = {}
try:
    sub = json.loads(subraw)
except Exception:
    sub = {}
w = pub.get("write_latency_us", {})
e = sub.get("end_to_end_us", {})
print(json.dumps({
    "group": group, "depth": int(depth), "reliability": rel,
    "consumer_delay_ms": int(delay), "size": pub.get("size"),
    "attempted": pub.get("attempted"), "write_ok": pub.get("write_ok"),
    "write_fail": pub.get("write_fail"), "write_fail_rc": pub.get("write_fail_rc"),
    "write_p50_us": w.get("p50"), "write_p95_us": w.get("p95"),
    "write_p99_us": w.get("p99"), "write_max_us": w.get("max"),
    "write_blocked_total_ms": pub.get("write_blocked_total_ms"),
    "pub_seconds": pub.get("seconds"),
    "got": sub.get("got"), "take_batches": sub.get("take_batches"),
    "max_batch": sub.get("max_batch"),
    "e2e_p50_us": e.get("p50"), "e2e_p95_us": e.get("p95"), "e2e_p99_us": e.get("p99"),
    "e2e_max_us": e.get("max"),
}, ensure_ascii=False))
PY
}

echo "=== A 组：Depth × Reliability（消费者固定 5 ms/批）==="
run_case A 1   best_effort 5
run_case A 5   best_effort 5
run_case A 20  best_effort 5
run_case A 200 best_effort 5
run_case A 1   reliable    5
run_case A 5   reliable    5
run_case A 20  reliable    5
run_case A 200 reliable    5

echo "=== C 组：消费者能力（Depth 5 / RELIABLE）==="
for d in 0 1 5 20 100; do run_case C 5 reliable "$d"; done
