#!/usr/bin/env bash
# History Depth 单变量对照。
#
# 固定：Topic / Type / Domain / Reliability / Durability / 数据量 / 载荷 / 收发逻辑 / 实例数 1:1
# 唯一变化：History Depth（writer 与 reader 同步），先取「整体效果」。
#
# 有效性门槛（Match Gate）由被测程序强制：任一端 matched<1 直接 valid=false 且退出码 3，
# 本脚本把这类运行记为 INVALID 并不计入统计。
#
# 用法：bash scripts/dds_depth_sweep.sh [--count 200] [--size 65536] [--reliability best_effort]
set -uo pipefail

COUNT=200
SIZE=65536
REL=best_effort
DUR=volatile
BENCH=build/dds/dds_bench
DEPTHS="1 5 20 200"
SETTLE_MS=2000
KEEP_LOGS=0
while [ $# -gt 0 ]; do
    case "$1" in
        --count) COUNT=$2; shift 2 ;;
        --size) SIZE=$2; shift 2 ;;
        --reliability) REL=$2; shift 2 ;;
        --depths) DEPTHS=$2; shift 2 ;;
        --settle-ms) SETTLE_MS=$2; shift 2 ;;
        --keep-logs) KEEP_LOGS=1; shift ;;
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

# 混杂变量清理：残留端点会让同 Domain 上的匹配关系失真
assert_clean() {
    local n
    # 必须用精确进程名：-f 会把脚本自身/ssh 命令行里的 "dds_bench" 也算进去
    n=$(pgrep -xc dds_bench 2>/dev/null | head -1)
    n=${n:-0}
    if [ "${n:-0}" -gt 0 ]; then
        echo "[INVALID] 仍有 $n 个 dds_bench 进程残留，拒绝开始下一轮" >&2
        pgrep -ax dds_bench >&2
        return 1
    fi
    return 0
}

echo "run_id,depth,pub_matched,sub_matched,sent,got,write_fail,loss_rate,elapsed_ms,p50_ms,p95_ms,p99_ms,max_ms,valid"

idx=0
for DEPTH in $DEPTHS; do
    idx=$((idx + 1))
    RUN="D$idx"
    assert_clean || exit 4
    TOPIC="rt/depth/$RUN"
    if [ "$KEEP_LOGS" = "1" ]; then
        SUBLOG="/tmp/sweep_${RUN}_sub.log"
        PUBLOG="/tmp/sweep_${RUN}_pub.log"
        : >"$SUBLOG"; : >"$PUBLOG"
    else
        SUBLOG=$(mktemp)
        PUBLOG=$(mktemp)
    fi

    T0=$(date +%s%N)
    timeout 300 "$BENCH" --role sub --count "$COUNT" --size "$SIZE" --topic "$TOPIC" \
        --depth "$DEPTH" --reliability "$REL" --durability "$DUR" \
        --await-match-ms 15000 --idle-ms 60000 >"$SUBLOG" 2>&1 &
    SUBPID=$!

    # 只等 READY（端点已创建、开始监听发现），不等 MATCHED。
    # 等 MATCHED 会让「订阅端等发布端」与「脚本等订阅端」形成闭环死锁。
    ready=0
    for _ in $(seq 1 300); do
        if grep -q '"stage": "READY"' "$SUBLOG" 2>/dev/null; then ready=1; break; fi
        if ! kill -0 "$SUBPID" 2>/dev/null; then break; fi
        sleep 0.1
    done

    if [ "$ready" -eq 0 ]; then
        echo "[INVALID] $RUN 订阅端未进入 READY" >&2
        kill "$SUBPID" 2>/dev/null; wait "$SUBPID" 2>/dev/null
        rm -f "$SUBLOG" "$PUBLOG"
        echo "$RUN,$DEPTH,0,0,0,0,0,0.0000,0,,,,,false"
        continue
    fi

    # 发现静默期：端点 READY 只代表本端已创建，不代表对端能在短时间内发现它。
    # 实测对端在 READY 后 ~0.1 s 启动时，15 s 预算内两侧互不发现；2 s 后启动则稳定匹配。
    sleep "$(awk -v ms="$SETTLE_MS" 'BEGIN{printf "%.3f", ms/1000}')"

    # 两端此后并发等待匹配，各自靠 Match Gate 判定
    timeout 300 "$BENCH" --role pub --count "$COUNT" --size "$SIZE" --topic "$TOPIC" \
        --depth "$DEPTH" --reliability "$REL" --durability "$DUR" \
        --await-match-ms 15000 >"$PUBLOG" 2>&1 &
    PUBPID=$!
    wait "$PUBPID" 2>/dev/null
    PUBRC=$?
    wait "$SUBPID" 2>/dev/null
    SUBRC=$?
    T1=$(date +%s%N)
    ELAPSED_MS=$(( (T1 - T0) / 1000000 ))

    python3 - "$RUN" "$DEPTH" "$COUNT" "$ELAPSED_MS" "$PUBRC" "$SUBRC" "$PUBLOG" "$SUBLOG" <<'PY'
import json, sys
run, depth, count, elapsed, pubrc, subrc, publog, sublog = sys.argv[1:9]
def lastjson(path):
    out = None
    for ln in open(path):
        ln = ln.strip()
        if ln.startswith("{") and ln.endswith("}"):
            try:
                out = json.loads(ln)
            except Exception:
                pass
    return out or {}
pub, sub = lastjson(publog), lastjson(sublog)
w = pub.get("write_latency_us") or {}
e = sub.get("end_to_end_us") or {}
sent = pub.get("attempted", 0)
got = sub.get("got", 0)
missing = sub.get("missing_in_range", 0)
maxseq = sub.get("max_seq", -1)
denom = (maxseq + 1) if isinstance(maxseq, int) and maxseq >= 0 else (sent or 1)
loss = missing / denom if denom else 0.0
valid = bool(pub.get("valid")) and bool(sub.get("valid")) and pub.get("matched", 0) >= 1 \
        and sub.get("matched", 0) >= 1 and int(pubrc) == 0
def ms(v):
    return "" if v is None else f"{v/1000.0:.3f}"
print(",".join(str(x) for x in [
    run, depth, pub.get("matched", 0), sub.get("matched", 0), sent, got,
    pub.get("write_fail", 0), f"{loss:.4f}", elapsed,
    ms(w.get("p50")), ms(w.get("p95")), ms(w.get("p99")), ms(w.get("max")),
    "true" if valid else "false",
]))
if not valid:
    sys.stderr.write(f"[INVALID] {run} pubrc={pubrc} subrc={subrc} "
                     f"pub={json.dumps(pub, ensure_ascii=False)[:160]} "
                     f"sub={json.dumps(sub, ensure_ascii=False)[:160]}\n")
PY
    if [ "$KEEP_LOGS" != "1" ]; then rm -f "$SUBLOG" "$PUBLOG"; fi
done
