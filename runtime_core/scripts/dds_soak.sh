#!/usr/bin/env bash
# 单机长稳（soak）：固定速率持续发送，采样订阅端 RSS 与 /dev/shm 占用，
# 用首/尾窗口对比判断延迟是否随运行时间漂移。
#
# 用法：bash scripts/dds_soak.sh [--size 65536] [--rate 10] [--duration 1200] [--shm on|off]
# 输出：RSS 采样（每 30 s 一条）+ 订阅端最终 JSON（含 head/tail 窗口）。
set -uo pipefail

SIZE=65536
RATE=10
DURATION=1200
SHM=on
BENCH=""
while [ $# -gt 0 ]; do
    case "$1" in
        --size) SIZE=$2; shift 2 ;;
        --rate) RATE=$2; shift 2 ;;
        --duration) DURATION=$2; shift 2 ;;
        --shm) SHM=$2; shift 2 ;;
        --bench) BENCH=$2; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done
BENCH=${BENCH:-build/dds/dds_bench}
[ -x "$BENCH" ] || { echo "找不到可执行文件 $BENCH" >&2; exit 2; }

# ROS 的 setup.bash 与 set -u 不兼容，source 前后临时关闭
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

URI=""
if [ "$SHM" = "on" ]; then
    URI='<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast></General><SharedMemory><Enable>true</Enable></SharedMemory></Domain></CycloneDDS>'
    if ! pgrep -x iox-roudi >/dev/null 2>&1; then
        cfg=/tmp/roudi_config.toml
        cat > "$cfg" <<'TOML'
[general]
version = 1

[[segment]]

[[segment.mempool]]
size = 128
count = 40000

[[segment.mempool]]
size = 1024
count = 40000

[[segment.mempool]]
size = 16384
count = 8000

[[segment.mempool]]
size = 65536
count = 4000

[[segment.mempool]]
size = 131072
count = 2000

[[segment.mempool]]
size = 1048576
count = 800

[[segment.mempool]]
size = 4194304
count = 200
TOML
        iox-roudi --config-file "$cfg" --monitoring-mode off >/tmp/roudi_soak.log 2>&1 &
        sleep 3
    fi
fi

TOPIC="rt/soak/envelope"
SUBLOG=/tmp/soak_sub.log
: > "$SUBLOG"

echo "{\"event\": \"start\", \"size\": $SIZE, \"rate_hz\": $RATE, \"duration_s\": $DURATION, \"shm\": \"$SHM\", \"bench\": \"$BENCH\"}"

CYCLONEDDS_URI="$URI" "$BENCH" --role sub --size "$SIZE" --topic "$TOPIC" \
    --duration-s "$DURATION" --bucket-s 60 --idle-ms 120000 >"$SUBLOG" 2>&1 &
SUBPID=$!
sleep 4

# 采样必须在 pub 运行期间并行进行，否则前 DURATION-4 秒完全没有数据
sample_loop() {
    for i in $(seq 1 $((DURATION / 30))); do
        sleep 30
        rss=$(awk '/^VmRSS:/{print $2}' "/proc/$SUBPID/status" 2>/dev/null | tr -d ' ')
        shm=$(du -b /dev/shm/iceoryx_mgmt /dev/shm/root 2>/dev/null | awk '{s+=$1} END {print s+0}')
        echo "{\"event\": \"sample\", \"t_s\": $((i * 30)), \"sub_rss_kb\": ${rss:-0}, \"shm_bytes\": ${shm:-0}}"
    done
}
sample_loop &
SAMPLER=$!

CYCLONEDDS_URI="$URI" "$BENCH" --role pub --size "$SIZE" --topic "$TOPIC" \
    --rate-hz "$RATE" --duration-s "$((DURATION - 4))" --await-match-ms 8000
PUBRC=$?
echo "{\"event\": \"pub_done\", \"rc\": $PUBRC}"

wait "$SUBPID"
kill "$SAMPLER" 2>/dev/null
wait "$SAMPLER" 2>/dev/null
SUB_RC=$?

echo "{\"event\": \"sub_done\", \"rc\": $SUB_RC}"
tail -1 "$SUBLOG"
