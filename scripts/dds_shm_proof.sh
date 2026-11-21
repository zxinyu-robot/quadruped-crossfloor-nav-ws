#!/usr/bin/env bash
# 证明 DDS 数据是否**实际**经由共享内存传输。
#
# 只看配置里写没写 Enable=true 是不够的——那只证明「被解析」，不证明「被使用」。
# 判据：同一对 pub/sub，在 SHM 关/开两种配置下抓 lo 上的 UDP 流量。
#   SHM 关：RTPS 数据必须走 UDP，lo 上 UDP 字节应与载荷量同量级
#   SHM 开：数据走 iceoryx，UDP 上只剩发现与握手，字节数断崖式下降
#
# 用法：bash scripts/dds_shm_proof.sh --size 65536 --count 2000 [--bench path]
set -uo pipefail

SIZE=65536
COUNT=2000
BENCH=build/dds/dds_bench
while [ $# -gt 0 ]; do
    case "$1" in
        --size) SIZE=$2; shift 2 ;;
        --count) COUNT=$2; shift 2 ;;
        --bench) BENCH=$2; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done
[ -x "$BENCH" ] || { echo "找不到 $BENCH" >&2; exit 2; }
command -v tcpdump >/dev/null || { echo "需要 tcpdump" >&2; exit 2; }

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

SHM_URI='<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast></General><SharedMemory><Enable>true</Enable></SharedMemory></Domain></CycloneDDS>'

ensure_roudi() {
    pgrep -x iox-roudi >/dev/null && return 0
    cat > /tmp/roudi_proof.toml <<'TOML'
[general]
version = 1

[[segment]]

[[segment.mempool]]
size = 65536
count = 4000

[[segment.mempool]]
size = 1048576
count = 400

[[segment.mempool]]
size = 4194304
count = 100
TOML
    iox-roudi --config-file /tmp/roudi_proof.toml --monitoring-mode off >/tmp/roudi_proof.log 2>&1 &
    sleep 3
}

measure() {  # label uri
    local label=$1 uri=$2
    local topic="rt/proof/$label"
    local pcap="/tmp/proof_$label.pcap" sublog
    sublog=$(mktemp)

    CYCLONEDDS_URI="$uri" "$BENCH" --role sub --count "$COUNT" --size "$SIZE" --topic "$topic" \
        --depth 5 --idle-ms 20000 >"$sublog" 2>&1 &
    local subpid=$!
    sleep 4

    tcpdump -i lo -n -q -w "$pcap" udp >/dev/null 2>&1 &
    local tdpid=$!
    sleep 2

    CYCLONEDDS_URI="$uri" "$BENCH" --role pub --count "$COUNT" --size "$SIZE" --topic "$topic" \
        --await-match-ms 8000 >/dev/null 2>&1

    sleep 3
    kill "$tdpid" 2>/dev/null; wait "$tdpid" 2>/dev/null
    wait "$subpid" 2>/dev/null
    local subout; subout=$(tail -1 "$sublog"); rm -f "$sublog"

    local udp_bytes pkts got
    udp_bytes=$(stat -c %s "$pcap" 2>/dev/null || echo 0)
    pkts=$(tcpdump -r "$pcap" -n 2>/dev/null | wc -l)
    got=$(sed -n 's/.*"got": *\([0-9]*\).*/\1/p' <<<"$subout" | head -1)
    got=${got:-0}

    printf '{"case": "%s", "size": %d, "count": %d, "payload_bytes": %d, "got": %d, "lo_udp_pcap_bytes": %d, "lo_udp_packets": %d}\n' \
        "$label" "$SIZE" "$COUNT" "$((COUNT * SIZE))" "$got" "$udp_bytes" "$pkts"
    rm -f "$pcap"
}

echo "=== SHM 实际生效判据（抓 lo 上 UDP 流量）==="
measure "shm_off" ""
ensure_roudi
measure "shm_on"  "$SHM_URI"
