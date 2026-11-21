#!/usr/bin/env bash
# 同机 DDS 吞吐/时延对比：SHM 关闭 vs 开启（Cyclone DDS + iceoryx）。
#
# 用法：bash scripts/dds_shm_bench.sh [build_dir]
# 输出：每个用例一行 JSON（pub/sub 各一行），可直接粘进报告。
set -euo pipefail

BENCH_DIR="${1:-build/dds}"
BENCH="${BENCH_DIR}/dds_bench"
[ -x "$BENCH" ] || { echo "找不到 $BENCH（先 cmake --build）" >&2; exit 2; }

# SHM 需要 iceoryx 守护进程
SHM_URI='<CycloneDDS><Domain><General><AllowMulticast>spdp</AllowMulticast></General><SharedMemory><Enable>true</Enable></SharedMemory></Domain></CycloneDDS>'

start_roudi() {
    if pgrep -x iox-roudi >/dev/null 2>&1; then return 0; fi
    # 池容量按本基准的最大在途样本数放大，避免 TOO_MANY_CHUNKS_HELD_IN_PARALLEL
    local cfg=/tmp/roudi_config.toml
    cat > "$cfg" <<'TOML'
[general]
version = 1

[[segment]]

[[segment.mempool]]
size = 128
count = 20000

[[segment.mempool]]
size = 1024
count = 20000

[[segment.mempool]]
size = 16384
count = 4000

[[segment.mempool]]
size = 65536
count = 2000

[[segment.mempool]]
size = 131072
count = 1000

[[segment.mempool]]
size = 1048576
count = 400

[[segment.mempool]]
size = 4194304
count = 100
TOML
    iox-roudi --config-file "$cfg" --monitoring-mode off >/tmp/roudi.log 2>&1 &
    sleep 3
}

run_case() {  # label size count uri
    local label=$1 size=$2 count=$3 uri=$4
    local sublog
    sublog=$(mktemp)
    ( CYCLONEDDS_URI="$uri" "$BENCH" --role sub --count "$count" --size "$size" \
        --idle-ms 20000 >"$sublog" 2>&1 & )
    sleep 4   # 等发现完成（SPDP 周期）
    printf '{"case": "%s", ' "$label"
    CYCLONEDDS_URI="$uri" "$BENCH" --role pub --count "$count" --size "$size" 2>/dev/null | tr -d '\n'
    printf ' }\n'
    sleep 3
    printf '{"case": "%s", ' "$label"
    tail -1 "$sublog" | tr -d '\n'
    printf ' }\n'
    rm -f "$sublog"
}

echo "=== SHM OFF ==="
for spec in "1KiB 1024 1000" "64KiB 65536 500" "1MiB 1048576 100"; do
    set -- $spec
    run_case "shm_off_$1" "$2" "$3" ""
done

echo "=== SHM ON (iceoryx) ==="
start_roudi
echo "/dev/shm 段（后看）: $(ls -l /dev/shm/iceoryx_mgmt 2>/dev/null | awk '{print $5}')"
for spec in "1KiB 1024 1000" "64KiB 65536 500" "1MiB 1048576 100"; do
    set -- $spec
    run_case "shm_on_$1" "$2" "$3" "$SHM_URI"
done
echo "/dev/shm 段（后）: $(du -b /dev/shm/iceoryx_mgmt /dev/shm/root 2>/dev/null | tr "\n" " ")"
