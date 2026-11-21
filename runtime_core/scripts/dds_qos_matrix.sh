#!/usr/bin/env bash
# DDS QoS 兼容性矩阵：按 DDS requested/offered 规则逐组验证。
#
# 每组起一个 reader + 一个 writer，记录 publisher 侧 matched 数，判定是否符合预期。
# 输出：每组一行 JSON，最后打印汇总表；任一组不符预期则退出码非 0。
#
# 用法：bash scripts/dds_qos_matrix.sh <build_dir>/dds_bench
set -uo pipefail

BENCH="${1:-build/dds/dds_bench}"
[ -x "$BENCH" ] || { echo "找不到可执行文件 $BENCH" >&2; exit 2; }

# dds_bench 链接了 libddsc + libiceoryx_binding_c，两者都在 ROS 的 lib 目录里。
# 注意：ROS 的 setup.bash 与本脚本的 set -u 不兼容，source 前后要临时关掉。
if [ -z "${ROS_DISTRO:-}" ]; then
    for s in /opt/ros/*/setup.bash; do
        if [ -f "$s" ]; then
            set +u
            . "$s" >/dev/null 2>&1
            set -u
            break
        fi
    done
fi
if ldd "$BENCH" 2>/dev/null | grep -q 'not found'; then
    for d in /opt/ros/*/lib/*/ /opt/ros/*/lib/; do
        [ -d "$d" ] && LD_LIBRARY_PATH="$d${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    done
    export LD_LIBRARY_PATH
fi
if ldd "$BENCH" 2>/dev/null | grep -q 'not found'; then
    echo "动态库缺失，dds_bench 无法启动：" >&2
    ldd "$BENCH" | grep 'not found' >&2
    exit 3
fi

idx=0
fails=0
rows=()

# 等 sub 打印出最终 JSON（含 "got"），最多等 $2 秒
wait_sub_final() {
    local log=$1 timeout=$2 i=0
    while [ $i -lt $((timeout * 10)) ]; do
        if grep -q '"got"' "$log" 2>/dev/null; then return 0; fi
        sleep 0.1
        i=$((i + 1))
    done
    return 1
}

run_pair() {  # writer_rel writer_dur reader_rel reader_dur expect_matched
    local wrel=$1 wdur=$2 rrel=$3 rdur=$4 expect=$5
    idx=$((idx + 1))
    local topic="rt/qos/case${idx}"
    local sublog; sublog=$(mktemp)

    setsid "$BENCH" --role sub --count 50 --size 4096 --topic "$topic" \
        --reliability "$rrel" --durability "$rdur" \
        --depth 5 --await-match-ms 4000 --idle-ms 4000 >"$sublog" 2>&1 &
    local subpid=$!
    sleep 3

    local pubout pubrc
    pubout=$("$BENCH" --role pub --count 50 --size 4096 --topic "$topic" \
        --reliability "$wrel" --durability "$wdur" --await-match-ms 4000 2>&1)
    pubrc=$?
    if [ "$pubrc" -ne 0 ] || [[ "$pubout" != *'"matched"'* ]]; then
        echo "ERROR: case $idx 的 writer 启动失败 rc=$pubrc: $pubout" >&2
    fi
    wait_sub_final "$sublog" 20
    kill "$subpid" 2>/dev/null
    wait "$subpid" 2>/dev/null

    local subout; subout=$(grep '"got"' "$sublog" | tail -1)
    rm -f "$sublog"

    local matched got matched_bool verdict applied
    matched=$(sed -n 's/.*"matched": *\([0-9]*\).*/\1/p' <<<"$pubout" | head -1)
    got=$(sed -n 's/.*"got": *\([0-9]*\).*/\1/p' <<<"$subout" | head -1)
    matched=${matched:-0}; got=${got:-0}
    # 以被测程序回显的 QoS 为准，脚本传参写错会在这里暴露出来
    applied=$(sed -n 's/.*"reliability": "\([a-z_]*\)".*"durability": "\([a-z_]*\)".*/\1|\2/p' <<<"$subout" | tail -1)
    if [ -n "$applied" ] && [ "$applied" != "$rrel|$rdur" ]; then
        echo "ERROR: case $idx reader QoS 期望 $rrel|$rdur 实际 $applied" >&2
        fails=$((fails + 1))
    fi
    matched_bool=False
    [ "$matched" -gt 0 ] && matched_bool=True

    if [ "$matched_bool" = "$expect" ]; then verdict=PASS; else verdict=FAIL; fails=$((fails + 1)); fi

    rows+=("$(printf '{"case": %d, "writer": "%s/%s", "reader": "%s/%s", "expect_matched": %s, "matched": %s, "got": %s, "verdict": "%s"}' \
        "$idx" "$wrel" "$wdur" "$rrel" "$rdur" "$expect" "$matched" "$got" "$verdict")")
}

run_pair reliable        volatile reliable        volatile        True
run_pair reliable        volatile best_effort     volatile        True
run_pair best_effort     volatile reliable        volatile        False
run_pair best_effort     volatile best_effort     volatile        True
run_pair reliable        transient_local reliable     volatile        True
run_pair reliable        volatile        reliable     transient_local False
run_pair reliable        transient_local reliable     transient_local True

echo "=== QoS 兼容性矩阵（DDS requested/offered）==="
printf '%s\n' "${rows[@]}"

if [ "$fails" -ne 0 ]; then
    echo "结论：$fails/$idx 组与预期不符（FAIL）"
    exit 1
fi
echo "结论：$idx/$idx 组符合 DDS requested/offered 预期（PASS）"
