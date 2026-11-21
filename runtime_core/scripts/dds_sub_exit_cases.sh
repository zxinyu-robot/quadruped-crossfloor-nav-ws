#!/usr/bin/env bash
# 订阅端退出机制验证（T1–T6）。只验证接收状态机的时间语义，不涉及 Depth/反压结论。
#
# 判据：
#   T1 匹配后延迟 3 s 才发 → 订阅端不得提前退出
#   T2 只发 1 条        → got=1，随后按 idle 窗口结束
#   T3 发 200 条突发     → 全部取空后进入 idle
#   T4 100 条 + 停 1 s + 100 条 → 暂停期间不得误退出（关键用例）
#   T5 匹配但不发        → NO_DATA（首包超时），不得伪装成 DONE
#   T6 发布端中途退出     → 按真实空闲时间结束
set -uo pipefail
BENCH=${BENCH:-build/dds/dds_bench}
SIZE=${SIZE:-4096}
[ -x "$BENCH" ] || { echo "找不到 $BENCH" >&2; exit 2; }
if [ -z "${ROS_DISTRO:-}" ]; then
    for s in /opt/ros/*/setup.bash; do
        if [ -f "$s" ]; then set +u; . "$s" >/dev/null 2>&1; set -u; break; fi
    done
fi

echo "case,expected,got,end_reason,stage,matched,verdict"

run() {  # name subargs pubargs expect got_expr
    local name=$1 subargs=$2 pubargs=$3 expect=$4
    local T="rt/t/$name"
    local S P
    S=$(mktemp); P=$(mktemp)
    # shellcheck disable=SC2086
    timeout 120 "$BENCH" --role sub --count -1 --size "$SIZE" --topic "$T" --depth 5 \
        --reliability best_effort --await-match-ms 15000 $subargs >"$S" 2>&1 &
    local SP=$!
    sleep 2
    # shellcheck disable=SC2086
    timeout 120 "$BENCH" --role pub --size "$SIZE" --topic "$T" --depth 5 \
        --reliability best_effort --await-match-ms 15000 $pubargs >"$P" 2>&1
    wait "$SP" 2>/dev/null
    local line reason stage got matched verdict
    line=$(grep -o '{"role": "sub", "stage": "[A-Z_]*", "valid": [a-z]*.*' "$S" | tail -1)
    reason=$(sed -n 's/.*"end_reason": "\([a-z_]*\)".*/\1/p' <<<"$line")
    stage=$(sed -n 's/.*"stage": "\([A-Z_]*\)".*/\1/p' <<<"$line")
    matched=$(sed -n 's/.*"matched": \([0-9]*\).*/\1/p' <<<"$line")
    got=$(sed -n 's/.*"got": \([0-9]*\).*/\1/p' <<<"$line")
    got=${got:-0}; matched=${matched:-0}; reason=${reason:-none}; stage=${stage:-none}
    verdict=PASS
    case "$expect" in
        no_early_exit) [ "$stage" = "DONE" ] && [ "$got" -ge 1 ] || verdict=FAIL ;;
        got1)          [ "$got" -eq 1 ] || verdict=FAIL ;;
        all)           [ "$got" -eq 200 ] || verdict=FAIL ;;
        no_data)       [ "$stage" = "NO_DATA" ] || verdict=FAIL ;;
        *)             verdict=UNKNOWN ;;
    esac
    echo "$name,$expect,$got,$reason,$stage,$matched,$verdict"
    rm -f "$S" "$P"
}

run T1_delay3s  "--first-packet-ms 10000 --idle-ms 2000 --overall-ms 60000" \
                "--count 200 --delay-after-match-ms 3000"  no_early_exit
run T2_one_msg  "--first-packet-ms 10000 --idle-ms 2000 --overall-ms 60000" \
                "--count 1"  got1
run T3_burst200 "--first-packet-ms 10000 --idle-ms 2000 --overall-ms 60000" \
                "--count 200"  all
run T4_pause1s  "--first-packet-ms 10000 --idle-ms 2000 --overall-ms 60000" \
                "--count 200 --pause-after 100 --pause-ms 1000"  all
run T5_no_send  "--first-packet-ms 3000 --idle-ms 2000 --overall-ms 15000" \
                "--count 0"  no_data
run T6_pub_exit "--first-packet-ms 10000 --idle-ms 2000 --overall-ms 60000" \
                "--count 50"  no_early_exit
