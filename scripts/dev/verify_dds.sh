#!/usr/bin/env bash
# 在仓库容器内一条命令验证 runtime_core 的原生 DDS 链路。
#
#   ./docker/run_container.sh ./scripts/dev/verify_dds.sh
#   ./docker/run_container.sh "./scripts/dev/verify_dds.sh --no-qos"   # 跳过 QoS 矩阵
#
# 覆盖：构建 → 契约一致性 → DDS 类型一致性 → C++ 测试 → ROS↔Native 双向互通 → QoS 兼容性矩阵
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_QOS=1
for a in "$@"; do
    case "$a" in
        --no-qos) RUN_QOS=0 ;;
        *) echo "未知参数: $a" >&2; exit 2 ;;
    esac
done

for d in humble jazzy; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
        set +u; . "/opt/ros/$d/setup.bash"; set -u
        echo "[verify_dds] ROS distro: $d"
        break
    fi
done
if [ -f "$WS/install/setup.bash" ]; then
    set +u; . "$WS/install/setup.bash"; set -u
else
    echo "[verify_dds] WARN: $WS/install/setup.bash 不存在，互通实验将缺 motionslam_msgs 类型" >&2
fi

cd "$WS/runtime_core"

echo "[verify_dds] 1/6 配置与构建"
cmake -S . -B build -DLEGGED_NAV_BUILD_TESTS=ON -DLEGGED_NAV_BUILD_DDS=ON
cmake --build build -j"$(nproc)"

echo "[verify_dds] 2/6 契约生成物一致性"
python3 scripts/gen_contract.py --check

echo "[verify_dds] 3/6 DDS 类型生成物一致性"
python3 scripts/gen_dds_types.py --check

echo "[verify_dds] 4/6 C++ 测试"
ctest --test-dir build --output-on-failure

echo "[verify_dds] 5/6 ROS ↔ Native 双向互通"
INTEROP_ARGS=(--native build/dds/interop_native)
if [ -f "$WS/install/setup.bash" ]; then
    INTEROP_ARGS+=(--ws-setup "$WS/install/setup.bash")
fi
python3 scripts/dds_interop_check.py "${INTEROP_ARGS[@]}"

if [ "$RUN_QOS" = "1" ]; then
    echo "[verify_dds] 6/6 QoS 兼容性矩阵"
    bash scripts/dds_qos_matrix.sh build/dds/dds_bench
else
    echo "[verify_dds] 6/6 已跳过（--no-qos）"
fi

echo "[verify_dds] 全部通过"
