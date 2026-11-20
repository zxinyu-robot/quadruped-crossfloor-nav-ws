#!/usr/bin/env bash
# 无 Docker 原生环境引导云端仿真依赖（Ubuntu 22.04 → ROS 2 Humble，24.04 → ROS 2 Jazzy）
#
# 为什么需要它：docker/Dockerfile.cloud 要求宿主提供 docker；部分 GPU pod 只给容器本身
# （无 docker.sock、无 CAP_SYS_ADMIN，装不了 dockerd），只能在系统里直接装 ROS 2。
# 本脚本按 Distro 自动选 Humble/Jazzy，并补齐仓库脚本写死的 /ws 与 /opt/ros/humble 约定。
#
# 用法（root）：./scripts/dev/cloud_sim_bootstrap_jazzy.sh [--build]
#   --build  引导完成后顺带编译 scan_planner_ws 与主 workspace
# 幂等：已存在的 apt 源 / 已克隆的第三方 / 已建软链会跳过。
set -eo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SCRIPTS/.." && pwd)"
UNITREE_COMMIT=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
SCAN_BRANCH=ros2-community
APT_MIRROR="${ROS_APT_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
ROSDISTRO_MIRROR="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml}"
# 云端用 vkernel 代替传感器，以下密钥在云上无对应包（详见 THIRD_PARTY.md）
SKIP_KEYS="yaml-cpp-vendor livox_ros_driver2 super_lio"

DO_BUILD=0
[[ "${1:-}" == "--build" ]] && DO_BUILD=1

log() { echo "[bootstrap] $*"; }

[[ "$(id -u)" -eq 0 ]] || { echo "ERROR: 需要 root（apt / 软链）"; exit 1; }

. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
case "$CODENAME" in
  jammy) ROS_DISTRO_NAME=humble ;;
  noble) ROS_DISTRO_NAME=jazzy ;;
  *) echo "ERROR: 仅支持 Ubuntu 22.04(jammy) / 24.04(noble)，当前 ${PRETTY_NAME}"; exit 1 ;;
esac
log "Ubuntu ${VERSION_ID} → ROS 2 ${ROS_DISTRO_NAME}"

# ---- 1. apt 源与密钥 ----
export DEBIAN_FRONTEND=noninteractive
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
if [[ ! -s "$KEYRING" ]]; then
  log "导入 ROS apt 密钥"
  curl -fsSL -m 60 "https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key" -o "$KEYRING" \
    || curl -fsSL -m 60 "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" -o "$KEYRING"
fi
if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  log "写入 ROS 2 apt 源（清华镜像）"
  echo "deb [arch=amd64 signed-by=${KEYRING}] ${APT_MIRROR} ${CODENAME} main" \
    > /etc/apt/sources.list.d/ros2.list
fi
apt-get update -qq

# ---- 2. ROS 2 与编译依赖 ----
APT_PKGS=(
  "ros-${ROS_DISTRO_NAME}-ros-base"
  "ros-${ROS_DISTRO_NAME}-rmw-cyclonedds-cpp"
  "ros-${ROS_DISTRO_NAME}-pcl-ros"
  "ros-${ROS_DISTRO_NAME}-pcl-conversions"
  "ros-${ROS_DISTRO_NAME}-nav2-msgs"
  "ros-${ROS_DISTRO_NAME}-tf2-ros"
  "ros-${ROS_DISTRO_NAME}-tf2-geometry-msgs"
  "ros-${ROS_DISTRO_NAME}-tf-transformations"
  "ros-${ROS_DISTRO_NAME}-rosbag2-storage-mcap"
  "ros-${ROS_DISTRO_NAME}-rosidl-generator-dds-idl"
  "ros-${ROS_DISTRO_NAME}-yaml-cpp-vendor"
  "ros-${ROS_DISTRO_NAME}-lifecycle-msgs"
  "ros-${ROS_DISTRO_NAME}-rclcpp-lifecycle"
  "ros-${ROS_DISTRO_NAME}-rclcpp-action"
  "ros-${ROS_DISTRO_NAME}-visualization-msgs"
  "ros-${ROS_DISTRO_NAME}-cv-bridge"
  "ros-${ROS_DISTRO_NAME}-image-transport"
  "ros-${ROS_DISTRO_NAME}-vision-opencv"
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-numpy
  build-essential cmake git curl
  libpcl-dev libeigen3-dev libyaml-cpp-dev libgoogle-glog-dev libtbb-dev
  libasio-dev libwebsocketpp-dev
)
log "安装 ROS 2 ${ROS_DISTRO_NAME} 与编译依赖"
apt-get install -y -qq --no-install-recommends "${APT_PKGS[@]}"

# rosdep / colcon 依赖 ROS_DISTRO 与 ament 环境
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

# ---- 3. 兼容仓库脚本写死的路径约定 ----
# 仓库脚本固定 source /opt/ros/humble/setup.bash 与 /ws/...，非 Humble 环境用软链对齐
if [[ ! -e /opt/ros/humble ]]; then
  ln -sfn "/opt/ros/${ROS_DISTRO_NAME}" /opt/ros/humble
  log "/opt/ros/humble -> /opt/ros/${ROS_DISTRO_NAME}"
fi
if [[ ! -e /ws ]]; then
  ln -sfn "$ROOT" /ws
  log "/ws -> $ROOT"
fi

# Jazzy 把 cv_bridge/cv_bridge.h 改名为 .hpp，SCAN-Planner 仍按旧名 include
CV_HPP="/opt/ros/${ROS_DISTRO_NAME}/include/cv_bridge/cv_bridge/cv_bridge.hpp"
CV_LEGACY="/opt/ros/${ROS_DISTRO_NAME}/include/cv_bridge/cv_bridge.h"
if [[ ! -e "$CV_LEGACY" && -e "$CV_HPP" ]]; then
  install -d /usr/local/include/cv_bridge
  ln -sfn "$CV_HPP" /usr/local/include/cv_bridge/cv_bridge.h
  log "cv_bridge.h 兼容软链已建立"
fi

# ---- 4. 第三方克隆 ----
if [[ ! -d "$ROOT/src/unitree_ros2" ]]; then
  log "克隆 unitree_ros2"
  git clone --filter=blob:none https://github.com/unitreerobotics/unitree_ros2.git \
    "$ROOT/src/unitree_ros2"
fi
git -C "$ROOT/src/unitree_ros2" checkout -q "$UNITREE_COMMIT"

if [[ ! -d "$ROOT/scan_planner_ws/src/SCAN-Planner" ]]; then
  log "克隆 SCAN-Planner（${SCAN_BRANCH}）"
  mkdir -p "$ROOT/scan_planner_ws/src"
  git clone --depth 1 -b "$SCAN_BRANCH" https://github.com/wuyi2121/SCAN-Planner.git \
    "$ROOT/scan_planner_ws/src/SCAN-Planner"
fi
SCAN_DIR="$ROOT/scan_planner_ws/src/SCAN-Planner"
if ! git -C "$SCAN_DIR" diff --quiet -- . 2>/dev/null; then
  log "SCAN-Planner 已有本地改动，跳过 patch"
elif git -C "$SCAN_DIR" apply --check "$ROOT/patches/scan-planner-motionslam.patch" 2>/dev/null; then
  git -C "$SCAN_DIR" apply "$ROOT/patches/scan-planner-motionslam.patch"
  log "SCAN-Planner patch 完整应用"
else
  # 已知：closed_loop_controller.cpp 的 hunk 与上游当前实现冲突（上游已移除 tracking_core_），
  # 该处跳过即可，其余 hunk 正常应用；详见 docs/云端闭环_vkernel.md
  log "SCAN-Planner patch 部分冲突，按 reject 方式应用"
  git -C "$SCAN_DIR" apply --reject "$ROOT/patches/scan-planner-motionslam.patch" || true
  find "$SCAN_DIR" -name '*.rej' -o -name '*.orig' | xargs -r rm -f
fi

# ---- 5. rosdep ----
export ROSDISTRO_INDEX_URL="$ROSDISTRO_MIRROR"
rosdep init 2>/dev/null || true
log "rosdep update"
rosdep update -q || log "WARN: rosdep update 失败，继续（可能已有缓存）"
log "rosdep install（跳过云上用不到的密钥：${SKIP_KEYS}）"
rosdep install --from-paths "$ROOT/scan_planner_ws/src" "$ROOT/src" \
  --ignore-src --rosdistro "$ROS_DISTRO_NAME" --skip-keys "$SKIP_KEYS" -y -q

# ---- 6. 可选编译 ----
if [[ "$DO_BUILD" -eq 1 ]]; then
  log "编译 scan_planner_ws"
  ( cd "$ROOT/scan_planner_ws" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release )
  log "编译主 workspace"
  ( cd "$ROOT" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release )
fi

log "完成。后续："
log "  source /ws/scripts/dev/source_cloud_env.sh"
log "  bash /ws/scripts/dev/cloud_sim_multifloor.sh          # 跨楼层仿真"
log "  python3 /ws/src/motionslam_bringup/scripts/cloud_mission_watch.py   # 冒烟判据"
