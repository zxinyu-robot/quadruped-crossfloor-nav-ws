#pragma once
/// @file spatial_state.hpp
/// @brief 空间状态契约：位姿 + 置信度 + 地图版本的有效性检查。
/// 失效状态触发 SAFETY_INTERLOCK，任务 HOLD 而不是带过期定位继续执行。

#include <optional>
#include <string>
#include <vector>

namespace legged_nav {

struct TaskIntent;  // task_intent.hpp

/// 轴角旋转（rx, ry, rz，弧度）→ yaw（度）。
/// ARKitScenes `.traj` 旋转列为轴角表示（方向=轴、模=角）；
/// Rodrigues 公式取第一列 xy 投影。零向量返回 0.0。
double yawFromAxisAngle(double rx, double ry, double rz);

/// 深度图前向走廊障碍门控：中央 corridor_frac 宽度的最小深度 < threshold_mm
/// 时返回全部 hazards，否则返回空。depth_mm 无效/空时保守返回全部。
/// depth_mm 为行优先扁平 buffer（尺寸 h*w）。
std::vector<std::string> blockingHazards(const std::vector<double>& depth_mm, int h, int w,
                                         const std::vector<std::string>& hazards,
                                         double corridor_frac = 0.4, double threshold_mm = 900.0);

struct SpatialState {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw_deg = 0.0;
    std::string map_id;
    int revision = 0;
    double localization_confidence = 0.0;  ///< 0~1
    bool valid = false;
    long long timestamp_ns = 0;  ///< 本地时钟 ns
};

/// 空间状态门控。ttl_ns 为有效期；min_confidence 为置信度下限。
class StateGate {
   public:
    explicit StateGate(long long ttl_ns = 500'000'000LL, double min_confidence = 0.5)
        : ttl_ns_(ttl_ns), min_confidence_(min_confidence) {}

    /// 返回 std::nullopt（放行）或 "SAFETY_INTERLOCK:<原因>"。
    std::optional<std::string> check(const SpatialState& state, const TaskIntent& intent,
                                     long long now_ns) const;

   private:
    long long ttl_ns_;
    double min_confidence_;
};

}  // namespace legged_nav
