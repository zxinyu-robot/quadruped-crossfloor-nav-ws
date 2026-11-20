#include "legged_nav/spatial_state.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "legged_nav/task_intent.hpp"

namespace legged_nav {

double yawFromAxisAngle(double rx, double ry, double rz) {
    double theta = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (theta == 0.0) return 0.0;
    double kx = rx / theta, ky = ry / theta, kz = rz / theta;
    double c = std::cos(theta), s = std::sin(theta);
    double r00 = c + kx * kx * (1.0 - c);
    double r10 = kx * ky * (1.0 - c) + kz * s;
    return std::atan2(r10, r00) * 180.0 / M_PI;
}

std::vector<std::string> blockingHazards(const std::vector<double>& depth_mm, int h, int w,
                                         const std::vector<std::string>& hazards,
                                         double corridor_frac, double threshold_mm) {
    if (depth_mm.empty() || h <= 0 || w <= 0 || static_cast<int>(depth_mm.size()) != h * w)
        return hazards;
    int cw = std::max(1, static_cast<int>(w * corridor_frac));
    int c0 = (w - cw) / 2;
    double min_depth = std::numeric_limits<double>::infinity();
    for (int r = 0; r < h; ++r)
        for (int c = c0; c < c0 + cw; ++c) {
            double d = depth_mm[r * w + c];
            if (d > 0 && d < min_depth) min_depth = d;
        }
    if (min_depth >= threshold_mm) return {};
    return hazards;
}

std::optional<std::string> StateGate::check(const SpatialState& state, const TaskIntent& intent,
                                            long long now_ns) const {
    if (!state.valid) return "SAFETY_INTERLOCK:state_invalid";
    if (now_ns - state.timestamp_ns > ttl_ns_) return "SAFETY_INTERLOCK:state_expired";
    if (state.localization_confidence < min_confidence_) return "SAFETY_INTERLOCK:low_confidence";
    if (!intent.map_id.empty() && !state.map_id.empty() && intent.map_id != state.map_id)
        return "SAFETY_INTERLOCK:map_mismatch:" + state.map_id + "!=" + intent.map_id;
    if (state.revision != intent.revision)
        return "SAFETY_INTERLOCK:revision_mismatch:" + std::to_string(state.revision) +
               "!=" + std::to_string(intent.revision);
    return std::nullopt;
}

}  // namespace legged_nav
