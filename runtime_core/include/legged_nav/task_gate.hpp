#pragma once
/// @file task_gate.hpp
/// @brief 五级准入闸门（C++ 权威实现）。
///
/// 与 `src/legged_nav/task_gate.py` 逐条同码同序，共享用例语料
/// `tests/task_gate_cases.json`（双实现对照）。
///
///   L1 契约与技能白名单（schema/平台/机器人/技能）+ L2 参数有界
///   L3 状态有效性（置信度 + 有效期 + 地图/代数）→ 降级 stop_and_hold
///   L4 度量可行性（搜索必须有候选）→ 降级 no_candidate
///   L5 安全约束（禁区 fail-closed）
///   仲裁（幂等 / 抢占 / 忙拒）
///
/// 语义只影响偏好，不构成几何可执行许可：L1/L2 只看意图，L3~L5 只信度量状态。

#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "legged_nav/admission.hpp"
#include "legged_nav/spatial_state.hpp"
#include "legged_nav/task_intent.hpp"

namespace legged_nav {

enum class GateDecision { ACCEPT, DEGRADED, REJECT };

inline const char* to_string(GateDecision d) {
    switch (d) {
        case GateDecision::ACCEPT: return "ACCEPT";
        case GateDecision::DEGRADED: return "DEGRADED";
        default: return "REJECT";
    }
}

namespace gate_level {
inline constexpr int kSchema = 1;
inline constexpr int kBounds = 2;
inline constexpr int kValidity = 3;
inline constexpr int kFeasibility = 4;
inline constexpr int kSafety = 5;
inline constexpr int kArbitration = 6;
}  // namespace gate_level

namespace degrade_mode {
inline constexpr const char* kStopAndHold = "stop_and_hold";
inline constexpr const char* kGeometricOnly = "geometric_only";
inline constexpr const char* kNoCandidate = "no_candidate";
}  // namespace degrade_mode

/// 需要"有候选才可执行"的技能（Python 侧同值）。
inline const std::set<std::string> GATE_CANDIDATE_SKILLS = {"ExploreFrontier", "Search"};

struct GateLimits {
    double min_confidence = 0.5;
    long long ttl_ns = 500'000'000LL;
    double max_goal_dist_m = 12.0;
    double min_x = -1.0;
    double min_y = -1.0;
    double max_x = 11.0;
    double max_y = 11.0;
    std::string local_platform;
    std::string local_robot_id;
    int priority_min = -100;
    int priority_max = 100;
};

struct GateResult {
    GateDecision decision = GateDecision::REJECT;
    int level = 0;
    std::string code;
    std::string detail;
    std::string degraded_mode;

    bool ok() const { return decision != GateDecision::REJECT; }
    bool accepted() const { return decision == GateDecision::ACCEPT; }
};

/// 裁决 → BT 动作的唯一解释口径（与 Python `degrade_policy()` 同值）。
inline std::string degrade_policy(const GateResult& r) {
    if (r.decision == GateDecision::ACCEPT) return "run";
    if (r.decision == GateDecision::REJECT) return "reject";
    if (r.degraded_mode == degrade_mode::kGeometricOnly) return "geometric_only";
    return "hold_in_place";
}

class TaskGate {
   public:
    explicit TaskGate(GateLimits limits = GateLimits{})
        : limits_(std::move(limits)),
          state_gate_(limits_.ttl_ns, limits_.min_confidence) {}

    TaskGate(GateLimits limits, TaskAdmission admission)
        : limits_(std::move(limits)),
          admission_(std::move(admission)),
          state_gate_(limits_.ttl_ns, limits_.min_confidence) {}

    /// 按 L1→L5 顺序裁决；首个失败即决定。通过后做任务仲裁。
    /// state 为空表示无空间状态（降级停站，而不是放行）。
    GateResult evaluate(
        const TaskIntent& intent, const SpatialState* state, long long now_ns,
        std::size_t candidate_count = 0, const TaskIntent* active = nullptr,
        const std::map<std::string, std::vector<std::pair<double, double>>>* forbidden_areas =
            nullptr,
        bool record = true) {
        if (auto r = l1_schema(intent)) return *r;
        if (auto r = l2_bounds(intent, state)) return *r;
        if (auto r = l3_validity(intent, state, now_ns)) return *r;
        if (auto r = l4_feasibility(intent, candidate_count)) return *r;
        if (auto r = l5_safety(intent, forbidden_areas)) return *r;

        auto verdict = admission_.decide(intent, active);
        if (std::holds_alternative<std::string>(verdict)) {
            GateResult out;
            out.decision = GateDecision::REJECT;
            out.level = gate_level::kArbitration;
            out.code = std::get<std::string>(verdict);
            return out;
        }
        const auto admitted = std::get<AdmitDecision>(verdict);
        if (record && admitted != AdmitDecision::IDEMPOTENT) admission_.record(intent.task_id);
        GateResult out;
        out.decision = GateDecision::ACCEPT;
        out.level = gate_level::kArbitration;
        out.code = admitted == AdmitDecision::ACCEPT      ? "accept"
                   : admitted == AdmitDecision::PREEMPT   ? "preempt"
                                                          : "idempotent";
        return out;
    }

    const GateLimits& limits() const { return limits_; }
    TaskAdmission& admission() { return admission_; }

   private:
    std::optional<GateResult> l1_schema(const TaskIntent& intent) const {
        if (intent.skills.empty()) return reject(gate_level::kSchema, "REJECT_NO_SKILL");
        if (intent.revision < 0)
            return reject(gate_level::kSchema, "REJECT_REVISION_MISSING");
        if (!limits_.local_platform.empty() && intent.platform != limits_.local_platform) {
            return reject(gate_level::kSchema, "REJECT_PLATFORM_MISMATCH",
                          intent.platform + "!=" + limits_.local_platform);
        }
        if (!limits_.local_robot_id.empty() && intent.robot_id != limits_.local_robot_id) {
            return reject(gate_level::kSchema, "REJECT_ROBOT_MISMATCH",
                          intent.robot_id + "!=" + limits_.local_robot_id);
        }
        for (const auto& s : intent.skills) {
            if (ALLOWED_SKILLS.count(s.name) == 0) {
                return reject(gate_level::kSchema, "REJECT_UNKNOWN_SKILL", s.name);
            }
        }
        return std::nullopt;
    }

    std::optional<GateResult> l2_bounds(const TaskIntent& intent,
                                        const SpatialState* state) const {
        for (const auto& s : intent.skills) {
            if (!s.goal.has_value()) continue;
            const auto& g = *s.goal;
            if (!std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z)) {
                return reject(gate_level::kBounds, "REJECT_GOAL_NOT_FINITE");
            }
            if (g.x < limits_.min_x || g.x > limits_.max_x || g.y < limits_.min_y ||
                g.y > limits_.max_y) {
                return reject(gate_level::kBounds, "REJECT_GOAL_OUT_OF_BOUNDS");
            }
            if (state != nullptr) {
                const double dx = g.x - state->x;
                const double dy = g.y - state->y;
                const double dist = std::sqrt(dx * dx + dy * dy);
                if (dist > limits_.max_goal_dist_m) {
                    return reject(gate_level::kBounds, "REJECT_GOAL_TOO_FAR");
                }
            }
        }
        if (intent.priority < limits_.priority_min || intent.priority > limits_.priority_max) {
            return reject(gate_level::kBounds, "REJECT_PRIORITY_RANGE");
        }
        if (intent.deadline_s < 0.0) {
            return reject(gate_level::kBounds, "REJECT_DEADLINE_RANGE");
        }
        return std::nullopt;
    }

    std::optional<GateResult> l3_validity(const TaskIntent& intent, const SpatialState* state,
                                          long long now_ns) const {
        if (state == nullptr) {
            GateResult out;
            out.decision = GateDecision::DEGRADED;
            out.level = gate_level::kValidity;
            out.code = "SAFETY_INTERLOCK:no_state";
            out.detail = "无空间状态";
            out.degraded_mode = degrade_mode::kStopAndHold;
            return out;
        }
        auto reason = state_gate_.check(*state, intent, now_ns);
        if (!reason.has_value()) return std::nullopt;
        GateResult out;
        out.decision = GateDecision::DEGRADED;
        out.level = gate_level::kValidity;
        out.code = *reason;
        out.detail = "状态过期/置信度不足/代数不一致";
        out.degraded_mode = degrade_mode::kStopAndHold;
        return out;
    }

    std::optional<GateResult> l4_feasibility(const TaskIntent& intent,
                                             std::size_t candidate_count) const {
        bool needs = false;
        for (const auto& s : intent.skills) {
            if (GATE_CANDIDATE_SKILLS.count(s.name) != 0) needs = true;
        }
        if (needs && candidate_count == 0) {
            GateResult out;
            out.decision = GateDecision::DEGRADED;
            out.level = gate_level::kFeasibility;
            out.code = "REJECT_NO_CANDIDATE";
            out.detail = "搜索技能无可用候选前沿";
            out.degraded_mode = degrade_mode::kNoCandidate;
            return out;
        }
        return std::nullopt;
    }

    std::optional<GateResult> l5_safety(
        const TaskIntent& intent,
        const std::map<std::string, std::vector<std::pair<double, double>>>* areas) const {
        if (areas == nullptr || areas->empty()) return std::nullopt;
        if (auto reason = check_forbidden_areas(intent, *areas)) {
            return reject(gate_level::kSafety, *reason, "禁区约束");
        }
        // 本地禁行区：无论意图是否点名，目标落在区内即拒（端侧拥有最终否决权）。
        for (const auto& [area_id, poly] : *areas) {
            if (poly.size() < 3) continue;
            for (std::size_t i = 0; i < intent.skills.size(); ++i) {
                if (!intent.skills[i].goal.has_value()) continue;
                const auto& g = *intent.skills[i].goal;
                if (point_in_polygon(g.x, g.y, poly)) {
                    return reject(gate_level::kSafety,
                                  "REJECT_GOAL_IN_FORBIDDEN_AREA:" + area_id + "@skill" +
                                      std::to_string(i),
                                  "本地禁行区");
                }
            }
        }
        return std::nullopt;
    }

    static bool point_in_polygon(double x, double y,
                                 const std::vector<std::pair<double, double>>& poly) {
        bool inside = false;
        const std::size_t n = poly.size();
        for (std::size_t i = 0; i < n; ++i) {
            const auto& [x1, y1] = poly[i];
            const auto& [x2, y2] = poly[(i + 1) % n];
            if ((y1 > y) != (y2 > y) &&
                x < x1 + (y - y1) * (x2 - x1) / (y2 - y1)) {
                inside = !inside;
            }
        }
        return inside;
    }

    static GateResult reject(int level, std::string code, std::string detail = "") {
        GateResult out;
        out.decision = GateDecision::REJECT;
        out.level = level;
        out.code = std::move(code);
        out.detail = std::move(detail);
        return out;
    }

    GateLimits limits_;
    TaskAdmission admission_;
    StateGate state_gate_;
};

}  // namespace legged_nav
