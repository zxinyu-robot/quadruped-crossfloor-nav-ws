#pragma once
/// @file executor.hpp
/// @brief 执行层：Forwarder 协议 + 仿真执行器。
/// Forwarder 只接受高层 ActionStep 原语，不接受意图层的速度/轨迹字段。
/// SimForwarder 仅记录指令流并更新仿真位姿（证据等级：合成数据·单元测试）。

#include <chrono>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "legged_nav/skills.hpp"
#include "legged_nav/spatial_state.hpp"

namespace legged_nav {

inline long long now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

struct StepResult {
    bool ok = false;
    std::string detail;
    std::optional<SpatialState> final_state;
};

/// 平台执行适配接口（Unitree Sport API 由部署侧实现）。
class Forwarder {
   public:
    virtual ~Forwarder() = default;
    virtual StepResult execute(const ActionStep& step, long long deadline_ns) = 0;
    virtual void abort(const std::string& reason) = 0;
    virtual SpatialState state() const = 0;
};

/// 仿真执行器：不产生真实运动指令。
class SimForwarder : public Forwarder {
   public:
    explicit SimForwarder(SpatialState initial = {}) {
        state_ = initial;
        state_.valid = true;
        if (state_.localization_confidence == 0.0) state_.localization_confidence = 1.0;
        state_.timestamp_ns = now_ns();
    }

    StepResult execute(const ActionStep& step, long long /*deadline_ns*/) override {
        steps_.push_back({step.kind, step.goal});
        if (fail_kinds_.count(step.kind))
            return StepResult{false, "sim_fail:" + step.kind, std::nullopt};
        if (step.kind == "navigate_to" && step.goal) {
            state_.x = step.goal->x;
            state_.y = step.goal->y;
            state_.z = step.goal->z;
        } else if (step.kind == "return_home") {
            state_.x = 0.0;
            state_.y = 0.0;
        }
        state_.timestamp_ns = now_ns();
        return StepResult{true, "", state_};
    }

    void abort(const std::string& reason) override {
        steps_.push_back({"abort:" + reason, std::nullopt});
    }

    SpatialState state() const override { return state_; }

    /// 注入失败的动作类型（测试用）。
    void fail_on(const std::string& kind) { fail_kinds_.insert(kind); }
    const std::vector<std::pair<std::string, std::optional<IntentGoal>>>& steps() const {
        return steps_;
    }

   private:
    SpatialState state_;
    std::set<std::string> fail_kinds_;
    std::vector<std::pair<std::string, std::optional<IntentGoal>>> steps_;
};

}  // namespace legged_nav
