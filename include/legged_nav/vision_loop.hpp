#pragma once
/// @file vision_loop.hpp
/// @brief 视觉驱动闭环编排：RGB 帧流 → VLM 观测 → 意图合成 → TaskRuntime。
/// 语义决策保守可审计；空间门控仍由 TaskRuntime 独立裁决。

#include <optional>
#include <string>
#include <vector>

#include "legged_nav/runtime.hpp"
#include "legged_nav/spatial_context.hpp"
#include "legged_nav/vlm_adapter.hpp"

namespace legged_nav {

struct VisionContext {
    std::string task_prompt;
    std::string platform;
    std::string robot_id;
    std::string map_id;
    int revision = 0;
    std::string frame_id = "world";
    std::optional<IntentGoal> goal;
    double deadline_s = 120.0;
    double min_confidence = 0.6;
};

struct VisionDecision {
    SemanticObservation observation;
    std::optional<std::string> intent_json;
    std::optional<std::map<std::string, std::string>> feedback;
    std::string reason;
};

/// 语义观测 → TaskIntent JSON；返回 (json, 原因) 或 (nullopt, 原因)。
std::pair<std::optional<std::string>, std::string> observation_to_intent(
    const SemanticObservation& obs, const VisionContext& ctx, int seq);

class VisionLoop {
   public:
    VisionLoop(VLMBackend& backend, TaskRuntime& runtime, VisionContext ctx,
               StreamContext* stream = nullptr)
        : backend_(backend), runtime_(runtime), ctx_(std::move(ctx)), stream_(stream) {}

    VisionDecision step(const std::string& frame, const std::string& frame_id, int seq,
                        std::optional<std::tuple<double, double, double>> pose = std::nullopt);

    const std::vector<VisionDecision>& decisions() const { return decisions_; }

   private:
    VLMBackend& backend_;
    TaskRuntime& runtime_;
    VisionContext ctx_;
    StreamContext* stream_;
    std::vector<VisionDecision> decisions_;
};

}  // namespace legged_nav
