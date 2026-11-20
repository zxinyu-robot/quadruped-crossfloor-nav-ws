#include "legged_nav/vision_loop.hpp"

namespace legged_nav {

namespace {
std::string goal_json(const IntentGoal& g) {
    return "{\"x\":" + std::to_string(g.x) + ",\"y\":" + std::to_string(g.y) +
           ",\"z\":" + std::to_string(g.z) + ",\"frame_id\":\"" + g.frame_id + "\"}";
}
}  // namespace

std::pair<std::optional<std::string>, std::string> observation_to_intent(
    const SemanticObservation& obs, const VisionContext& ctx, int seq) {
    if (obs.confidence < ctx.min_confidence) {
        return {std::nullopt, "low_observation_confidence:" + std::to_string(obs.confidence)};
    }
    std::string skills;
    std::string reason;
    if (!obs.hazards.empty()) {
        skills = "[{\"name\":\"Stop\"}]";
        reason = "hazard:" + obs.hazards.front();
    } else if (obs.target_visible && ctx.goal) {
        skills = "[{\"name\":\"Navigate\",\"goal\":" + goal_json(*ctx.goal) + "}]";
        reason = "target_visible";
    } else {
        return {std::nullopt, "no_actionable_observation"};
    }

    char seq_buf[16];
    std::snprintf(seq_buf, sizeof(seq_buf), "%04d", seq);
    std::string json =
        "{\"schema\":\"" + std::string(SCHEMA_INTENT) + "\",\"task_id\":\"vision-" + seq_buf +
        "\",\"robot\":{\"platform\":\"" + ctx.platform + "\",\"sn\":\"" + ctx.robot_id +
        "\"},\"world_ref\":{\"map_id\":\"" + ctx.map_id + "\",\"frame_id\":\"" + ctx.frame_id +
        "\",\"revision\":" + std::to_string(ctx.revision) + "},\"skills\":" + skills +
        ",\"policy\":{\"deadline_s\":" + std::to_string(ctx.deadline_s) +
        "},\"execution\":{\"priority\":0,\"preemptible\":true}}";
    return {json, reason};
}

VisionDecision VisionLoop::step(const std::string& frame, const std::string& frame_id, int seq,
                                std::optional<std::tuple<double, double, double>> pose) {
    SemanticObservation obs = backend_.observe(frame, ctx_.task_prompt, frame_id);
    if (stream_ && pose) {
        std::string summary = obs.scene_class;
        for (const auto& t : obs.landmarks) summary += "," + t;
        for (const auto& t : obs.hazards) summary += "," + t;
        stream_->push(VisionToken{frame_id, summary, *pose, now_ns(), true});
    }
    auto [intent, reason] = observation_to_intent(obs, ctx_, seq);
    std::optional<std::map<std::string, std::string>> fb;
    if (intent) fb = runtime_.submit(*intent);
    VisionDecision dec{obs, intent, fb, reason};
    decisions_.push_back(dec);
    return dec;
}

}  // namespace legged_nav
