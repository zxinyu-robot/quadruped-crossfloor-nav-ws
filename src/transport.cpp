#include "legged_nav/transport.hpp"

#include "legged_nav/task_intent.hpp"

namespace legged_nav {

namespace {
const std::map<std::string, Channel> kChannels = {
    {"rgb/frame", {"rgb/frame", QoS::BEST_EFFORT, "sensor.rgb_frame.v1", "RGB 视觉流上行"}},
    {"sensor/lidar_scan",
     {"sensor/lidar_scan", QoS::BEST_EFFORT, "sensor.scan.v1", "LiDAR 点云/扫描"}},
    {"sensor/imu", {"sensor/imu", QoS::BEST_EFFORT, "sensor.imu.v1", "IMU 数据"}},
    {"spatial/state",
     {"spatial/state", QoS::BEST_EFFORT, "spatial.state.v1", "位姿·置信度·地图版本"}},
    {"perception/semantic_obs",
     {"perception/semantic_obs", QoS::RELIABLE, "perception.semantic_observation.v1",
      "VLM 语义观测"}},
    {"task/intent", {"task/intent", QoS::RELIABLE, "motionslam.task_intent.v1", "技能级决策下行"}},
    {"motion/action_step",
     {"motion/action_step", QoS::RELIABLE, "motion.action_step.v1",
      "高层运动原语（禁含 cmd_vel）"}},
    {"exec/feedback",
     {"exec/feedback", QoS::RELIABLE, "motionslam.execution_feedback.v1", "执行反馈回流"}},
};
}  // namespace

const std::map<std::string, Channel>& channels() { return kChannels; }

std::string scan_forbidden_payload(const JsonValue& v) {
    if (v.is_obj()) {
        for (const auto& [k, val] : v.obj()) {
            if (FORBIDDEN_CONTROL_KEYS.count(k)) return "REJECT_FORBIDDEN_CONTROL:" + k;
            std::string n = scan_forbidden_payload(val);
            if (!n.empty()) return n;
        }
    } else if (v.is_arr()) {
        for (const auto& item : v.arr()) {
            std::string n = scan_forbidden_payload(item);
            if (!n.empty()) return n;
        }
    }
    return "";
}

std::optional<std::string> InProcessTransport::publish(const std::string& channel,
                                                       const JsonValue& payload) {
    if (!kChannels.count(channel)) return "REJECT_UNKNOWN_CHANNEL:" + channel;
    std::string bad = scan_forbidden_payload(payload);
    if (!bad.empty()) return bad;
    Message msg{channel, payload, now_ns()};
    log_.push_back(msg);
    auto it = subs_.find(channel);
    if (it != subs_.end())
        for (auto& cb : it->second) cb(msg);
    return std::nullopt;
}

}  // namespace legged_nav
