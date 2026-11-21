#include "legged_nav/feedback.hpp"

#include "legged_nav/task_intent.hpp"

namespace legged_nav {

std::map<std::string, std::string> build_feedback(
    const std::string& task_id, const std::string& state, const std::string& event,
    const std::map<std::string, std::string>& progress, const std::string& detail,
    long long timestamp_ns) {
    std::map<std::string, std::string> fb{
        {"schema", std::string(SCHEMA_FEEDBACK)},
        {"task_id", task_id},
        {"state", state},
        {"event", event},
        {"detail", detail},
        {"timestamp_ns", std::to_string(timestamp_ns ? timestamp_ns : now_ns())},
    };
    for (const auto& [k, v] : progress) fb["progress." + k] = v;
    return fb;
}

}  // namespace legged_nav
