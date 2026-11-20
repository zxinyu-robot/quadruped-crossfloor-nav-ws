#pragma once
/// @file task_intent.hpp
/// @brief Task Intent 契约校验与禁区检查（C++ 权威实现）。
/// schema: motionslam.task_intent.v1。错误语义：返回 REJECT_* 字符串或 std::nullopt。
/// 安全隔离：意图任意层级禁止携带底层控制字段（cmd_vel/twist/轨迹话题名）。

#include <cstddef>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "legged_nav/contract_gen.hpp"
#include "legged_nav/json.hpp"

namespace legged_nav {

// 契约常量唯一来源：contracts/runtime_contract.json（经 scripts/gen_contract.py 生成），
// 与 Python 侧 src/legged_nav/contract_gen.py 完全同源。
inline constexpr std::string_view SCHEMA_INTENT = contract::kIntentSchema;
inline constexpr std::string_view SCHEMA_FEEDBACK = contract::kFeedbackSchema;

namespace detail {

template <std::size_t N>
std::set<std::string> to_set(const std::string_view (&items)[N]) {
    std::set<std::string> out;
    for (const auto& item : items) out.emplace(item);
    return out;
}

}  // namespace detail

inline const std::set<std::string> ALLOWED_PLATFORMS = detail::to_set(contract::kAllowedPlatforms);
inline const std::set<std::string> ALLOWED_SKILLS = detail::to_set(contract::kAllowedSkills);
inline const std::set<std::string> FORBIDDEN_CONTROL_KEYS =
    detail::to_set(contract::kForbiddenControlKeys);

struct IntentGoal {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    std::string frame_id = "world";  ///< world 系，米
};

struct IntentSkill {
    std::string name;
    std::optional<IntentGoal> goal;
};

struct TaskIntent {
    std::string task_id;
    std::string platform;
    std::string robot_id;
    std::string map_id;
    std::string floor_id;
    std::string frame_id = "world";
    int revision = -1;
    std::vector<IntentSkill> skills;
    double deadline_s = 0.0;  ///< 秒；<=0 表示无期限
    int priority = 0;
    bool preemptible = false;
    std::vector<std::string> forbidden_areas;
};

struct IntentOptions {
    std::optional<std::set<std::string>> allowed_skills;
    std::string local_platform;
    std::string local_robot_id;
};

/// 校验 TaskIntent JSON；成功返回 intent，失败返回错误码字符串。
std::variant<TaskIntent, std::string> validate_task_intent(const JsonValue& raw,
                                                           const IntentOptions& opt = {});

/// 便捷入口：JSON 文本版；解析失败返回 "REJECT_INTENT_TYPE"。
std::variant<TaskIntent, std::string> validate_task_intent_text(const std::string& text,
                                                                const IntentOptions& opt = {});

/// 禁区校验；仅校验 Navigate 目标点。未知区域 ID fail closed。
/// 返回 std::nullopt（通过）或 REJECT_* 错误码。
std::optional<std::string> check_forbidden_areas(
    const TaskIntent& intent,
    const std::map<std::string, std::vector<std::pair<double, double>>>& areas);

}  // namespace legged_nav
