#pragma once
/// @file skills.hpp
/// @brief 技能层：IntentSkill → ActionStep 序列。技能是决策与底层控制间的唯一通道，
///        不产生 cmd_vel。

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "legged_nav/task_intent.hpp"

namespace legged_nav {

struct ActionStep {
    std::string kind;  ///< navigate_to | return_home | explore | stand | stop
    std::optional<IntentGoal> goal;
    double timeout_s = 30.0;
    std::map<std::string, std::string> params;
};

class SkillError : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

/// 技能 → 动作序列；未知技能抛 SkillError（fail closed）。
std::vector<ActionStep> compile_skill(const IntentSkill& skill);

}  // namespace legged_nav
