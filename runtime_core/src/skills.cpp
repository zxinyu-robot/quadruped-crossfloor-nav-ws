#include "legged_nav/skills.hpp"

#include "legged_nav/contract_gen.hpp"

namespace legged_nav {
namespace {

const contract::SkillSpec* find_skill(const std::string& name) {
    for (const auto& spec : contract::kSkills)
        if (spec.name == name) return &spec;
    return nullptr;
}

std::map<std::string, std::string> params_for(const std::string& name) {
    std::map<std::string, std::string> params;
    for (const auto& item : contract::kSkillParams)
        if (item.skill == name) params.emplace(std::string(item.key), std::string(item.value));
    return params;
}

}  // namespace

std::vector<ActionStep> compile_skill(const IntentSkill& skill) {
    const contract::SkillSpec* spec = find_skill(skill.name);
    if (spec == nullptr) throw SkillError("unsupported skill: " + skill.name);
    if (spec->kind == "navigate_to" && !skill.goal) throw SkillError("Navigate requires goal");
    return {
        ActionStep{std::string(spec->kind), skill.goal, spec->timeout_s, params_for(skill.name)}};
}

}  // namespace legged_nav
