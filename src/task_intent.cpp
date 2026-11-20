#include "legged_nav/task_intent.hpp"

namespace legged_nav {
namespace {

std::string trim_ascii(std::string text) {
    const auto space = [](unsigned char c) {
        return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
    };
    std::size_t b = 0, e = text.size();
    while (b < e && space(static_cast<unsigned char>(text[b]))) ++b;
    while (e > b && space(static_cast<unsigned char>(text[e - 1]))) --e;
    return text.substr(b, e - b);
}

/// 契约归一化：仅接受字符串，去 ASCII 首尾空白；其余类型视为缺失（与 Python _norm 一致）。
std::string norm(const JsonValue* v) { return (v && v->is_str()) ? trim_ascii(v->str()) : ""; }

std::string has_forbidden_control(const JsonValue& v) {
    if (v.is_obj()) {
        for (const auto& [k, val] : v.obj()) {
            if (FORBIDDEN_CONTROL_KEYS.count(k)) return "REJECT_FORBIDDEN_CONTROL:" + k;
            std::string nested = has_forbidden_control(val);
            if (!nested.empty()) return nested;
        }
    } else if (v.is_arr()) {
        for (const auto& item : v.arr()) {
            std::string nested = has_forbidden_control(item);
            if (!nested.empty()) return nested;
        }
    }
    return "";
}

bool point_in_polygon(double x, double y, const std::vector<std::pair<double, double>>& poly) {
    bool inside = false;
    size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        auto [x1, y1] = poly[i];
        auto [x2, y2] = poly[(i + 1) % n];
        if ((y1 > y) != (y2 > y) && x < x1 + (y - y1) * (x2 - x1) / (y2 - y1)) inside = !inside;
    }
    return inside;
}

}  // namespace

std::variant<TaskIntent, std::string> validate_task_intent_text(const std::string& text,
                                                                const IntentOptions& opt) {
    auto v = parse_json(text);
    if (!v) return std::string("REJECT_INTENT_TYPE");
    return validate_task_intent(*v, opt);
}

std::variant<TaskIntent, std::string> validate_task_intent(const JsonValue& raw,
                                                           const IntentOptions& opt) {
    auto err = [](const std::string& s) { return std::variant<TaskIntent, std::string>(s); };

    if (!raw.is_obj()) return err("REJECT_INTENT_TYPE");
    std::string forbidden = has_forbidden_control(raw);
    if (!forbidden.empty()) return err(forbidden);

    std::string schema = norm(raw.get("schema"));
    if (schema != SCHEMA_INTENT)
        return err("REJECT_SCHEMA:" + (schema.empty() ? "missing" : schema));

    std::string task_id = norm(raw.get("task_id"));
    if (task_id.empty()) return err("REJECT_MISSING_TASK_ID");

    const JsonValue* robot = raw.get("robot");
    std::string platform = (robot && robot->is_obj()) ? norm(robot->get("platform")) : "";
    if (platform.empty()) return err("REJECT_MISSING_ROBOT");
    if (!ALLOWED_PLATFORMS.count(platform)) return err("REJECT_ROBOT_PLATFORM:" + platform);
    std::string sn;
    if (robot && robot->is_obj()) {
        sn = norm(robot->get("sn"));
        if (sn.empty()) sn = norm(robot->get("robot_id"));
    }
    if (sn.empty()) return err("REJECT_MISSING_ROBOT_ID");
    if (!opt.local_platform.empty() && platform != opt.local_platform)
        return err("REJECT_ROBOT_MISMATCH");
    if (!opt.local_robot_id.empty() && sn != opt.local_robot_id)
        return err("REJECT_ROBOT_MISMATCH");

    const JsonValue* world = raw.get("world_ref");
    if (!world || !world->is_obj()) return err("REJECT_MISSING_WORLD_REF");
    std::string map_id = norm(world->get("map_id"));
    if (map_id.empty()) return err("REJECT_MISSING_MAP_ID");
    const JsonValue* rev_v = world->get("revision");
    if (!rev_v || !rev_v->is_num()) return err("REJECT_BAD_REVISION");
    int revision = static_cast<int>(rev_v->num());
    if (revision < 0) return err("REJECT_BAD_REVISION");

    const JsonValue* skills_raw = raw.get("skills");
    if (!skills_raw || !skills_raw->is_arr() || skills_raw->arr().empty())
        return err("REJECT_MISSING_SKILLS");

    std::vector<IntentSkill> skills;
    size_t i = 0;
    for (const auto& item : skills_raw->arr()) {
        if (!item.is_obj()) return err("REJECT_SKILL_" + std::to_string(i) + "_TYPE");
        std::string name = norm(item.get("name"));
        if (name.empty()) name = norm(item.get("skill"));
        if (name.empty()) return err("REJECT_SKILL_" + std::to_string(i) + "_MISSING_NAME");
        if (!ALLOWED_SKILLS.count(name) || (opt.allowed_skills && !opt.allowed_skills->count(name)))
            return err("REJECT_SKILL_UNSUPPORTED:" + name);
        std::optional<IntentGoal> goal;
        if (name == "Navigate") {
            const JsonValue* g = item.get("goal");
            if (!g || !g->is_obj())
                return err("REJECT_SKILL_" + std::to_string(i) + "_MISSING_GOAL");
            auto gx = g->get("x"), gy = g->get("y"), gz = g->get("z");
            if (!gx || !gx->is_num() || !gy || !gy->is_num() || !gz || !gz->is_num())
                return err("REJECT_SKILL_" + std::to_string(i) + "_BAD_GOAL");
            std::string fid = norm(g->get("frame_id"));
            if (fid.empty()) fid = norm(world->get("frame_id"));
            goal = IntentGoal{gx->num(), gy->num(), gz->num(), fid.empty() ? "world" : fid};
        }
        skills.push_back(IntentSkill{name, goal});
        ++i;
    }

    double deadline = 0.0;
    const JsonValue* policy = raw.get("policy");
    if (policy && policy->is_obj()) {
        const JsonValue* dl = policy->get("deadline_s");
        if (dl && !dl->is_null()) {
            if (!dl->is_num()) return err("REJECT_BAD_DEADLINE");
            deadline = dl->num();
            if (deadline <= 0) return err("REJECT_BAD_DEADLINE");
        }
    }

    int priority = 0;
    bool preemptible = false;
    const JsonValue* exec = raw.get("execution");
    if (exec && exec->is_obj()) {
        const JsonValue* p = exec->get("priority");
        if (p && !p->is_num()) return err("REJECT_BAD_PRIORITY");
        priority = p ? static_cast<int>(p->num()) : 0;
        const JsonValue* pr = exec->get("preemptible");
        if (pr && !pr->is_bool()) return err("REJECT_BAD_PREEMPTIBLE");
        preemptible = pr ? pr->boolean() : false;
    }

    std::vector<std::string> forbidden_areas;
    const JsonValue* nav = raw.get("navigation");
    if (nav && nav->is_obj()) {
        const JsonValue* fa = nav->get("forbidden_areas");
        if (fa && !fa->is_null()) {
            if (!fa->is_arr()) return err("REJECT_BAD_FORBIDDEN_AREAS");
            for (const auto& a : fa->arr()) {
                std::string area = norm(&a);
                if (area.empty()) return err("REJECT_BAD_FORBIDDEN_AREAS");
                forbidden_areas.push_back(area);
            }
        }
    }

    TaskIntent intent;
    intent.task_id = task_id;
    intent.platform = platform;
    intent.robot_id = sn;
    intent.map_id = map_id;
    intent.floor_id = norm(world->get("floor_id"));
    intent.frame_id = norm(world->get("frame_id"));
    if (intent.frame_id.empty()) intent.frame_id = "world";
    intent.revision = revision;
    intent.skills = std::move(skills);
    intent.deadline_s = deadline;
    intent.priority = priority;
    intent.preemptible = preemptible;
    intent.forbidden_areas = std::move(forbidden_areas);
    return intent;
}

std::optional<std::string> check_forbidden_areas(
    const TaskIntent& intent,
    const std::map<std::string, std::vector<std::pair<double, double>>>& areas) {
    for (const auto& area_id : intent.forbidden_areas) {
        auto it = areas.find(area_id);
        if (it == areas.end() || it->second.size() < 3) return "REJECT_UNKNOWN_AREA:" + area_id;
        for (size_t i = 0; i < intent.skills.size(); ++i) {
            const auto& skill = intent.skills[i];
            if (skill.goal && point_in_polygon(skill.goal->x, skill.goal->y, it->second))
                return "REJECT_GOAL_IN_FORBIDDEN_AREA:" + area_id + "@skill" + std::to_string(i);
        }
    }
    return std::nullopt;
}

}  // namespace legged_nav
