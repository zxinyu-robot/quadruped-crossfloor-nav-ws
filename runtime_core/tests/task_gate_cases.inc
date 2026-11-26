// tests/task_gate_cases.inc —— 由 test_all.cpp include；共享语料双实现对照。
// 语料：tests/task_gate_cases.json（Python 侧 tests/test_task_gate_corpus.py 读同一份）。

static GateLimits limits_from(const JsonValue& j, const GateLimits& base) {
    GateLimits out = base;
    if (auto v = j.get("min_confidence")) out.min_confidence = v->num();
    if (auto v = j.get("ttl_ns")) out.ttl_ns = static_cast<long long>(v->num());
    if (auto v = j.get("max_goal_dist_m")) out.max_goal_dist_m = v->num();
    if (auto v = j.get("min_x")) out.min_x = v->num();
    if (auto v = j.get("min_y")) out.min_y = v->num();
    if (auto v = j.get("max_x")) out.max_x = v->num();
    if (auto v = j.get("max_y")) out.max_y = v->num();
    if (auto v = j.get("local_platform")) out.local_platform = v->str_or();
    if (auto v = j.get("local_robot_id")) out.local_robot_id = v->str_or();
    if (auto v = j.get("priority_min")) out.priority_min = static_cast<int>(v->num());
    if (auto v = j.get("priority_max")) out.priority_max = static_cast<int>(v->num());
    return out;
}

static SpatialState state_from(const JsonValue& j, const SpatialState& base, long long now_ns) {
    SpatialState out = base;
    if (auto v = j.get("x")) out.x = v->num();
    if (auto v = j.get("y")) out.y = v->num();
    if (auto v = j.get("z")) out.z = v->num();
    if (auto v = j.get("yaw_deg")) out.yaw_deg = v->num();
    if (auto v = j.get("map_id")) out.map_id = v->str_or();
    if (auto v = j.get("revision")) out.revision = static_cast<int>(v->num());
    if (auto v = j.get("localization_confidence")) out.localization_confidence = v->num();
    if (auto v = j.get("valid")) out.valid = v->boolean();
    long long age_ns = 10'000'000LL;
    if (auto v = j.get("age_ns")) age_ns = static_cast<long long>(v->num());
    out.timestamp_ns = now_ns - age_ns;
    return out;
}

static int run_task_gate_corpus(const char* path, int* counter) {
    std::ifstream in(path);
    CHECK(in.good());
    std::stringstream ss;
    ss << in.rdbuf();
    auto doc_opt = parse_json(ss.str());
    CHECK(doc_opt.has_value());
    const JsonValue& doc = *doc_opt;

    const long long now_ns = static_cast<long long>(doc.get("now_ns")->num());
    const GateLimits base_limits = limits_from(*doc.get("limits"), GateLimits{});
    const SpatialState base_state = state_from(*doc.get("state"), SpatialState{}, now_ns);

    const JsonValue* cases = doc.get("cases");
    CHECK(cases != nullptr && cases->is_arr());
    CHECK(!cases->arr().empty());

    for (const auto& c : cases->arr()) {
        const std::string name = c.get("name")->str_or();
        GateLimits limits = base_limits;
        if (auto lj = c.get("limits")) limits = limits_from(*lj, base_limits);
        SpatialState state = base_state;
        if (auto sj = c.get("state")) state = state_from(*sj, base_state, now_ns);

        const IntentOptions opt{std::nullopt, limits.local_platform, limits.local_robot_id};
        auto intent_v = validate_task_intent(*c.get("intent"), opt);
        if (std::holds_alternative<std::string>(intent_v)) {
            std::fprintf(stderr, "FAIL case=%s validate=%s\n", name.c_str(),
                         std::get<std::string>(intent_v).c_str());
            return 1;
        }
        const TaskIntent& intent = std::get<TaskIntent>(intent_v);

        std::optional<TaskIntent> active;
        if (auto aj = c.get("active_intent")) {
            auto av = validate_task_intent(*aj, opt);
            CHECK(std::holds_alternative<TaskIntent>(av));
            active = std::get<TaskIntent>(av);
        }

        std::map<std::string, std::vector<std::pair<double, double>>> areas;
        if (auto fa = c.get("forbidden_areas")) {
            for (const auto& [area_id, poly_json] : fa->obj()) {
                std::vector<std::pair<double, double>> poly;
                for (const auto& pt : poly_json.arr()) {
                    poly.emplace_back(pt.arr()[0].num(), pt.arr()[1].num());
                }
                areas[area_id] = std::move(poly);
            }
        }

        TaskGate gate(limits);
        if (auto seeds = c.get("seed_records")) {
            for (const auto& s : seeds->arr()) gate.admission().record(s.str_or());
        }

        std::size_t candidates = 0;
        if (auto cc = c.get("candidate_count")) {
            candidates = static_cast<std::size_t>(cc->num());
        }
        const GateResult res = gate.evaluate(
            intent, &state, now_ns, candidates, active ? &*active : nullptr,
            areas.empty() ? nullptr : &areas);

        const JsonValue* expect = c.get("expect");
        CHECK(expect != nullptr);
        const std::string want_decision = expect->get("decision")->str_or();
        const int want_level = static_cast<int>(expect->get("level")->num());
        const std::string want_code = expect->get("code")->str_or();
        std::string want_mode;
        if (auto m = expect->get("degraded_mode")) want_mode = m->str_or();

        if (to_string(res.decision) != want_decision || res.level != want_level ||
            res.code != want_code || res.degraded_mode != want_mode) {
            std::fprintf(stderr,
                         "FAIL case=%s got=%s/L%d/%s/%s want=%s/L%d/%s/%s\n",
                         name.c_str(), to_string(res.decision), res.level, res.code.c_str(),
                         res.degraded_mode.c_str(), want_decision.c_str(), want_level,
                         want_code.c_str(), want_mode.c_str());
            return 1;
        }
        if (counter != nullptr) ++(*counter);
    }
    std::fprintf(stderr, "[task_gate] corpus %zu cases OK\n", cases->arr().size());
    return 0;
}
