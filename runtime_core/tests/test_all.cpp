// 端侧闭环核心测试（CTest 入口）。断言失败即非零退出。
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <variant>

#include "legged_nav/admission.hpp"
#include "legged_nav/executor.hpp"
#include "legged_nav/feedback.hpp"
#include "legged_nav/json.hpp"
#include "legged_nav/runtime.hpp"
#include "legged_nav/spatial_context.hpp"
#include "legged_nav/spatial_state.hpp"
#include "legged_nav/task_gate.hpp"
#include "legged_nav/task_intent.hpp"
#include "legged_nav/transport.hpp"
#include "legged_nav/vision_loop.hpp"
#include "legged_nav/vlm_adapter.hpp"

using namespace legged_nav;

static int passed = 0;
#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); \
            return 1;                                                           \
        }                                                                       \
        ++passed;                                                               \
    } while (0)

static std::string err_of(const std::variant<TaskIntent, std::string>& v) {
    return std::holds_alternative<std::string>(v) ? std::get<std::string>(v) : "";
}
static bool has_err(const std::variant<TaskIntent, std::string>& v, const char* prefix) {
    return err_of(v).rfind(prefix, 0) == 0;
}

#include "task_gate_cases.inc"

static SpatialState good_state(long long ts = now_ns()) {
    SpatialState s;
    s.map_id = "map_01";
    s.revision = 3;
    s.localization_confidence = 0.9;
    s.valid = true;
    s.timestamp_ns = ts;
    return s;
}

static std::string intent_json(const char* task_id) {
    std::string s = R"({"schema":"motionslam.task_intent.v1","task_id":")";
    s += task_id;
    s += R"(","robot":{"platform":"unitree_go2_edu","sn":"go2_001"},
            "world_ref":{"map_id":"map_01","revision":3},
            "skills":[{"name":"Navigate","goal":{"x":2.0,"y":0.5,"z":0.35}},
                      {"name":"ReturnHome"}],
            "policy":{"deadline_s":120.0},
            "execution":{"priority":0,"preemptible":false}})";
    return s;
}

static std::string intent_with(const char* extra) {
    std::string s = R"({"schema":"motionslam.task_intent.v1","task_id":"t1",
            "robot":{"platform":"unitree_go2_edu","sn":"go2_001"},
            "world_ref":{"map_id":"map_01","revision":3},)";
    s += extra;
    s += "}";
    return s;
}

int main() {
    // ---------- task gate corpus（与 Python 同语料） ----------
    CHECK(run_task_gate_corpus(TASK_GATE_CASES, &passed) == 0);
    // ---------- json ----------
    auto j = parse_json(R"({"a":1,"b":[true,"x"],"c":{"d":null}})");
    CHECK(j && j->get("a")->num() == 1.0 && j->get("b")->arr().size() == 2);
    CHECK(!parse_json("{bad"));

    // ---------- task_intent ----------
    auto ok = validate_task_intent_text(intent_json("task-001"));
    CHECK(std::holds_alternative<TaskIntent>(ok));
    CHECK(std::get<TaskIntent>(ok).skills[0].goal->x == 2.0);

    CHECK(has_err(validate_task_intent_text(R"({"schema":"other.v9"})"), "REJECT_SCHEMA"));
    CHECK(has_err(validate_task_intent_text(intent_with(R"("task_id":"")")),
                  "REJECT_MISSING_TASK_ID"));
    CHECK(has_err(validate_task_intent_text(
                      R"({"schema":"motionslam.task_intent.v1","task_id":"t",
                          "robot":{"platform":"a1","sn":"x"},
                          "world_ref":{"map_id":"m","revision":0},
                          "skills":[{"name":"Navigate","goal":{"x":1,"y":0,"z":0}}]})"),
                  "REJECT_ROBOT_PLATFORM"));
    CHECK(
        has_err(validate_task_intent_text(intent_with(
                    R"("skills":[{"name":"Navigate","goal":{"x":1,"y":0,"z":0,"cmd_vel":0.5}}])")),
                "REJECT_FORBIDDEN_CONTROL"));
    CHECK(has_err(validate_task_intent_text(intent_with(R"("skills":[{"name":"Fly"}])")),
                  "REJECT_SKILL_UNSUPPORTED"));
    CHECK(has_err(validate_task_intent_text(
                      intent_with(R"("policy":{"deadline_s":-1},"skills":[{"name":"Stand"}])")),
                  "REJECT_BAD_DEADLINE"));
    CHECK(has_err(validate_task_intent_text(intent_with(
                      R"("navigation":{"forbidden_areas":[1]},
                        "skills":[{"name":"Stand"}])")),
                  "REJECT_BAD_FORBIDDEN_AREAS"));
    CHECK(has_err(validate_task_intent_text(intent_with(R"("skills":[{"name":"Navigate"}])")),
                  "REJECT_SKILL_0_MISSING_GOAL"));

    // robot mismatch
    IntentOptions opt;
    opt.local_robot_id = "go2_002";
    CHECK(has_err(validate_task_intent_text(intent_json("t-m"), opt), "REJECT_ROBOT_MISMATCH"));

    // forbidden areas
    auto intent = std::get<TaskIntent>(validate_task_intent_text(intent_with(
        R"("skills":[{"name":"Navigate","goal":{"x":2.0,"y":0.5,"z":0.0}}],
           "navigation":{"forbidden_areas":["zone_A"]})")));
    std::map<std::string, std::vector<std::pair<double, double>>> areas = {
        {"zone_A", {{1, 0}, {3, 0}, {3, 1}, {1, 1}}}};
    auto fa = check_forbidden_areas(intent, areas);
    CHECK(fa && fa->find("REJECT_GOAL_IN_FORBIDDEN_AREA") != std::string::npos);
    auto ua = check_forbidden_areas(intent, {});
    CHECK(ua && ua->find("REJECT_UNKNOWN_AREA") != std::string::npos);

    // ---------- admission ----------
    TaskAdmission adm;
    auto i1 = std::get<TaskIntent>(validate_task_intent_text(intent_with(
        R"("skills":[{"name":"Stand"}],"execution":{"preemptible":true,"priority":1})")));
    auto i2 = std::get<TaskIntent>(validate_task_intent_text(
        intent_with(R"("task_id":"t2","skills":[{"name":"Stand"}],"execution":{"priority":5})")));
    auto i3 = std::get<TaskIntent>(validate_task_intent_text(
        intent_with(R"("task_id":"t3","skills":[{"name":"Stand"}],"execution":{"priority":0})")));
    CHECK(std::get<AdmitDecision>(adm.decide(i2, &i1)) == AdmitDecision::PREEMPT);
    CHECK(std::get<std::string>(adm.decide(i3, &i1)) == "REJECT_BUSY");
    CHECK(std::get<AdmitDecision>(adm.decide(i1, &i1)) == AdmitDecision::IDEMPOTENT);

    // ---------- runtime closed loop ----------
    SimForwarder fwd(good_state());
    SpatialState gs = good_state();
    IntentOptions ropt;
    ropt.local_platform = "unitree_go2_edu";
    ropt.local_robot_id = "go2_001";
    TaskRuntime rt(fwd, ropt);
    auto fb = rt.submit(intent_json("t1"), &gs);
    CHECK(fb["event"] == "succeeded");
    CHECK(rt.events()[0].at("event") == "accepted" && rt.events()[1].at("event") == "started");
    CHECK(fwd.steps().size() == 2 && fwd.steps()[0].first == "navigate_to");
    CHECK(fwd.state().x == 0.0);  // ReturnHome 归位

    // reject: no dispatch
    SimForwarder fwd2(good_state());
    TaskRuntime rt2(fwd2);
    auto fb2 = rt2.submit(std::string(R"({"schema":"bad"})"));
    CHECK(fb2["event"] == "rejected" && fwd2.steps().empty());

    // expired state → holding
    SimForwarder fwd3(good_state());
    TaskRuntime rt3(fwd3, {}, {}, StateGate(1000));
    auto stale = good_state(now_ns() - 5000);
    auto fb3 = rt3.submit(intent_json("t-h"), &stale);
    CHECK(fb3["event"] == "holding" && fb3["detail"].find("state_expired") != std::string::npos);

    // revision mismatch / low confidence
    SimForwarder fwd4(good_state());
    TaskRuntime rt4(fwd4);
    auto rev9 = good_state();
    rev9.revision = 9;
    CHECK(rt4.submit(intent_json("t-r"), &rev9)["detail"].find("revision_mismatch") !=
          std::string::npos);
    SimForwarder fwd5(good_state());
    TaskRuntime rt5(fwd5);
    auto low = good_state();
    low.localization_confidence = 0.1;
    CHECK(rt5.submit(intent_json("t-l"), &low)["detail"].find("low_confidence") !=
          std::string::npos);

    // duplicate
    SimForwarder fwd6(good_state());
    TaskRuntime rt6(fwd6);
    rt6.submit(intent_json("t-d"), &gs);
    CHECK(rt6.submit(intent_json("t-d"), &gs)["detail"].find("REJECT_DUPLICATE_TASK_ID") !=
          std::string::npos);

    // execution failure → abort + failed
    SimForwarder fwd7(good_state());
    fwd7.fail_on("navigate_to");
    TaskRuntime rt7(fwd7);
    auto fb7 = rt7.submit(intent_json("t-f"), &gs);
    CHECK(fb7["event"] == "failed" &&
          fb7["detail"].find("step_failed:navigate_to") != std::string::npos);
    CHECK(fwd7.steps().back().first.find("abort") == 0);

    // forbidden area at runtime
    SimForwarder fwd8(good_state());
    TaskRuntime rt8(fwd8, {}, areas);
    auto fba = rt8.submit(intent_with(
                              R"("skills":[{"name":"Navigate","goal":{"x":2.0,"y":0.5,"z":0.0}}],
           "navigation":{"forbidden_areas":["zone_A"]})"),
                          &gs);
    CHECK(fba["state"] == "rejected" && fwd8.steps().empty());

    // ---------- transport ----------
    InProcessTransport tp;
    CHECK(tp.publish("nope/x", JsonValue()) &&
          tp.publish("nope/x", JsonValue())->find("REJECT_UNKNOWN_CHANNEL") == 0);
    auto bad = parse_json(R"({"kind":"x","params":{"cmd_vel":1.0}})");
    CHECK(tp.publish("motion/action_step", *bad) == "REJECT_FORBIDDEN_CONTROL:cmd_vel");
    int got = 0;
    tp.subscribe("task/intent", [&](const Message&) { ++got; });
    auto good = parse_json(R"({"task_id":"x"})");
    CHECK(!tp.publish("task/intent", *good) && got == 1 && tp.log().size() == 1);
    CHECK(channels().at("task/intent").qos == QoS::RELIABLE);

    // ---------- spatial context ----------
    SpatialStateStore store;
    store.update("pose", &gs, nullptr, 1000, now_ns());
    store.update("pose2", &gs, nullptr, 1'000'000'000LL, now_ns());
    auto live = store.entries(now_ns() + 5000);
    CHECK(live.count("pose") == 0 && live.count("pose2") == 1);

    StreamContext sc(3, 0.3);
    CHECK(sc.push(VisionToken{"a", "kitchen", {0, 0, 0}, 0}));
    CHECK(!sc.push(VisionToken{"b", "kitchen", {0.1, 0.05, 0}, 0}));  // 去重
    CHECK(sc.push(VisionToken{"c", "corridor", {2.0, 0, 0}, 0}));
    for (int i = 0; i < 4; ++i)
        sc.push(VisionToken{"f" + std::to_string(i), "s" + std::to_string(i), {3.0 + i, 0, 0}, 0});
    auto st = sc.stats();
    CHECK(st["fast_len"] == 3 && st["kv_cached"] == 6 && st["dedup_dropped"] == 1);
    CHECK(st["slow_compressed"] == 3);

    // ---------- vision loop ----------
    SimForwarder fwd9(good_state());
    TaskRuntime rt9(fwd9, ropt);
    SemanticObservation obs;
    obs.scene_class = "kitchen";
    obs.confidence = 0.9;
    obs.target_visible = true;
    StubVLMBackend stub(obs);
    VisionContext vctx;
    vctx.task_prompt = "导航到厨房";
    vctx.platform = "unitree_go2_edu";
    vctx.robot_id = "go2_001";
    vctx.map_id = "map_01";
    vctx.revision = 3;
    vctx.goal = IntentGoal{2.0, 0.5, 0.35, "world"};
    StreamContext stream(3, 0.3);
    VisionLoop loop(stub, rt9, vctx, &stream);
    auto dec = loop.step("frame-bytes", "f0", 0, std::tuple{0.0, 0.0, 0.0});
    CHECK(dec.reason == "target_visible" && dec.feedback &&
          (*dec.feedback)["event"] == "succeeded");
    CHECK(fwd9.state().x == 2.0);
    CHECK(stream.stats()["fast_len"] == 1);

    // hazard → Stop
    SemanticObservation hz;
    hz.confidence = 0.9;
    hz.hazards = {"dynamic_obstacle"};
    StubVLMBackend stub2(hz);
    SimForwarder fwd10(good_state());
    TaskRuntime rt10(fwd10, ropt);
    VisionLoop loop2(stub2, rt10, vctx);
    auto dec2 = loop2.step("f", "f1", 1);
    CHECK(dec2.reason.find("hazard:") == 0 && (*dec2.feedback)["event"] == "succeeded");
    CHECK(fwd10.steps()[0].first == "stop");

    // low confidence → no intent
    SemanticObservation low_obs;
    low_obs.confidence = 0.2;
    StubVLMBackend stub3(low_obs);
    VisionLoop loop3(stub3, rt9, vctx);
    auto dec3 = loop3.step("f", "f2", 2);
    CHECK(!dec3.intent_json && dec3.reason.find("low_observation_confidence") == 0);

    // QwenVLBackend 构造即抛（权重/算力为部署侧）
    bool threw = false;
    try {
        QwenVLBackend q("Qwen/Qwen2.5-VL-7B-Instruct");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);

    // 轴角 → yaw：绕 z 轴转 90°、零向量归零
    CHECK(std::abs(yawFromAxisAngle(0.0, 0.0, M_PI / 2) - 90.0) < 0.5);
    CHECK(yawFromAxisAngle(0.0, 0.0, 0.0) == 0.0);

    // 深度门控：走廊近场阻挡 / 远场放行 / 无效保守
    std::vector<std::string> hazard_ids{"cup", "bottle"};
    std::vector<double> near_d(100, 3000.0);
    for (int r = 0; r < 10; ++r)
        for (int c = 4; c < 6; ++c) near_d[r * 10 + c] = 500.0;
    std::vector<double> far_d(100, 3000.0);
    CHECK(blockingHazards(near_d, 10, 10, hazard_ids).size() == 2);
    CHECK(blockingHazards(far_d, 10, 10, hazard_ids).empty());
    CHECK(blockingHazards({}, 0, 0, hazard_ids).size() == 2);

    // ---------- feedback（progress 展平，与 Python build_feedback 同构） ----------
    auto fb_flat = build_feedback("t-flat", "succeeded", "succeeded",
                                  {{"skill_index", "0"}, {"skill", "Navigate"}},
                                  "", 12345);
    CHECK(fb_flat.count("progress.skill_index") == 1);
    CHECK(fb_flat.count("progress.skill") == 1);
    CHECK(fb_flat.count("progress") == 0);
    CHECK(fb_flat.at("progress.skill_index") == "0");

    // ---------- 契约对照语料（与 Python tests/test_contract_parity.py 同一文件） ----------
    {
        std::ifstream corpus(LEGGED_NAV_CONTRACT_CASES);
        CHECK(corpus.good());
        std::stringstream buffer;
        buffer << corpus.rdbuf();
        auto doc = parse_json(buffer.str());
        CHECK(doc.has_value() && doc->is_obj());
        const JsonValue* cases = doc->get("cases");
        CHECK(cases != nullptr && cases->is_arr() && !cases->arr().empty());
        for (const auto& c : cases->arr()) {
            const JsonValue* name = c.get("name");
            const JsonValue* intent_text = c.get("intent");
            const JsonValue* expect = c.get("expect");
            CHECK(name != nullptr && name->is_str());
            CHECK(intent_text != nullptr && intent_text->is_str());
            CHECK(expect != nullptr && expect->is_str());
            auto verdict = validate_task_intent_text(intent_text->str());
            std::string got = std::holds_alternative<std::string>(verdict)
                                  ? std::get<std::string>(verdict)
                                  : std::string("OK");
            if (got != expect->str()) {
                std::fprintf(stderr, "contract case %s: got %s, want %s\n", name->str().c_str(),
                             got.c_str(), expect->str().c_str());
                return 1;
            }
            ++passed;
        }
    }

    std::printf("all %d checks passed\n", passed);
    return 0;
}
