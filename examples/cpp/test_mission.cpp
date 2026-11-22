#include "mission/mission_orchestrator.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

using namespace fork_spatial::mission;

int failures = 0;
void expect(bool c, const char* m) {
    if (!c) { std::cerr << "FAIL: " << m << '\n'; ++failures; }
}

OrchestratorConfig fast_cfg() {
    OrchestratorConfig c;
    c.ready_min_messages = 3;
    c.ready_window_s = 2.0;
    c.ready_settle_s = 0.5;
    c.min_execute_before_reach_s = 1.0;
    c.recovery_wait_s = 2.0;
    c.stuck_timeout_s = 10.0;
    c.plan_ready_timeout_s = 2.0;
    c.stuck_redispatch_max = 2;
    c.replan_fail_threshold = 3;
    c.max_plan_fail_recovery = 2;
    return c;
}

RobotState at(double x, double y, double z, double speed, double t_s, bool loc = true) {
    RobotState s;
    s.x = x; s.y = y; s.z = z;
    s.speed_mps = speed;
    s.localization_valid = loc;
    s.stamp_ns = static_cast<fork_spatial::runtime::Timestamp>(t_s * 1e9);
    return s;
}

RobotState at_yaw(double x, double y, double z, double yaw, double speed,
                  double t_s) {
    RobotState s = at(x, y, z, speed, t_s);
    s.yaw = yaw;
    return s;
}

const MissionCommand* last_dispatch(const std::vector<MissionCommand>& cmds) {
    const MissionCommand* out = nullptr;
    for (const auto& c : cmds) {
        if (c.type == CommandType::DispatchSubgoal) out = &c;
    }
    return out;
}

std::vector<MissionCommand> drive(MissionOrchestrator& o, RobotState s, bool ready,
                                  int ticks = 1) {
    std::vector<MissionCommand> out;
    for (int i = 0; i < ticks; ++i) {
        s.stamp_ns += static_cast<int64_t>(i * 0.1e9);
        auto cmd = o.tick(s, ready);
        if (cmd.type != CommandType::None) out.push_back(cmd);
    }
    return out;
}

MissionRequest two_subgoals() {
    MissionRequest r;
    r.subgoals = {{2.0, 0.0, 0.35, "map", 0.45}, {4.0, 0.0, 0.35, "map", 0.45}};
    return r;
}

bool has_cmd(const std::vector<MissionCommand>& cmds, CommandType t) {
    for (const auto& c : cmds) if (c.type == t) return true;
    return false;
}

void test_label_passthrough() {
    MissionOrchestrator o(fast_cfg());
    MissionRequest r;
    r.subgoals = {{2.0, 0.0, 0.35, "map", 0.45, "chair"}};
    expect(o.submit(r), "submit accepted");
    drive(o, at(0, 0, 0.35, 0.0, 0.1), true);
    drive(o, at(0, 0, 0.35, 0.0, 0.3), true);
    drive(o, at(0, 0, 0.35, 0.0, 0.5), true);
    auto cmds = drive(o, at(0, 0, 0.35, 0.0, 1.1), true);
    const MissionCommand* disp = nullptr;
    for (const auto& c : cmds)
        if (c.type == CommandType::DispatchSubgoal) disp = &c;
    expect(disp && disp->target.label == "chair", "dispatch carries label");
    bool saw_dispatch_label = false;
    for (const auto& e : o.events())
        if (e.name == "subgoal_dispatched" && e.detail == "chair") saw_dispatch_label = true;
    expect(saw_dispatch_label, "dispatch event carries label in detail");
    cmds = drive(o, at(2.0, 0, 0.35, 0.05, 3.0), true);
    bool saw_reach_label = false;
    for (const auto& e : o.events())
        if (e.name == "subgoal_reached" && e.detail == "chair") saw_reach_label = true;
    expect(saw_reach_label, "reached event carries label in detail");
}

void test_happy_path() {
    MissionOrchestrator o(fast_cfg());
    expect(o.submit(two_subgoals()), "submit accepted");
    expect(o.status() == MissionStatus::WaitReady, "waits for ready");

    // 热身: 3 条消息在 2s 窗口 + settle 0.5s
    drive(o, at(0, 0, 0.35, 0.0, 0.1), true);
    drive(o, at(0, 0, 0.35, 0.0, 0.3), true);
    drive(o, at(0, 0, 0.35, 0.0, 0.5), true);
    auto cmds = drive(o, at(0, 0, 0.35, 0.0, 1.1), true);
    expect(o.status() == MissionStatus::Running, "running after warmup");
    expect(has_cmd(cmds, CommandType::DispatchSubgoal), "first subgoal dispatched");

    // 到达子目标 0: 距离+速度稳定+最短执行时长
    cmds = drive(o, at(1.9, 0, 0.35, 0.3, 2.0), true);
    expect(o.current_subgoal() == 0, "not arrived: speed too high");
    cmds = drive(o, at(1.9, 0, 0.35, 0.05, 3.2), true);
    expect(o.current_subgoal() == 1 && has_cmd(cmds, CommandType::DispatchSubgoal),
           "subgoal 0 reached -> next dispatched");
    expect(o.status() == MissionStatus::Running, "still running mid-mission");

    // 只有最终子目标到达才 succeeded；StopTrajectory + MissionTerminal 分拍出队
    cmds = drive(o, at(4.0, 0, 0.35, 0.05, 4.6), true, 3);
    expect(o.status() == MissionStatus::Succeeded, "final subgoal -> succeeded");
    expect(has_cmd(cmds, CommandType::StopTrajectory) && has_cmd(cmds, CommandType::MissionTerminal),
           "terminal commands emitted");
}

void test_stuck_then_fail() {
    auto cfg = fast_cfg();
    cfg.max_plan_fail_recovery = 2;
    MissionOrchestrator o(cfg);
    o.submit(two_subgoals());
    for (double t : {0.1, 0.3, 0.5, 1.1}) drive(o, at(0, 0, 0.35, 0.0, t), true);
    expect(o.status() == MissionStatus::Running, "running");

    // 停在原地不走 -> stuck 触发, redispatch(2 次) -> recovering -> 再 redispatch -> fail
    for (int i = 0; i < 200 && o.status() != MissionStatus::Failed; ++i) {
        drive(o, at(0, 0, 0.35, 0.0, 2.0 + i * 1.0), true);
    }
    expect(o.status() == MissionStatus::Failed, "stuck exhausts recovery -> failed");
}

void test_localization_lost_recovery() {
    MissionOrchestrator o(fast_cfg());
    o.submit(two_subgoals());
    for (double t : {0.1, 0.3, 0.5, 1.1}) drive(o, at(0, 0, 0.35, 0.0, t), true);

    auto cmds = drive(o, at(1.0, 0, 0.35, 0.2, 2.0, /*loc=*/false), true);
    expect(has_cmd(cmds, CommandType::StopTrajectory), "loc lost stops old trajectory");
    expect(o.status() == MissionStatus::Recovering, "enters recovering");

    drive(o, at(1.0, 0, 0.35, 0.0, 4.2), true);  // > recovery_wait
    expect(o.status() == MissionStatus::Running, "recovery redispatches");
}

void test_replan_timeout() {
    MissionOrchestrator o(fast_cfg());
    o.submit(two_subgoals());
    for (double t : {0.1, 0.3, 0.5, 1.1}) drive(o, at(0, 0, 0.35, 0.0, t), true);
    // planner_ready=false 超过 plan_ready_timeout -> replan_timeout 累计 -> Recovering
    for (int i = 0; i < 100 && o.status() != MissionStatus::Recovering; ++i) {
        drive(o, at(0.5, 0, 0.35, 0.1, 2.0 + i * 1.0), false);
        o.on_plan_failed();
    }
    expect(o.status() == MissionStatus::Recovering, "replan failures enter recovery");
}

void test_escape_ladder() {
    auto cfg = fast_cfg();
    cfg.escape_step_timeout_s = 5.0;
    MissionOrchestrator o(cfg);
    o.submit(two_subgoals());
    for (double t : {0.1, 0.3, 0.5, 1.1}) drive(o, at(0, 0, 0.35, 0.0, t), true);

    // 耗到 PlanFailHold：stuck → wait → redispatch×2 → stuck → escape 阶梯
    std::vector<MissionCommand> cmds;
    double t = 2.0;
    const MissionCommand* esc = nullptr;
    for (int i = 0; i < 300 && !esc; ++i, t += 0.5) {
        auto out = drive(o, at_yaw(1.0, 0, 0.35, 0.0, 0.0, t), true);
        for (const auto& c : out) {
            cmds.push_back(c);
            if (c.type == CommandType::DispatchSubgoal && c.reason == "escape:backup")
                esc = &cmds.back();
        }
    }
    expect(o.status() == MissionStatus::Recovering, "stuck exhaustion enters hold");
    expect(has_cmd(cmds, CommandType::StopTrajectory), "hold stops trajectory");
    expect(esc && esc->reason == "escape:backup", "backup escape dispatched");
    // yaw=0 → backup = x - 0.8；sidestep = +y 0.35
    expect(esc && std::abs(esc->target.x - 0.2) < 1e-6 &&
                   std::abs(esc->target.y - 0.0) < 1e-6, "backup target geometry");
    expect(esc && std::abs(esc->target.z - 0.35) < 1e-6, "escape keeps body height");

    // 移动到 backup 点附近 -> 位移 > delta -> 进入 sidestep
    cmds.clear();
    MissionCommand side;
    bool got_side = false;
    for (int i = 0; i < 40 && !got_side; ++i, t += 0.5) {
        auto out = drive(o, at_yaw(0.4, 0, 0.35, 0.0, 0.1, t), true);
        if (const auto* d = last_dispatch(out)) { side = *d; got_side = true; }
    }
    expect(got_side && side.reason == "escape:sidestep", "sidestep escape dispatched");
    expect(got_side && std::abs(side.target.x - 0.4) < 1e-6 &&
                   std::abs(side.target.y - 0.35) < 1e-6, "sidestep target geometry");

    // sidestep 到位 -> escape_done -> plan_fail_recovery_done -> Running 重派发
    MissionCommand re;
    bool got_re = false;
    for (int i = 0; i < 40 && !got_re; ++i, t += 0.5) {
        auto out = drive(o, at_yaw(0.4, 0.3, 0.35, 0.0, 0.1, t), true);
        if (const auto* d = last_dispatch(out); d && d->reason.empty()) {
            re = *d; got_re = true;
        }
    }
    expect(o.status() == MissionStatus::Running, "escape ladder back to running");
    expect(got_re, "current subgoal redispatched after escape");
}

void test_pose_corrected() {
    MissionOrchestrator o(fast_cfg());
    o.submit(two_subgoals());
    for (double t : {0.1, 0.3, 0.5, 1.1}) drive(o, at(0, 0, 0.35, 0.0, t), true);
    expect(o.status() == MissionStatus::Running, "running");

    o.on_pose_corrected();  // PGO APPLIED
    auto cmds = drive(o, at(0.3, 0, 0.35, 0.1, 2.0), true);
    expect(has_cmd(cmds, CommandType::DispatchSubgoal),
           "pose correction redispatches current subgoal");
    expect(o.current_subgoal() == 0, "still on same subgoal");
}

void test_cancel_and_empty() {
    MissionOrchestrator o(fast_cfg());
    expect(!o.submit({}), "empty request rejected");
    expect(o.status() == MissionStatus::Failed, "empty request fails terminal");

    MissionOrchestrator o2(fast_cfg());
    o2.submit(two_subgoals());
    drive(o2, at(0, 0, 0.35, 0.0, 0.1), true);
    expect(o2.cancel("user"), "cancel accepted");
    auto cmds = drive(o2, at(0, 0, 0.35, 0.0, 0.2), true);
    expect(has_cmd(cmds, CommandType::StopTrajectory), "cancel stops trajectory");
    cmds = drive(o2, at(0, 0, 0.35, 0.0, 0.3), true);
    expect(has_cmd(cmds, CommandType::MissionTerminal), "cancel emits terminal");
}

}  // namespace

int main() {
    test_label_passthrough();
    test_happy_path();
    test_stuck_then_fail();
    test_localization_lost_recovery();
    test_replan_timeout();
    test_escape_ladder();
    test_pose_corrected();
    test_cancel_and_empty();
    if (failures > 0) return EXIT_FAILURE;
    std::cout << "mission tests passed\n";
    return EXIT_SUCCESS;
}
