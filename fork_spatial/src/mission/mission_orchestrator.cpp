#include "mission/mission_orchestrator.h"

#include <algorithm>
#include <cmath>
#include <deque>

namespace fork_spatial::mission {
namespace {

constexpr double kNsPerS = 1e9;
double to_s(Timestamp ns) { return static_cast<double>(ns) / kNsPerS; }

// Recovering 内部的恢复子阶段，对齐 Python ExecPhase：
//   RECOVERY_WAIT → 重派发；PLAN_FAIL_HOLD → ESCAPE(backup → sidestep) → 重派发。
enum class RecoveryPhase { None, Wait, EscapeBackup, EscapeSidestep };

}  // namespace

class MissionOrchestrator::Core {
public:
    explicit Core(OrchestratorConfig c) : cfg(c) {}

    OrchestratorConfig cfg;
    MissionStatus status{MissionStatus::Idle};
    MissionRequest request;
    std::size_t idx{0};
    std::deque<MissionCommand> commands;
    std::vector<MissionEvent> events;

    std::deque<double> ready_ticks;      // WaitReady 窗口内收到状态的时刻
    double ready_since{-1.0};
    double mission_start_s{-1.0};

    bool dispatched{false};
    double dispatch_s{-1.0};
    double best_dist{1e18};
    double last_progress_s{-1.0};
    double plan_deadline_s{-1.0};

    int plan_fail_count{0};
    int redispatch_count{0};
    int recovery_count{0};     // PlanFailHold 进入次数（=Python plan_fail_recovery_count）
    int recovery_step{0};      // 0 = 尚未等过 RecoveryWait
    double recovery_until_s{-1.0};
    std::string recovery_reason;

    RecoveryPhase recovery_phase{RecoveryPhase::None};
    double escape_start_x{0.0}, escape_start_y{0.0};
    double escape_step_since{-1.0};

    double now{-1.0};

    void emit(std::string name, std::string detail = {}) {
        events.push_back({std::move(name), status, idx, std::move(detail),
                          static_cast<Timestamp>(now * kNsPerS)});
    }

    void queue(CommandType t, std::string reason = {}) {
        queue(t, std::move(reason),
              idx < request.subgoals.size() ? request.subgoals[idx] : Subgoal{});
    }

    void queue(CommandType t, std::string reason, const Subgoal& target) {
        commands.push_back({t, idx, target, std::move(reason)});
    }

    void dispatch() {
        dispatched = true;
        dispatch_s = now;
        plan_deadline_s = now + cfg.plan_ready_timeout_s;
        emit("subgoal_dispatched",
             idx < request.subgoals.size() ? request.subgoals[idx].label : "");
        queue(CommandType::DispatchSubgoal);
    }

    // Escape 目标沿正常目标链路下发（Go2: /goal_pose → PCT → SCAN），
    // 语义与 Python _dispatch_escape_goal 一致。
    void dispatch_escape(bool sidestep, const RobotState& st) {
        Subgoal g;
        g.frame_id = idx < request.subgoals.size() ? request.subgoals[idx].frame_id
                                                   : std::string("map");
        g.z = std::max(st.z, 0.3);
        if (sidestep) {
            g.x = st.x + cfg.escape_sidestep_m * (-std::sin(st.yaw));
            g.y = st.y + cfg.escape_sidestep_m * std::cos(st.yaw);
        } else {
            g.x = st.x - cfg.escape_backup_m * std::cos(st.yaw);
            g.y = st.y - cfg.escape_backup_m * std::sin(st.yaw);
        }
        escape_start_x = st.x;
        escape_start_y = st.y;
        escape_step_since = now;
        const char* mode = sidestep ? "sidestep" : "backup";
        emit("escape_step", mode);
        queue(CommandType::DispatchSubgoal,
              std::string("escape:") + mode, g);
    }

    void to_recovering_wait(std::string reason) {
        recovery_reason = std::move(reason);
        status = MissionStatus::Recovering;
        recovery_phase = RecoveryPhase::Wait;
        recovery_until_s = now + cfg.recovery_wait_s;
        dispatched = false;
        emit("recovery_wait", recovery_reason);
        queue(CommandType::RecoveryWait, recovery_reason);
    }

    void to_plan_fail_hold(std::string reason, const RobotState& st) {
        recovery_reason = std::move(reason);
        ++recovery_count;
        if (recovery_count > cfg.max_plan_fail_recovery) {
            status = MissionStatus::Failed;
            emit("mission_terminal", "recovery_budget_exhausted:" + recovery_reason);
            queue(CommandType::StopTrajectory, "failed");
            queue(CommandType::MissionTerminal, "failed:" + recovery_reason);
            return;
        }
        status = MissionStatus::Recovering;
        recovery_phase = RecoveryPhase::EscapeBackup;
        dispatched = false;
        emit("plan_fail_hold", recovery_reason);
        queue(CommandType::StopTrajectory, "plan_fail_hold");
        // 无漂移评估时直接进入 Escape 阶梯；漂移/ICP 判定归适配层。
        emit("escape_start", recovery_reason);
        dispatch_escape(false, st);
    }

    void finish_plan_fail_recovery() {
        recovery_phase = RecoveryPhase::None;
        recovery_step = 0;
        plan_fail_count = 0;
        redispatch_count = 0;
        dispatched = false;
        last_progress_s = now;
        best_dist = 1e18;
        status = MissionStatus::Running;
        emit("plan_fail_recovery_done");
        emit("recovery_done", recovery_reason);
        dispatch();
    }

    void tick_wait_ready(const RobotState& st) {
        if (!st.localization_valid) {
            return;
        }
        ready_ticks.push_back(now);
        while (!ready_ticks.empty() && now - ready_ticks.front() > cfg.ready_window_s) {
            ready_ticks.pop_front();
        }
        if (ready_ticks.size() >= static_cast<std::size_t>(cfg.ready_min_messages)) {
            if (ready_since < 0) {
                ready_since = now;
            }
            if (now - ready_since >= cfg.ready_settle_s) {
                status = MissionStatus::Running;
                mission_start_s = now;
                emit("mission_started");
                dispatch();
            }
        } else {
            ready_since = -1.0;
        }
    }

    void tick_running(const RobotState& st, bool planner_ready) {
        if (request.timeout_s > 0 && now - mission_start_s > request.timeout_s) {
            status = MissionStatus::Failed;
            emit("mission_terminal", "timeout");
            queue(CommandType::StopTrajectory, "timeout");
            queue(CommandType::MissionTerminal, "failed:timeout");
            return;
        }
        if (!st.localization_valid || st.localization_jump) {
            emit("localization_lost");
            queue(CommandType::StopTrajectory, "localization_lost");
            to_recovering_wait("localization_lost");
            return;
        }
        if (planner_ready) {
            plan_deadline_s = now + cfg.plan_ready_timeout_s;
        } else if (now > plan_deadline_s) {
            emit("replan_timeout");
            ++plan_fail_count;
            if (plan_fail_count >= cfg.replan_fail_threshold) {
                plan_fail_count = 0;
                emit("plan_fail_assess");
                to_plan_fail_hold("replan_fail_threshold", st);
            }
            return;
        }

        const auto& sg = request.subgoals[idx];
        const double dist = std::hypot(st.x - sg.x, st.y - sg.y, st.z - sg.z);
        if (dist < best_dist - cfg.progress_dist_delta_m) {
            best_dist = dist;
            last_progress_s = now;
        }
        if (now - last_progress_s > cfg.stuck_timeout_s) {
            emit("stuck");
            if (recovery_step == 0) {
                // Python: 首次 stuck → RecoveryWait 后重派发
                recovery_step = 1;
                to_recovering_wait("stuck");
            } else if (redispatch_count < cfg.stuck_redispatch_max) {
                ++redispatch_count;
                dispatched = false;
                emit("stuck_redispatch");
                dispatch();
            } else {
                to_plan_fail_hold("stuck", st);
            }
            return;
        }

        const bool reached = planner_ready && dispatched &&
            dist <= sg.reach_tolerance_m && st.speed_mps <= cfg.reach_stable_speed_mps &&
            now - dispatch_s >= cfg.min_execute_before_reach_s;
        if (reached) {
            emit("subgoal_reached", sg.label);
            ++idx;
            if (idx >= request.subgoals.size()) {
                status = MissionStatus::Succeeded;
                emit("mission_terminal", "all_subgoals");
                queue(CommandType::StopTrajectory, "succeeded");
                queue(CommandType::MissionTerminal, "succeeded");
            } else {
                dispatched = false;
                best_dist = 1e18;
                last_progress_s = now;
                plan_fail_count = 0;
                redispatch_count = 0;
                recovery_step = 0;
                dispatch();
            }
        }
    }

    void tick_recovering(const RobotState& st) {
        if (recovery_phase == RecoveryPhase::Wait) {
            if (now < recovery_until_s) {
                return;
            }
            recovery_phase = RecoveryPhase::None;
            emit("recovery_wait_done");
            emit("recovery_done", recovery_reason);
            status = MissionStatus::Running;
            dispatch();
            return;
        }
        if (recovery_phase == RecoveryPhase::EscapeBackup ||
            recovery_phase == RecoveryPhase::EscapeSidestep) {
            if (escape_step_since < 0) {
                // on_plan_failed 入口没有位姿，首拍用最新状态补发 escape 目标。
                emit("escape_start", recovery_reason);
                dispatch_escape(recovery_phase == RecoveryPhase::EscapeSidestep, st);
                return;
            }
            const double moved =
                std::hypot(st.x - escape_start_x, st.y - escape_start_y);
            const bool step_done = moved > cfg.progress_dist_delta_m ||
                                   now - escape_step_since > cfg.escape_step_timeout_s;
            if (!step_done) {
                return;
            }
            if (recovery_phase == RecoveryPhase::EscapeBackup) {
                recovery_phase = RecoveryPhase::EscapeSidestep;
                dispatch_escape(true, st);
                return;
            }
            emit("escape_done");
            finish_plan_fail_recovery();
        }
    }
};

MissionOrchestrator::MissionOrchestrator(OrchestratorConfig config) : core_(new Core(config)) {}
MissionOrchestrator::~MissionOrchestrator() { delete core_; }

bool MissionOrchestrator::submit(MissionRequest request) {
    if (core_->status != MissionStatus::Idle) {
        return false;
    }
    if (request.subgoals.empty()) {
        core_->status = MissionStatus::Failed;
        core_->emit("mission_terminal", "no_waypoints");
        core_->queue(CommandType::MissionTerminal, "failed:no_waypoints");
        return false;
    }
    core_->request = std::move(request);
    core_->status = MissionStatus::WaitReady;
    core_->emit("mission_wait_ready");
    return true;
}

bool MissionOrchestrator::cancel(const std::string& reason) {
    if (core_->status == MissionStatus::Succeeded || core_->status == MissionStatus::Failed ||
        core_->status == MissionStatus::Canceled || core_->status == MissionStatus::Idle) {
        return false;
    }
    core_->status = MissionStatus::Canceled;
    core_->emit("mission_terminal", "canceled:" + reason);
    core_->queue(CommandType::StopTrajectory, "canceled:" + reason);
    core_->queue(CommandType::MissionTerminal, "canceled:" + reason);
    return true;
}

MissionCommand MissionOrchestrator::tick(const RobotState& state, bool planner_ready) {
    core_->now = to_s(state.stamp_ns);
    switch (core_->status) {
        case MissionStatus::WaitReady:
            core_->tick_wait_ready(state);
            break;
        case MissionStatus::Running:
            core_->tick_running(state, planner_ready);
            break;
        case MissionStatus::Recovering:
            core_->tick_recovering(state);
            break;
        default:
            break;
    }
    if (!core_->commands.empty()) {
        auto cmd = std::move(core_->commands.front());
        core_->commands.pop_front();
        return cmd;
    }
    return {};
}

void MissionOrchestrator::on_plan_failed() {
    ++core_->plan_fail_count;
    if (core_->status == MissionStatus::Running &&
        core_->plan_fail_count >= core_->cfg.replan_fail_threshold) {
        core_->plan_fail_count = 0;
        core_->emit("plan_fail_assess");
        // on_plan_failed 不携带位姿；Escape 目标在下一拍由 tick_recovering
        // 依赖最新 RobotState 派发——这里先用一个占位进入 EscapeBackup，
        // 由 tick_recovering 检测 escape_step_since<0 时补发。
        core_->status = MissionStatus::Recovering;
        core_->recovery_reason = "replan_fail_threshold";
        ++core_->recovery_count;
        if (core_->recovery_count > core_->cfg.max_plan_fail_recovery) {
            core_->status = MissionStatus::Failed;
            core_->emit("mission_terminal", "recovery_budget_exhausted:" +
                                              core_->recovery_reason);
            core_->queue(CommandType::StopTrajectory, "failed");
            core_->queue(CommandType::MissionTerminal,
                         "failed:" + core_->recovery_reason);
            return;
        }
        core_->recovery_phase = RecoveryPhase::EscapeBackup;
        core_->escape_step_since = -1.0;
        core_->dispatched = false;
        core_->emit("plan_fail_hold", core_->recovery_reason);
        core_->queue(CommandType::StopTrajectory, "plan_fail_hold");
    }
}

void MissionOrchestrator::on_planner_ready() { core_->plan_deadline_s = core_->now + core_->cfg.plan_ready_timeout_s; }

void MissionOrchestrator::on_pose_corrected() {
    if (core_->status != MissionStatus::Running &&
        core_->status != MissionStatus::Recovering) {
        return;
    }
    // PGO 纠偏后旧轨迹失效：清恢复子阶段、重派发当前子目标。
    core_->recovery_phase = RecoveryPhase::None;
    core_->status = MissionStatus::Running;
    core_->dispatched = false;
    core_->emit("pose_correction_redispatch");
    core_->dispatch();
}

MissionStatus MissionOrchestrator::status() const { return core_->status; }
std::size_t MissionOrchestrator::current_subgoal() const { return core_->idx; }
const std::vector<MissionEvent>& MissionOrchestrator::events() const { return core_->events; }

}  // namespace fork_spatial::mission
