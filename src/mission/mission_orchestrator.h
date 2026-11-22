#ifndef FORK_SPATIAL_MISSION_MISSION_ORCHESTRATOR_H
#define FORK_SPATIAL_MISSION_MISSION_ORCHESTRATOR_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "contract/spatial_state.h"
#include "runtime/model.h"

namespace fork_spatial::mission {

using runtime::Timestamp;

// 任务编排层的机器人无关语义：从 MotionSLAM_ws 的 Python BT 编排器
// （demo_scan_bt_orchestrator.py）提炼，迁到 C++ 后 Python 版降为兜底。
// 本层只编排"子目标派发 / 到达判定 / 失败恢复 / 终态"，
// 不做几何规划与设备控制——那是 Go2 等适配层的事。

enum class MissionStatus {
    Idle,
    WaitReady,    // 状态源未就绪（云端点云/定位未热身）
    Running,
    Recovering,
    Succeeded,
    Failed,
    Canceled,
};

// 原因码统一到 contract::ReasonCode；本编排器仅为参考执行器 / 测试夹具，
// 真机执行权在 MotionSLAM 的 BT（见 docs/contracts/spatial_state.md）。
using FailureCode = contract::ReasonCode;

enum class CommandType {
    None,
    DispatchSubgoal,   // 适配层转成机器人局部规划请求（Go2: SCAN /initial_path）
    StopTrajectory,    // 停掉当前轨迹（定位失效、超时时）
    SlowModeOn,
    SlowModeOff,
    RecoveryWait,      // 暂停 recovery_wait_s 再重试
    MissionTerminal,   // 任务结束（含 reason）
};

struct Subgoal {
    double x{}, y{}, z{};
    std::string frame_id{"map"};
    double reach_tolerance_m{0.45};
    std::string label;  // 语义目标标签（如物体类别/名称），仅随事件透传，不影响度量判定
};

struct MissionRequest {
    std::vector<Subgoal> subgoals;
    double timeout_s{0.0};  // 0 = 不限
};

// 适配层每个控制节拍喂进来的最小机器人状态。
struct RobotState {
    double x{}, y{}, z{};
    double yaw{};                   // 机体朝向（escape 目标计算需要）
    double speed_mps{0.0};          // 建议喂滤波后速度（真机 odom twist 噪声大）
    bool localization_valid{true};
    bool localization_jump{false};  // 漂移跳变（里程计跳变超阈值）
    Timestamp stamp_ns{};
};

struct MissionCommand {
    CommandType type{CommandType::None};
    std::size_t subgoal_index{};
    Subgoal target;
    std::string reason;
};

struct MissionEvent {
    std::string name;  // subgoal_dispatched / subgoal_reached / escape_* /
                       // plan_fail_* / mission_terminal ...
    MissionStatus status{MissionStatus::Idle};
    std::size_t subgoal_index{};
    std::string detail;
    Timestamp stamp_ns{};
};

struct OrchestratorConfig {
    double subgoal_reach_m{0.45};
    double reach_stable_speed_mps{0.08};   // 到达还要求速度稳定（新加的判定）
    double min_execute_before_reach_s{1.5};
    double progress_dist_delta_m{0.12};
    double stuck_timeout_s{60.0};
    int stuck_redispatch_max{3};
    int replan_fail_threshold{8};
    double plan_ready_timeout_s{8.0};
    double ready_min_messages{10.0};       // 状态源热身：窗口内最小消息数
    double ready_window_s{4.0};
    double ready_settle_s{3.0};
    double recovery_wait_s{2.5};
    int max_plan_fail_recovery{4};
    double escape_backup_m{0.80};
    double escape_sidestep_m{0.35};
    double escape_step_timeout_s{8.0};
    double tick_hz{10.0};
};

// 用法（适配层，以 Go2 为例）：
//   orchestrator.submit(MissionRequest{waypoints});
//   每节拍: auto cmd = orchestrator.tick(robot_state, planner_ready);
//   if cmd.type == DispatchSubgoal -> 转成 SCAN /initial_path
//   if cmd.type == MissionTerminal -> 回报 mission_terminal
//   恢复辅助: orchestrator.on_plan_failed() / on_planner_ready()
// 规则：
//   - 到达判定 = 距离 ≤ tolerance 且速度 ≤ reach_stable_speed 且已执行
//     满 min_execute_before_reach_s；不得仅凭位姿更新判定。
//   - 恢复阶梯（对齐 Python 编排器）：stuck → RecoveryWait → 重派发
//     （≤stuck_redispatch_max）→ PlanFailHold → Escape（backup →
//     sidestep，沿正常目标链路执行，步完成 = 位移超 progress_delta
//     或 escape_step_timeout）→ 重派发；PlanFailHold 次数超
//     max_plan_fail_recovery -> Failed。定位失效 -> StopTrajectory。
//   - 只有最终子目标到达才 Succeeded；中间子目标到达继续派发。
class MissionOrchestrator {
public:
    explicit MissionOrchestrator(OrchestratorConfig config = {});
    ~MissionOrchestrator();
    MissionOrchestrator(const MissionOrchestrator&) = delete;
    MissionOrchestrator& operator=(const MissionOrchestrator&) = delete;

    bool submit(MissionRequest request);
    bool cancel(const std::string& reason);
    MissionCommand tick(const RobotState& state, bool planner_ready);
    void on_plan_failed();
    void on_planner_ready();
    // 位姿图优化/重定位收敛后的无感纠偏：重新派发当前子目标。
    // Go2 侧对应 /lio/backend/pgo_state OPTIMIZING→APPLIED。
    void on_pose_corrected();

    MissionStatus status() const;
    std::size_t current_subgoal() const;
    const std::vector<MissionEvent>& events() const;

private:
    class Core;
    Core* core_;
};

}  // namespace fork_spatial::mission

#endif
