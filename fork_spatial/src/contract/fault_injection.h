#ifndef FORK_SPATIAL_CONTRACT_FAULT_INJECTION_H
#define FORK_SPATIAL_CONTRACT_FAULT_INJECTION_H

#include <cstdint>
#include <vector>

#include "contract/spatial_state.h"

namespace fork_spatial::contract {

// 故障注入框架 v1：把失效用例变成可自动注入的确定性场景。
// 用法：healthy_world() 造一个自洽世界 → 一组 FaultStep 按顺序注入 →
// run_scenario 每步先变异世界，再跑 admit() 与 dispose()，返回每步判决。
// 只驱动契约纯函数，不碰 ROS / DDS / 真机。

enum class FaultKind : std::uint8_t {
    Noop,                   // 只推进时间
    LocalizationLost,       // health.mode = Hold（定位失效），清重定位进度
    PoseJump,               // health.pose_jump_m = amount，mode = Relocalizing
    GenerationBump,         // expected.generation++（换图/楼层切换，全作废）
    LocalizationEpochBump,  // expected.localization_epoch++（定位重置，只作废位姿锚定）
    Expired,                // stamp.valid_until_ns 设为过去（过期 Token/状态）
    LinkLost,               // health.reason = LinkLost（断链；按契约 admit 不受直接影响）
    LeaseExpired,           // lease.expiry_ns 设为过去
    WrongOwnerEpoch,        // current_epoch 超过 lease.epoch（过期执行者）
    DeadlineMiss,           // 推进时间越过 lease.start_deadline_ns
    Recover,                // health 恢复 Normal3D / quality=1 / reason=None，重定位进度置为成功
};

struct FaultWorld {
    SpatialStateHeader header;   // 当前观测（上行状态）
    ExecutionLease lease;        // 当前执行令牌（下行）
    AssetRef expected;           // 本地持有的"当前"状态
    std::uint64_t current_epoch{0};
    std::int64_t now_ns{0};
};

struct FaultStep {
    FaultKind kind{FaultKind::Noop};
    std::int64_t advance_ns{0};  // 注入前先推进的时间
    double amount{0.0};          // 幅度参数（目前仅 PoseJump 用）
};

struct StepResult {
    FaultKind kind{FaultKind::Noop};
    std::int64_t now_ns{0};
    AdmitResult admit;           // admit(header, expected, now)
    LeaseDisposition lease{LeaseDisposition::Accept};  // dispose(lease, ..., now)
};

// 健康基线：robot=1 / session=1 / asset "map.f1" rev=3 gen=2 epoch=7
// frame "world"；健康 Normal3D；lease owner "edge-a" epoch=7
// issued=0 expiry=10000 state=expected；now=0。
[[nodiscard]] FaultWorld healthy_world();

// 注入一个故障并返回该步判决；run_scenario 顺序执行全部步骤。
[[nodiscard]] StepResult inject(FaultWorld& world, const FaultStep& step);
[[nodiscard]] std::vector<StepResult> run_scenario(FaultWorld world,
                                                   const std::vector<FaultStep>& steps);

}  // namespace fork_spatial::contract

#endif
