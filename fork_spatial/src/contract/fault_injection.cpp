#include "contract/fault_injection.h"

namespace fork_spatial::contract {

namespace {
void clear_relocalization(LocalizationHealth& health) {
    health.candidate_found = false;
    health.alignment_complete = false;
    health.alignment_success = false;
}
}  // namespace

FaultWorld healthy_world() {
    FaultWorld world;
    world.expected = AssetRef{1, 1, "map.f1", 3, 2, "world", "occupancy", 0, 7, false};
    world.header.asset = world.expected;
    world.header.stamp = Stamp{0, 10'000, ClockDomain::Monotonic};
    world.header.health = LocalizationHealth{};
    world.lease = ExecutionLease{"task-1", "edge-a", 1, 7, 0, 10'000, world.expected};
    world.current_epoch = 7;
    world.now_ns = 0;
    return world;
}

StepResult inject(FaultWorld& world, const FaultStep& step) {
    world.now_ns += step.advance_ns;
    switch (step.kind) {
        case FaultKind::Noop:
            break;
        case FaultKind::LocalizationLost:
            world.header.health.mode = LocalizationMode::Hold;
            world.header.health.reason = ReasonCode::LocalizationLost;
            clear_relocalization(world.header.health);
            break;
        case FaultKind::PoseJump:
            world.header.health.mode = LocalizationMode::Relocalizing;
            world.header.health.pose_jump_m = step.amount;
            world.header.health.reason = ReasonCode::LocalizationJump;
            clear_relocalization(world.header.health);
            break;
        case FaultKind::GenerationBump:
            ++world.expected.generation;
            break;
        case FaultKind::LocalizationEpochBump:
            ++world.expected.localization_epoch;
            break;
        case FaultKind::Expired:
            world.header.stamp.valid_until_ns = world.now_ns - 1;
            break;
        case FaultKind::LinkLost:
            world.header.health.reason = ReasonCode::LinkLost;
            break;
        case FaultKind::LeaseExpired:
            world.lease.expiry_ns = world.now_ns - 1;
            break;
        case FaultKind::WrongOwnerEpoch:
            world.current_epoch = world.lease.epoch + 1;
            break;
        case FaultKind::DeadlineMiss:
            if (world.lease.start_deadline_ns == 0) {
                world.lease.start_deadline_ns = world.now_ns;
            }
            world.now_ns = world.lease.start_deadline_ns + 1;
            break;
        case FaultKind::Recover:
            world.header.health.mode = LocalizationMode::Normal3D;
            world.header.health.quality = 1.0;
            world.header.health.pose_jump_m = 0.0;
            world.header.health.reason = ReasonCode::None;
            world.header.health.candidate_found = true;
            world.header.health.alignment_complete = true;
            world.header.health.alignment_success = true;
            world.header.asset = world.expected;
            world.lease.state = world.expected;
            break;
    }
    return StepResult{step.kind, world.now_ns,
                      admit(world.header, world.expected, world.now_ns),
                      dispose(world.lease, world.expected.robot_id,
                              world.current_epoch, world.expected,
                              world.lease.clock, world.now_ns)};
}

std::vector<StepResult> run_scenario(FaultWorld world,
                                     const std::vector<FaultStep>& steps) {
    std::vector<StepResult> results;
    results.reserve(steps.size());
    for (const auto& step : steps) {
        results.push_back(inject(world, step));
    }
    return results;
}

}  // namespace fork_spatial::contract
