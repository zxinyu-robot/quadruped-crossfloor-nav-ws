#include "contract/spatial_state.h"

namespace fork_spatial::contract {

bool same_generation(const AssetRef& a, const AssetRef& b) noexcept {
    return a.robot_id == b.robot_id && a.session_id == b.session_id &&
           a.asset_id == b.asset_id && a.frame_id == b.frame_id &&
           a.generation == b.generation;
}

namespace {
bool same_pose_context(const AssetRef& a, const AssetRef& b) noexcept {
    return same_generation(a, b) &&
           ((!a.pose_anchored && !b.pose_anchored) ||
            a.localization_epoch == b.localization_epoch);
}
}

bool validate(const AssetRef& ref, std::string& error) {
    if (ref.asset_id.empty()) {
        error = "asset_id empty";
        return false;
    }
    if (ref.frame_id.empty()) {
        error = "frame_id empty (no implicit world/map default)";
        return false;
    }
    return true;
}

DeltaDisposition dispose(const DeltaRef& delta, const AssetRef& local) noexcept {
    if (delta.asset_id != local.asset_id || delta.generation != local.generation) {
        return DeltaDisposition::StaleGeneration;
    }
    if (delta.target_revision <= local.revision) {
        return DeltaDisposition::AlreadyApplied;
    }
    if (delta.base_revision == local.revision) {
        return DeltaDisposition::Apply;
    }
    return DeltaDisposition::MissingDeltas;
}

LeaseDisposition dispose(const ExecutionLease& lease, std::uint32_t robot_id,
                         std::uint64_t current_epoch, const AssetRef& local,
                         ClockDomain local_clock, std::int64_t now_ns) noexcept {
    if (lease.robot_id != robot_id) {
        return LeaseDisposition::WrongRobot;
    }
    if (lease.epoch < current_epoch) {
        return LeaseDisposition::StaleEpoch;
    }
    if (lease.clock != local_clock) {
        return LeaseDisposition::ClockMismatch;
    }
    if (lease.start_deadline_ns != 0 && now_ns > lease.start_deadline_ns) {
        return LeaseDisposition::DeadlineMissed;
    }
    if (lease.expiry_ns != 0 && now_ns > lease.expiry_ns) {
        return LeaseDisposition::Expired;
    }
    if (!same_pose_context(lease.state, local)) {
        return LeaseDisposition::StateMismatch;
    }
    return LeaseDisposition::Accept;
}

IdempotencyDisposition dispose_idempotency(const IntentEnvelope& incoming,
                                           const IntentEnvelope* stored) noexcept {
    if (stored == nullptr) {
        return IdempotencyDisposition::New;
    }
    if (incoming.idempotency_key != stored->idempotency_key) {
        return IdempotencyDisposition::New;
    }
    return incoming == *stored ? IdempotencyDisposition::DuplicateSame
                               : IdempotencyDisposition::ConflictDifferent;
}

LeaseDisposition dispose_running(const ExecutionLease& lease, std::uint32_t robot_id,
                                 std::uint64_t current_epoch, const AssetRef& local,
                                 ClockDomain local_clock, std::int64_t started_ns,
                                 std::int64_t now_ns) noexcept {
    const auto admission = dispose(lease, robot_id, current_epoch, local, local_clock, started_ns);
    if (admission != LeaseDisposition::Accept) {
        return admission;
    }
    if (lease.execution_timeout_ns != 0 &&
        now_ns >= started_ns && now_ns - started_ns > lease.execution_timeout_ns) {
        return LeaseDisposition::DeadlineMissed;
    }
    return LeaseDisposition::Accept;
}

LeaseDisposition dispose(const ExecutionLease& lease, std::uint32_t robot_id,
                         std::uint64_t current_epoch, const AssetRef& local,
                         std::int64_t now_ns) noexcept {
    return dispose(lease, robot_id, current_epoch, local, ClockDomain::Monotonic, now_ns);
}

std::string_view name(ReasonCode code) noexcept {
    switch (code) {
        case ReasonCode::None: return "none";
        case ReasonCode::NoWaypoints: return "no_waypoints";
        case ReasonCode::PlanFailed: return "plan_failed";
        case ReasonCode::PlanFailedRepeated: return "plan_failed_repeated";
        case ReasonCode::ReplanTimeout: return "replan_timeout";
        case ReasonCode::Stuck: return "stuck";
        case ReasonCode::LocalizationLost: return "localization_lost";
        case ReasonCode::LocalizationJump: return "localization_jump";
        case ReasonCode::RelocalizationTimeout: return "relocalization_timeout";
        case ReasonCode::FloorTransition: return "floor_transition";
        case ReasonCode::AssetStale: return "asset_stale";
        case ReasonCode::LinkLost: return "link_lost";
        case ReasonCode::SafetyVeto: return "safety_veto";
        case ReasonCode::ResyncRequired: return "resync_required";
        case ReasonCode::Timeout: return "timeout";
        case ReasonCode::Canceled: return "canceled";
        case ReasonCode::LeaseExpired: return "lease_expired";
        case ReasonCode::DeadlineMissed: return "deadline_missed";
        case ReasonCode::Preempted: return "preempted";
        case ReasonCode::IdempotencyConflict: return "idempotency_conflict";
    }
    return "unknown";
}

std::string_view name(LocalizationMode mode) noexcept {
    switch (mode) {
        case LocalizationMode::Normal3D: return "normal_3d";
        case LocalizationMode::Degraded2D: return "degraded_2d";
        case LocalizationMode::Relocalizing: return "relocalizing";
        case LocalizationMode::Verifying: return "verifying";
        case LocalizationMode::Hold: return "hold";
    }
    return "unknown";
}

ExecutionAuthority authority_for(LocalizationMode mode) noexcept {
    switch (mode) {
        case LocalizationMode::Normal3D: return ExecutionAuthority::Full;
        case LocalizationMode::Degraded2D: return ExecutionAuthority::Restricted;
        case LocalizationMode::Relocalizing:
        case LocalizationMode::Verifying: return ExecutionAuthority::Suspended;
        case LocalizationMode::Hold: return ExecutionAuthority::Revoked;
    }
    return ExecutionAuthority::Revoked;
}

AdmitResult admit(const SpatialStateHeader& header, const AssetRef& expected,
                  std::int64_t now_ns) noexcept {
    if (header.asset.robot_id != expected.robot_id ||
        header.asset.session_id != expected.session_id ||
        header.asset.asset_id != expected.asset_id ||
        header.asset.frame_id != expected.frame_id ||
        header.asset.generation != expected.generation) {
        return {AdmitDisposition::Reject, ReasonCode::AssetStale};
    }
    if (header.stamp.expired_at(now_ns)) {
        return {AdmitDisposition::Reject, ReasonCode::AssetStale};
    }
    if (header.asset.localization_epoch != expected.localization_epoch) {
        if (header.asset.pose_anchored || expected.pose_anchored) {
            return {AdmitDisposition::Reverify, ReasonCode::LocalizationLost};
        }
    }
    if (!header.health.usable()) {
        return {header.health.mode == LocalizationMode::Hold
                    ? AdmitDisposition::Hold
                    : AdmitDisposition::Reverify,
                header.health.reason != ReasonCode::None ? header.health.reason
                                                         : ReasonCode::LocalizationLost};
    }
    return {AdmitDisposition::Accept, ReasonCode::None};
}

}  // namespace fork_spatial::contract
