// 空间状态契约失效用例：定位重置 / 跨楼层 / 过期 Token / 地图版本不一致 /
// 规划失败 / 安全否决 / 执行超时。每项验证 Runtime 停止使用旧证据。
#include "contract/fault_injection.h"
#include "contract/spatial_state.h"
#include "cognition/edge_policy.h"
#include "cognition/semantic_token.h"
#include "mission/mission_orchestrator.h"

#include <cstdlib>
#include <type_traits>
#include <iostream>

namespace {

using namespace fork_spatial::contract;
namespace cog = fork_spatial::cognition;

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

AssetRef floor(const char* asset, std::uint64_t revision, std::uint64_t generation) {
    return AssetRef{1, 7, asset, revision, generation, "map"};
}

cog::SemanticToken token(std::uint64_t id, const AssetRef& ref, std::int64_t ttl_ns) {
    cog::SemanticToken t;
    t.token_id = id;
    t.kind = cog::TokenKind::Frontier;
    t.resource = {ref.robot_id, ref.session_id, ref.asset_id, ref.revision, ref.frame_id,
                  ref.generation};
    t.stamp_ns = 1'000;
    t.ttl_ns = ttl_ns;
    t.confidence = 0.9;
    return t;
}

void case_localization_reset() {
    const auto gen1 = floor("b1/f2/map", 3, 1);
    const auto gen2 = floor("b1/f2/map", 3, 2);
    cog::TokenStore store;
    expect(store.upsert(token(1, gen1, 60'000'000'000)) == cog::UpsertCode::Inserted,
           "reset: token accepted");
    expect(store.upsert(token(2, gen2, 60'000'000'000)) == cog::UpsertCode::Inserted,
           "reset: new-gen token accepted");
    expect(store.invalidate_generation(gen2) == 1, "reset: TTL 未到的旧代 Token 必须作废");
    expect(store.valid_tokens().size() == 1 && store.valid_tokens()[0].token_id == 2,
           "reset: only current generation survives");
    expect(store.invalidate_generation(gen2) == 0, "reset: idempotent");

    SpatialStateHeader old_state{gen1, Stamp{1'000, 0}, {}};
    expect(admit(old_state, gen2, 2'000) == ReasonCode::AssetStale,
           "reset: old-generation state rejected as AssetStale");
}

void case_floor_transition() {
    const auto f2 = floor("b1/f2/map", 1, 5);
    const auto f3 = floor("b1/f3/map", 1, 6);
    expect(!same_generation(f2, f3), "floor: different asset is never same generation");
    LocalizationHealth h;
    h.mode = LocalizationMode::Hold;
    h.reason = ReasonCode::FloorTransition;
    SpatialStateHeader during{f3, Stamp{1'000, 0}, h};
    expect(admit(during, f3, 1'500) == ReasonCode::FloorTransition,
           "floor: Hold during transition surfaces FloorTransition");
    expect(h.authority() == ExecutionAuthority::Revoked, "floor: Hold revokes execution");
    expect(authority_for(LocalizationMode::Relocalizing) == ExecutionAuthority::Suspended &&
               authority_for(LocalizationMode::Verifying) == ExecutionAuthority::Suspended,
           "relocalizing/verifying only suspend, task is kept");
    expect(authority_for(LocalizationMode::Degraded2D) == ExecutionAuthority::Restricted,
           "degraded 2d keeps restricted authority");
}

void case_expired_token() {
    const auto ref = floor("b1/f2/map", 1, 1);
    Stamp stamp{1'000, 5'000};
    expect(!stamp.expired_at(4'999) && stamp.expired_at(5'001), "expiry boundary");
    SpatialStateHeader h{ref, stamp, {}};
    expect(admit(h, ref, 4'000) == ReasonCode::None, "fresh state admitted");
    expect(admit(h, ref, 6'000) == ReasonCode::AssetStale, "expired state rejected");
    Stamp forever{1'000, 0};
    expect(!forever.expired_at(1'000'000'000'000), "valid_until=0 never expires");
}

void case_map_revision_mismatch() {
    const auto r3 = floor("b1/f2/map", 3, 1);
    const auto r4 = floor("b1/f2/map", 4, 1);
    // revision 变化不等于代数切换：局部更新仍可继续（MotionSLAM 规划 §5.2）。
    expect(same_generation(r3, r4), "revision bump alone keeps generation");
    SpatialStateHeader h{r3, Stamp{1'000, 0}, {}};
    expect(admit(h, r4, 2'000) == ReasonCode::None, "older revision still admitted");

    // 但决策引用的版本与实际执行的版本必须可对账。
    cog::PolicyDecision d;
    d.policy_version = 9;
    d.context = r3;
    cog::DecisionOutcome o;
    o.decision = d;
    o.executed_policy_version = 9;
    o.executed_context = r4;
    expect(o.executed_context != o.decision.context, "evidence chain records revision drift");

    std::string err;
    AssetRef no_frame = r3;
    no_frame.frame_id.clear();
    expect(!validate(no_frame, err) && !err.empty(), "empty frame_id rejected");
    expect(validate(r3, err), "complete ref valid");
}

void case_plan_failed() {
    using fork_spatial::mission::FailureCode;
    static_assert(std::is_same_v<FailureCode, ReasonCode>, "mission FailureCode is ReasonCode");
    expect(name(FailureCode::PlanFailedRepeated) == "plan_failed_repeated", "stable name");
    expect(name(ReasonCode::PlanFailed) == "plan_failed", "single-shot plan failure name");
    cog::PolicyDecision d;
    d.fallback = true;
    d.degrade_reason = ReasonCode::PlanFailedRepeated;
    expect(d.fallback && d.degrade_reason != ReasonCode::None, "fallback carries reason");
}

void case_safety_veto() {
    cog::DecisionOutcome o;
    o.vetoed = true;
    o.failure_code = ReasonCode::SafetyVeto;
    expect(name(o.failure_code) == "safety_veto", "veto name");
    expect(!o.succeeded && o.vetoed, "veto is not success");
}

void case_timeout() {
    expect(name(ReasonCode::Timeout) == "timeout", "timeout name");
    expect(name(ReasonCode::RelocalizationTimeout) == "relocalization_timeout", "reloc timeout");
    LocalizationHealth h;
    h.mode = LocalizationMode::Relocalizing;
    h.reason = ReasonCode::None;
    SpatialStateHeader s{floor("b1/f2/map", 1, 1), Stamp{1'000, 0}, h};
    expect(admit(s, floor("b1/f2/map", 1, 1), 2'000) == ReasonCode::LocalizationLost,
           "unusable mode without reason defaults to LocalizationLost");
}

void case_delta_out_of_order() {
    const auto local = floor("b1/f2/map", 42, 1);
    DeltaRef d{"b1/f2/map", 42, 43, 1, "token_stream", 0};
    expect(dispose(d, local) == DeltaDisposition::Apply, "delta: base matches → apply");
    expect(dispose(d, floor("b1/f2/map", 40, 1)) == DeltaDisposition::MissingDeltas,
           "delta: local at 40 must not apply delta(base=42)");
    expect(dispose(d, floor("b1/f2/map", 43, 1)) == DeltaDisposition::AlreadyApplied,
           "delta: duplicate delivery is a no-op");
    expect(dispose(d, floor("b1/f2/map", 42, 2)) == DeltaDisposition::StaleGeneration,
           "delta: generation changed → discard and resync");
    expect(dispose(d, floor("b1/f3/map", 42, 1)) == DeltaDisposition::StaleGeneration,
           "delta: other asset → discard");
    expect(name(ReasonCode::ResyncRequired) == "resync_required", "resync name");
}

void case_stale_executor() {
    const auto local = floor("b1/f2/map", 5, 3);
    ExecutionLease lease{"task_1", "edge_a", 1, 7, 1'000, 5'000, local};
    expect(dispose(lease, 1, 7, local, 2'000) == LeaseDisposition::Accept, "lease: current executor ok");
    expect(dispose(lease, 1, 8, local, 2'000) == LeaseDisposition::StaleEpoch,
           "lease: edge_a dropped, edge_b took over with epoch 8 → old command rejected");
    expect(dispose(lease, 1, 7, local, 6'000) == LeaseDisposition::Expired, "lease: expired");
    expect(dispose(lease, 2, 7, local, 2'000) == LeaseDisposition::WrongRobot, "lease: other robot");
    expect(dispose(lease, 1, 7, floor("b1/f2/map", 5, 4), 2'000) == LeaseDisposition::StateMismatch,
           "lease: issued on old generation → rejected");
    expect(dispose(lease, 1, 7, local, ClockDomain::WallUtc, 2'000) == LeaseDisposition::ClockMismatch,
           "lease: mismatched clock domain → rejected");
    lease.start_deadline_ns = 1'500;
    expect(dispose(lease, 1, 7, local, 2'000) == LeaseDisposition::DeadlineMissed,
           "lease: start deadline missed");
    lease.start_deadline_ns = 0;
    lease.execution_timeout_ns = 500;
    expect(dispose_running(lease, 1, 7, local, ClockDomain::Monotonic, 2'000, 2'501)
               == LeaseDisposition::DeadlineMissed,
           "lease: execution timeout missed");
    ExecutionLease forever = lease;
    forever.expiry_ns = 0;
    expect(dispose(forever, 1, 7, local, 1'000'000'000) == LeaseDisposition::Accept,
           "lease: expiry 0 never expires");
}

void case_intent_contract() {
    const auto state = floor("b1/f2/map", 5, 3);
    IntentEnvelope first;
    first.intent_id = "intent-1";
    first.idempotency_key = "idem-1";
    first.lease = ExecutionLease{"task_1", "edge_a", 1, 7, 1'000, 5'000, state};
    expect(dispose_idempotency(first, nullptr) == IdempotencyDisposition::New,
           "intent: unseen key is new");
    expect(dispose_idempotency(first, &first) == IdempotencyDisposition::DuplicateSame,
           "intent: same key and content is idempotent");
    IntentEnvelope conflict = first;
    conflict.intent_id = "intent-2";
    expect(dispose_idempotency(conflict, &first) == IdempotencyDisposition::ConflictDifferent,
           "intent: same key with different content is rejected");

    AssetRef pose_old = state;
    pose_old.localization_epoch = 1;
    pose_old.pose_anchored = true;
    AssetRef pose_new = pose_old;
    pose_new.localization_epoch = 2;
    SpatialStateHeader pose_header{pose_new, Stamp{1'000, 0}, {}};
    const auto reverify = admit(pose_header, pose_old, 2'000);
    expect(reverify.disposition == AdmitDisposition::Reverify,
           "intent: localization epoch change requires re-verification");

    LocalizationHealth hold_health;
    hold_health.mode = LocalizationMode::Hold;
    hold_health.reason = ReasonCode::LocalizationLost;
    SpatialStateHeader held{state, Stamp{1'000, 0}, hold_health};
    const auto hold = admit(held, state, 2'000);
    expect(hold.disposition == AdmitDisposition::Hold && hold.reason == ReasonCode::LocalizationLost,
           "intent: uncertain state enters hold");

    RecoveryOutcome outcome;
    outcome.trigger = ReasonCode::LocalizationLost;
    outcome.action = "relocalize";
    outcome.duration_ns = 2'000'000;
    outcome.cost = 1.5;
    outcome.resumed = true;
    outcome.post_state = state;
    expect(outcome.resumed && outcome.post_state == state,
           "intent: recovery outcome records verified continuation state");
}

// ---------- 故障注入框架 v1：失效用例 → 可自动注入场景 ----------
void case_fault_injection() {
    using fork_spatial::contract::AdmitDisposition;
    using fork_spatial::contract::FaultKind;
    using fork_spatial::contract::FaultStep;
    using fork_spatial::contract::LeaseDisposition;
    using fork_spatial::contract::LocalizationMode;
    using fork_spatial::contract::ReasonCode;
    using fork_spatial::contract::healthy_world;
    using fork_spatial::contract::run_scenario;

    // 场景 1：定位丢失 → Hold → 恢复后 Accept
    {
        auto results = run_scenario(healthy_world(), {
            {FaultKind::LocalizationLost, 500},
            {FaultKind::Recover, 500},
        });
        expect(results[0].admit.disposition == AdmitDisposition::Hold, "fault: results[0].admit.disposition == AdmitDisposition::Hold");
        expect(results[0].admit.reason == ReasonCode::LocalizationLost, "fault: results[0].admit.reason == ReasonCode::LocalizationLost");
        expect(results[1].admit.disposition == AdmitDisposition::Accept, "fault: results[1].admit.disposition == AdmitDisposition::Accept");
    }

    // 场景 1b：重定位进度随故障清零、随恢复置为成功
    {
        auto world = healthy_world();
        expect(!world.header.health.candidate_found && !world.header.health.alignment_success,
               "fault: healthy world has no relocalization progress");
        (void)inject(world, {FaultKind::Recover, 0});
        expect(world.header.health.candidate_found && world.header.health.alignment_complete &&
                   world.header.health.alignment_success,
               "fault: Recover reports relocalization success");
        (void)inject(world, {FaultKind::PoseJump, 0, 2.0});
        expect(!world.header.health.candidate_found && !world.header.health.alignment_complete &&
                   !world.header.health.alignment_success,
               "fault: PoseJump clears relocalization progress");
        (void)inject(world, {FaultKind::Recover, 0});
        (void)inject(world, {FaultKind::LocalizationLost, 0});
        expect(!world.header.health.alignment_success,
               "fault: LocalizationLost clears relocalization progress");
    }

    // 场景 2：位姿跳变（定位重置）→ 位姿锚定状态 Reverify；换图 → Reject
    {
        auto world = healthy_world();
        world.header.asset.pose_anchored = true;
        world.expected.pose_anchored = true;
        auto results = run_scenario(world, {
            {FaultKind::LocalizationEpochBump, 100},
            {FaultKind::GenerationBump, 100},
        });
        expect(results[0].admit.disposition == AdmitDisposition::Reverify, "fault: results[0].admit.disposition == AdmitDisposition::Reverify");
        expect(results[0].admit.reason == ReasonCode::LocalizationLost, "fault: results[0].admit.reason == ReasonCode::LocalizationLost");
        expect(results[1].admit.disposition == AdmitDisposition::Reject, "fault: results[1].admit.disposition == AdmitDisposition::Reject");
        expect(results[1].admit.reason == ReasonCode::AssetStale, "fault: results[1].admit.reason == ReasonCode::AssetStale");
    }

    // 场景 3：过期 Token + 断链
    {
        auto results = run_scenario(healthy_world(), {
            {FaultKind::Expired, 100},
            {FaultKind::LinkLost, 50},
        });
        expect(results[0].admit.disposition == AdmitDisposition::Reject, "fault: results[0].admit.disposition == AdmitDisposition::Reject");
        expect(results[0].admit.reason == ReasonCode::AssetStale, "fault: results[0].admit.reason == ReasonCode::AssetStale");
        // 断链按契约只上报，不改准入判决（状态仍过期）
        expect(results[1].admit.disposition == AdmitDisposition::Reject, "fault: results[1].admit.disposition == AdmitDisposition::Reject");
    }

    // 场景 4：过期执行者 + 租约过期 + 开始截止错过
    {
        auto world = healthy_world();
        world.lease.start_deadline_ns = 5'000;
        auto results = run_scenario(world, {
            {FaultKind::WrongOwnerEpoch, 0},
            {FaultKind::LeaseExpired, 100},
            {FaultKind::DeadlineMiss, 0},
        });
        expect(results[0].lease == LeaseDisposition::StaleEpoch, "fault: results[0].lease == LeaseDisposition::StaleEpoch");
        expect(results[1].lease == LeaseDisposition::StaleEpoch, "fault: epoch checked before expiry");
        expect(results[2].lease == LeaseDisposition::StaleEpoch, "fault: epoch still guards");

        // 干净世界只注入 deadline miss
        auto world2 = healthy_world();
        world2.lease.start_deadline_ns = 5'000;
        auto r2 = run_scenario(world2, {{FaultKind::DeadlineMiss, 6'000}});
        expect(r2[0].lease == LeaseDisposition::DeadlineMissed, "fault: r2[0].lease == LeaseDisposition::DeadlineMissed");
    }
}

}  // namespace

int main() {
    case_localization_reset();
    case_floor_transition();
    case_expired_token();
    case_map_revision_mismatch();
    case_plan_failed();
    case_safety_veto();
    case_timeout();
    case_delta_out_of_order();
    case_stale_executor();
    case_intent_contract();
    case_fault_injection();
    if (failures == 0) {
        std::cout << "contract tests passed\n";
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

