#include "cognition/behavior_memory.h"
#include "cognition/edge_policy.h"

#include <array>
#include "cognition/semantic_token.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

using namespace fork_spatial::cognition;

int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

GeometricObservation make_observation(fork_spatial::runtime::Timestamp stamp) {
    GeometricObservation obs;
    obs.map = {1, 1, "occupancy", 3, "map"};
    obs.stamp_ns = stamp;
    obs.localization_valid = true;
    obs.localization_quality = 0.9;
    obs.occupancy_ratio = 0.2;
    obs.frontiers = {{1, 4.0, 0.0, 0.0, 10.0, 0.9}, {2, 20.0, 0.0, 0.0, 5.0, 0.8}};
    obs.goal = std::array<double, 3>{6.0, 0.0, 1.0};
    return obs;
}

void test_encoder_and_lifecycle() {
    RuleBasedTokenEncoder encoder;
    TokenStore store;
    const auto tokens = encoder.encode(make_observation(0));
    expect(tokens.size() == 5, "scene + risk + 2 frontiers + goal");
    for (const auto& t : tokens) {
        expect(store.upsert(t) == UpsertCode::Inserted, "insert encoded token");
    }
    expect(store.valid_tokens().size() == 5, "all tokens valid");
    const auto near = store.find(1001);
    const auto far = store.find(1002);
    expect(near && far && near->attributes.priority > far->attributes.priority,
           "closer high-gain frontier has higher priority");

    auto older = tokens.front();
    older.resource.revision = 2;
    expect(store.upsert(older) == UpsertCode::RejectedStaleRevision, "reject older map revision");

    auto invalid = tokens.front();
    invalid.attributes.risk = 2.0;
    expect(store.upsert(invalid) == UpsertCode::RejectedInvalid, "reject out-of-range attribute");

    expect(store.tick(3'000'000'000) == 5, "ttl expiry marks stale");
    expect(store.valid_tokens().empty(), "no valid tokens after ttl");
    expect(store.tick(9'000'000'000) == 5, "grace expiry evicts");
    expect(!store.find(1).has_value(), "evicted token removed from store");
}

void test_eviction_hardening() {
    // 逐出/失效后：更旧 revision 的延迟观测不可复活；同 revision 乱序按观测时戳取新者
    RuleBasedTokenEncoder encoder;
    TokenStore store;
    const auto tokens = encoder.encode(make_observation(0));
    for (const auto& t : tokens) {
        store.upsert(t);
    }
    auto newer = tokens[3];
    newer.stamp_ns = 100;
    expect(store.upsert(newer) == UpsertCode::Updated,
           "same revision newer stamp accepted");
    auto late = tokens[3];
    late.stamp_ns = 0;
    expect(store.upsert(late) == UpsertCode::RejectedStaleRevision,
           "same revision older stamp rejected");

    store.tick(3'000'000'000);
    store.tick(9'000'000'000);
    expect(store.valid_tokens().empty() && !store.find(1001).has_value(),
           "evicted tokens removed");

    auto ghost = tokens[3];
    ghost.resource.revision = 2;
    expect(store.upsert(ghost) == UpsertCode::RejectedStaleRevision,
           "evicted id rejects older revision");
    ghost.resource.revision = 3;
    expect(store.upsert(ghost) == UpsertCode::Inserted,
           "evicted id accepts current revision");
    expect(store.find(ghost.token_id)->version == 3,
           "same-owner resurrection continues version");

    store.invalidate_session(1, 1);
    ghost.resource.revision = 1;
    expect(store.upsert(ghost) == UpsertCode::RejectedStaleRevision,
           "invalidated id rejects older revision");

    // 新会话复用同一 token_id 空间：不受旧会话墓碑约束，version 独立重计
    auto next_session = ghost;
    next_session.resource.session_id = 2;
    expect(store.upsert(next_session) == UpsertCode::Inserted,
           "new session reuses token id despite tombstone");
    expect(store.find(ghost.token_id)->version == 1,
           "new session lineage restarts version");

    // 旧会话的延迟观测（rev 高于新会话活 token、低于其自身墓碑）仍被挡，
    // 不得覆盖新会话的 token
    auto straggler = ghost;
    straggler.resource.revision = 2;
    straggler.stamp_ns = 200;
    expect(store.upsert(straggler) == UpsertCode::RejectedStaleRevision,
           "old-session straggler cannot overwrite new session token");
    expect(store.find(ghost.token_id)->resource.session_id == 2,
           "live token still belongs to new session");
}

void test_session_invalidation() {
    RuleBasedTokenEncoder encoder;
    TokenStore store;
    for (const auto& t : encoder.encode(make_observation(0))) {
        store.upsert(t);
    }
    expect(store.invalidate_session(1, 1) == 5, "localization reset evicts session tokens");
    expect(store.valid_tokens().empty(), "nothing valid after session reset");
}

void test_esn_memory() {
    EsnConfig cfg;
    cfg.input_dim = kBehaviorInputDim;
    cfg.reservoir_dim = 32;
    EchoStateMemory a(cfg);
    EchoStateMemory b(cfg);
    const std::vector<double> u(kBehaviorInputDim, 0.5);
    for (int i = 0; i < 10; ++i) {
        a.update(u);
        b.update(u);
    }
    expect(a.state() == b.state(), "deterministic reservoir for same seed");
    expect(a.warmed_up(), "warmed up after washout");
    bool bounded = true;
    for (double x : a.state()) {
        bounded = bounded && std::isfinite(x) && std::abs(x) <= 1.0;
    }
    expect(bounded, "state bounded by tanh");

    const auto snap = a.snapshot();
    a.reset(MemoryResetReason::FloorChange);
    expect(!a.warmed_up() && a.epoch() == 1, "reset starts new epoch with washout");
    expect(a.restore(snap) && a.state() == b.state(), "restore snapshot");
}

void test_edge_policy() {
    RuleBasedTokenEncoder encoder;
    const auto tokens = encoder.encode(make_observation(0));
    EsnConfig cfg;
    cfg.input_dim = kBehaviorInputDim;
    cfg.reservoir_dim = 16;
    cfg.washout_steps = 2;
    EchoStateMemory memory(cfg);

    EdgePolicyConfig pcfg;
    pcfg.min_validation_samples = 10;
    EdgePolicyManager manager(pcfg, cfg.reservoir_dim + kBehaviorInputDim);

    const auto u0 = make_behavior_input(tokens, {});
    auto d = manager.decide(tokens, u0, memory);
    expect(d.fallback && d.action == BehaviorAction::Explore && d.target_token == 1001,
           "no active policy -> rule fallback explores best frontier");

    const auto u = make_behavior_input(tokens, {});
    for (int i = 0; i < 30; ++i) {
        memory.update(u);
        PolicyDecision explore{BehaviorAction::Explore, 1001, manager.active_version(), false, "shadow"};
        manager.report({explore, true, false, fork_spatial::contract::ReasonCode::None}, {memory.state(), u});
    }
    expect(manager.try_promote(), "validated candidate promoted");
    expect(manager.active_version() > 0, "active policy version set");

    d = manager.decide(tokens, u, memory);
    expect(!d.fallback && d.action == BehaviorAction::Explore, "active policy learned to explore");

    for (int i = 0; i < 5; ++i) {
        PolicyDecision bad{BehaviorAction::Explore, 1001, manager.active_version(), false, "active"};
        manager.report({bad, false, true, fork_spatial::contract::ReasonCode::SafetyVeto}, {memory.state(), u});
    }
    expect(manager.active_version() == 0, "failure streak rolls back to rule fallback");
    expect(manager.decide(tokens, u, memory).fallback, "fallback after rollback");
}

void test_policy_rejection_paths() {
    RuleBasedTokenEncoder encoder;
    const auto tokens = encoder.encode(make_observation(0));
    EsnConfig cfg;
    cfg.input_dim = kBehaviorInputDim;
    cfg.reservoir_dim = 16;
    cfg.washout_steps = 0;
    EchoStateMemory memory(cfg);

    EdgePolicyConfig pcfg;
    pcfg.min_validation_samples = 10;
    pcfg.max_veto_rate = 0.2;
    EdgePolicyManager manager(pcfg, cfg.reservoir_dim + kBehaviorInputDim);
    const auto u = make_behavior_input(tokens, {});

    // 样本不足不能 promote
    for (int i = 0; i < 5; ++i) {
        PolicyDecision d{BehaviorAction::Explore, 1001, 0, false, ""};
        manager.report({d, true, false, fork_spatial::contract::ReasonCode::None}, {memory.state(), u});
    }
    expect(!manager.try_promote(), "reject promote below min samples");

    // 否决率过高不能 promote
    for (int i = 0; i < 10; ++i) {
        PolicyDecision d{BehaviorAction::Explore, 1001, 0, false, ""};
        manager.report({d, false, true, fork_spatial::contract::ReasonCode::SafetyVeto}, {memory.state(), u});
    }
    expect(!manager.try_promote(), "reject promote above veto rate");

    // 无 frontier 时兜底必须 Wait 而不是 Explore 空目标
    auto obs = make_observation(0);
    obs.frontiers.clear();
    const auto no_frontier = encoder.encode(obs);
    const auto d = EdgePolicyManager::rule_fallback(no_frontier, "test");
    expect(d.action == BehaviorAction::Wait && !d.target_token.has_value(),
           "fallback waits when no frontier");
}

void test_evicted_reupsert_and_invalid_loc() {
    RuleBasedTokenEncoder encoder;
    TokenStore store;
    for (const auto& t : encoder.encode(make_observation(0))) {
        store.upsert(t);
    }
    store.tick(9'000'000'000);  // 全部 evict
    // evicted token 可以用更新的 revision 重新插入
    auto fresh = encoder.encode(make_observation(10'000'000'000));
    for (auto& t : fresh) {
        t.resource.revision = 4;
        expect(store.upsert(t) == UpsertCode::Updated, "evicted token re-upserted");
    }
    expect(store.valid_tokens().size() == 5, "tokens revived after re-upsert");

    // 定位失效 -> 编码器产出低置信度 token -> 被 store 拒收
    auto bad = make_observation(20'000'000'000);
    bad.localization_valid = false;
    bad.localization_quality = 0.0;
    TokenStore store2;
    std::size_t inserted = 0;
    for (const auto& t : encoder.encode(bad)) {
        inserted += store2.upsert(t) == UpsertCode::Inserted ? 1 : 0;
    }
    expect(inserted == 0, "localization-invalid tokens rejected by store");
}

void test_esn_restore_bounds() {
    EsnConfig cfg;
    cfg.input_dim = kBehaviorInputDim;
    cfg.reservoir_dim = 16;
    EchoStateMemory memory(cfg);
    MemorySnapshot bad;
    bad.state.assign(4, 0.0);  // 维度不符
    expect(!memory.restore(bad), "reject restore with wrong dimension");
}

}  // namespace

int main() {
    test_encoder_and_lifecycle();
    test_eviction_hardening();
    test_session_invalidation();
    test_esn_memory();
    test_edge_policy();
    test_policy_rejection_paths();
    test_evicted_reupsert_and_invalid_loc();
    test_esn_restore_bounds();
    if (failures > 0) {
        return EXIT_FAILURE;
    }
    std::cout << "cognition tests passed\n";
    return EXIT_SUCCESS;
}
