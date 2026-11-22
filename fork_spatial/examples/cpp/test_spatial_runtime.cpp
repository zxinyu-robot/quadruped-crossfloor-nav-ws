#include "runtime/spatial_runtime.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <utility>

namespace {

class FakeBackend final : public fork_spatial::runtime::IOptimizerBackend {
public:
    bool apply(
        const fork_spatial::runtime::GraphTransaction& transaction,
        std::string& error) override {
        if (reject_next_apply_) {
            reject_next_apply_ = false;
            error = "injected backend failure";
            return false;
        }
        last_sequence_ = transaction.sequence;
        return true;
    }

    fork_spatial::runtime::EstimateSnapshot optimize(
        std::uint64_t graph_version) override {
        return fork_spatial::runtime::EstimateSnapshot{graph_version, 0, {}};
    }

    bool marginalize(
        std::span<const fork_spatial::runtime::StateKey> states,
        fork_spatial::runtime::FactorProposal& prior,
        std::string& error) override {
        (void)error;
        prior = {};
        marginalized_count_ += states.size();
        return true;
    }

    std::uint64_t last_sequence_{};
    std::size_t marginalized_count_{};
    bool reject_next_apply_{};
};

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

fork_spatial::runtime::FactorProposal odometry_factor(
    fork_spatial::runtime::FactorId id,
    const fork_spatial::runtime::StateKey& first,
    const fork_spatial::runtime::StateKey& second) {
    return fork_spatial::runtime::FactorProposal{
        id,
        "test_odometry",
        "relative_pose",
        second.timestamp_ns,
        {first, second},
        {1.0, 0.0, 0.0},
        {{0.1, 0.1, 0.1}},
        {},
    };
}

}  // namespace

int main() {
    using namespace fork_spatial::runtime;

    auto backend = std::make_unique<FakeBackend>();
    FakeBackend* backend_view = backend.get();
    SpatialRuntime runtime(
        std::move(backend),
        std::make_unique<FixedLagWindowPolicy>(100));

    const StateKey old_pose{1, 1, StateType::Pose, 0, 0};
    const StateKey current_pose{1, 1, StateType::Pose, 200, 0};
    FactorBatch initial_batch;
    initial_batch.state_requests = {
        {old_pose, {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}},
        {current_pose, {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}},
    };
    initial_batch.factors.push_back(
        odometry_factor(1, old_pose, current_pose));

    const SubmitResult accepted = runtime.submit(initial_batch);
    expect(static_cast<bool>(accepted), "initial batch should be accepted");
    expect(runtime.graph_version() == 1, "accepted batch should advance version");

    FactorBatch duplicate_batch;
    duplicate_batch.factors.push_back(
        odometry_factor(1, old_pose, current_pose));
    expect(
        runtime.submit(duplicate_batch).code == SubmitCode::DuplicateFactor,
        "duplicate factor id should be rejected");

    FactorBatch invalid_noise_batch;
    FactorProposal invalid_noise =
        odometry_factor(2, old_pose, current_pose);
    invalid_noise.noise.diagonal_covariance[0] = 0.0;
    invalid_noise_batch.factors.push_back(std::move(invalid_noise));
    expect(
        runtime.submit(invalid_noise_batch).code == SubmitCode::InvalidNoise,
        "non-positive covariance should be rejected");

    const StateKey rejected_pose{1, 1, StateType::Pose, 300, 0};
    FactorBatch rejected_batch;
    rejected_batch.state_requests.push_back(
        {rejected_pose, {{2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}});
    backend_view->reject_next_apply_ = true;
    expect(
        runtime.submit(rejected_batch).code == SubmitCode::BackendRejected,
        "backend failure should be surfaced");
    expect(runtime.graph_version() == 1, "backend failure must not advance version");
    expect(
        runtime.states().size() == 2,
        "backend failure must not commit candidate states");

    const SubmitResult marginalized = runtime.advance_window(150);
    expect(static_cast<bool>(marginalized), "window advance should succeed");
    expect(runtime.graph_version() == 2, "window advance should advance version");

    const std::vector<StateRecord> states = runtime.states();
    const auto old_state = std::find_if(
        states.begin(), states.end(),
        [&](const StateRecord& state) { return state.key == old_pose; });
    expect(old_state != states.end(), "old state should still be addressable");
    expect(
        old_state->lifecycle == StateLifecycle::Marginalized,
        "old state should be marked marginalized");

    FactorBatch late_batch;
    late_batch.factors.push_back(
        odometry_factor(2, old_pose, current_pose));
    expect(
        runtime.submit(late_batch).code == SubmitCode::LateObservation,
        "late observation should be rejected");

    const EstimateSnapshot snapshot = runtime.optimize();
    expect(snapshot.graph_version == 2, "snapshot should carry graph version");
    return 0;
}
