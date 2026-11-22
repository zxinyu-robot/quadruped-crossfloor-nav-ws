#include "runtime/spatial_runtime.h"

#include <iostream>
#include <memory>
#include <span>
#include <string>

namespace {

class DemoBackend final : public fork_spatial::runtime::IOptimizerBackend {
public:
    bool apply(
        const fork_spatial::runtime::GraphTransaction& transaction,
        std::string&) override {
        operation_count_ += transaction.operations.size();
        return true;
    }

    fork_spatial::runtime::EstimateSnapshot optimize(
        std::uint64_t graph_version) override {
        return {graph_version, 0, {}};
    }

    bool marginalize(
        std::span<const fork_spatial::runtime::StateKey>,
        fork_spatial::runtime::FactorProposal& prior,
        std::string&) override {
        prior = {};
        return true;
    }

    [[nodiscard]] std::size_t operation_count() const { return operation_count_; }

private:
    std::size_t operation_count_{};
};

}  // namespace

int main() {
    using namespace fork_spatial::runtime;

    auto backend = std::make_unique<DemoBackend>();
    DemoBackend* backend_view = backend.get();
    SpatialRuntime runtime(
        std::move(backend),
        std::make_unique<FixedLagWindowPolicy>(1'000'000'000));

    const StateKey pose{1, 1, StateType::Pose, 1'000'000'000, 0};
    FactorBatch batch;
    batch.state_requests.push_back(
        {pose, {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}});

    const SubmitResult result = runtime.submit(batch);
    if (!result) {
        std::cerr << "submission failed: " << result.message << '\n';
        return 1;
    }

    std::cout << "accepted graph version: " << result.graph_version << '\n'
              << "active states: " << runtime.states().size() << '\n'
              << "backend operations: " << backend_view->operation_count() << '\n';
    return 0;
}
