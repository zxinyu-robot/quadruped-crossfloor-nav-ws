#ifndef FORK_SPATIAL_RUNTIME_SPATIAL_RUNTIME_H
#define FORK_SPATIAL_RUNTIME_SPATIAL_RUNTIME_H

#include "runtime/interfaces.h"
#include "runtime/state_store.h"

#include <memory>
#include <mutex>
#include <unordered_set>

namespace fork_spatial::runtime {

class SpatialRuntime {
public:
    SpatialRuntime(
        std::unique_ptr<IOptimizerBackend> backend,
        std::unique_ptr<IWindowPolicy> window_policy);

    SubmitResult submit(const FactorBatch& batch);
    SubmitResult advance_window(Timestamp watermark_ns);
    EstimateSnapshot optimize();

    [[nodiscard]] std::uint64_t graph_version() const;
    [[nodiscard]] std::vector<StateRecord> states() const;

private:
    static bool valid_noise(const NoiseModel& noise);
    SubmitResult validate_factor(
        const FactorProposal& factor,
        const StateStore& candidate,
        const std::unordered_set<FactorId>& candidate_factor_ids) const;

    mutable std::mutex mutex_;
    StateStore states_;
    std::unordered_set<FactorId> factor_ids_;
    std::unique_ptr<IOptimizerBackend> backend_;
    std::unique_ptr<IWindowPolicy> window_policy_;
    std::uint64_t graph_version_{};
};

}  // namespace fork_spatial::runtime

#endif
