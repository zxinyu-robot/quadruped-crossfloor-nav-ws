#include "runtime/spatial_runtime.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace fork_spatial::runtime {
namespace {

SubmitResult failure(SubmitCode code, std::string message, std::uint64_t version) {
    return SubmitResult{code, std::move(message), version};
}

}  // namespace

SpatialRuntime::SpatialRuntime(
    std::unique_ptr<IOptimizerBackend> backend,
    std::unique_ptr<IWindowPolicy> window_policy)
    : backend_(std::move(backend)), window_policy_(std::move(window_policy)) {
    if (!backend_ || !window_policy_) {
        throw std::invalid_argument("runtime backend and window policy are required");
    }
}

SubmitResult SpatialRuntime::submit(const FactorBatch& batch) {
    std::scoped_lock lock(mutex_);

    StateStore candidate_states = states_;
    auto candidate_factor_ids = factor_ids_;
    GraphTransaction transaction;
    transaction.sequence = graph_version_ + 1;
    std::string error;

    for (const StateRequest& request : batch.state_requests) {
        if (request.initial_lifecycle != StateLifecycle::Active &&
            request.initial_lifecycle != StateLifecycle::Fixed) {
            return failure(
                SubmitCode::InvalidState,
                "new states must enter the graph as Active or Fixed",
                graph_version_);
        }

        StateRecord state{
            request.key,
            request.initial_value,
            StateLifecycle::Requested,
            0,
        };
        if (!candidate_states.add(state, error)) {
            return failure(SubmitCode::InvalidState, error, graph_version_);
        }
        transaction.operations.emplace_back(AddState{state});

        candidate_states.transition(
            request.key, StateLifecycle::Initialized, error);
        transaction.operations.emplace_back(
            TransitionState{request.key, StateLifecycle::Initialized});
        candidate_states.transition(request.key, StateLifecycle::Active, error);
        transaction.operations.emplace_back(
            TransitionState{request.key, StateLifecycle::Active});

        if (request.initial_lifecycle == StateLifecycle::Fixed) {
            candidate_states.transition(request.key, StateLifecycle::Fixed, error);
            transaction.operations.emplace_back(
                TransitionState{request.key, StateLifecycle::Fixed});
        }
    }

    for (const FactorProposal& factor : batch.factors) {
        SubmitResult validation =
            validate_factor(factor, candidate_states, candidate_factor_ids);
        if (!validation) {
            return validation;
        }
        candidate_factor_ids.insert(factor.id);
        transaction.operations.emplace_back(AddFactor{factor});
    }

    if (transaction.operations.empty()) {
        return SubmitResult{SubmitCode::Accepted, "empty batch", graph_version_};
    }
    if (!backend_->apply(transaction, error)) {
        return failure(SubmitCode::BackendRejected, error, graph_version_);
    }

    states_ = std::move(candidate_states);
    factor_ids_ = std::move(candidate_factor_ids);
    graph_version_ = transaction.sequence;
    return SubmitResult{SubmitCode::Accepted, {}, graph_version_};
}

SubmitResult SpatialRuntime::advance_window(Timestamp watermark_ns) {
    std::scoped_lock lock(mutex_);

    const std::vector<StateRecord> records = states_.records();
    const std::vector<StateKey> selected =
        window_policy_->select_for_marginalization(records, watermark_ns);
    if (selected.empty()) {
        return SubmitResult{
            SubmitCode::Accepted, "no states selected", graph_version_};
    }

    FactorProposal prior;
    std::string error;
    if (!backend_->marginalize(selected, prior, error)) {
        return failure(SubmitCode::BackendRejected, error, graph_version_);
    }

    StateStore candidate_states = states_;
    auto candidate_factor_ids = factor_ids_;
    GraphTransaction transaction;
    transaction.sequence = graph_version_ + 1;
    for (const StateKey& key : selected) {
        if (!candidate_states.transition(
                key, StateLifecycle::Marginalized, error)) {
            return failure(SubmitCode::InvalidState, error, graph_version_);
        }
        transaction.operations.emplace_back(
            TransitionState{key, StateLifecycle::Marginalized});
    }

    if (prior.id != 0) {
        SubmitResult validation =
            validate_factor(prior, candidate_states, candidate_factor_ids);
        if (!validation) {
            return validation;
        }
        candidate_factor_ids.insert(prior.id);
        transaction.operations.emplace_back(AddFactor{std::move(prior)});
    }

    if (!backend_->apply(transaction, error)) {
        return failure(SubmitCode::BackendRejected, error, graph_version_);
    }
    states_ = std::move(candidate_states);
    factor_ids_ = std::move(candidate_factor_ids);
    graph_version_ = transaction.sequence;
    return SubmitResult{SubmitCode::Accepted, {}, graph_version_};
}

EstimateSnapshot SpatialRuntime::optimize() {
    std::scoped_lock lock(mutex_);
    EstimateSnapshot snapshot = backend_->optimize(graph_version_);
    snapshot.graph_version = graph_version_;
    return snapshot;
}

std::uint64_t SpatialRuntime::graph_version() const {
    std::scoped_lock lock(mutex_);
    return graph_version_;
}

std::vector<StateRecord> SpatialRuntime::states() const {
    std::scoped_lock lock(mutex_);
    return states_.records();
}

bool SpatialRuntime::valid_noise(const NoiseModel& noise) {
    if (noise.diagonal_covariance.empty()) {
        return false;
    }
    for (double variance : noise.diagonal_covariance) {
        if (!std::isfinite(variance) || variance <= 0.0) {
            return false;
        }
    }
    return true;
}

SubmitResult SpatialRuntime::validate_factor(
    const FactorProposal& factor,
    const StateStore& candidate,
    const std::unordered_set<FactorId>& candidate_factor_ids) const {
    if (factor.id == 0 || factor.source.empty() || factor.factor_type.empty() ||
        factor.states.empty()) {
        return failure(
            SubmitCode::InvalidState,
            "factor identity, source, type and state list are required",
            graph_version_);
    }
    if (candidate_factor_ids.contains(factor.id)) {
        return failure(
            SubmitCode::DuplicateFactor, "factor id already exists", graph_version_);
    }
    if (!valid_noise(factor.noise)) {
        return failure(
            SubmitCode::InvalidNoise,
            "factor covariance must be finite and positive",
            graph_version_);
    }
    for (const StateKey& key : factor.states) {
        const StateRecord* state = candidate.find(key);
        if (state != nullptr &&
            (state->lifecycle == StateLifecycle::Marginalized ||
             state->lifecycle == StateLifecycle::Archived)) {
            return failure(
                SubmitCode::LateObservation,
                "factor references a marginalized state",
                graph_version_);
        }
        if (!candidate.factor_eligible(key)) {
            return failure(
                SubmitCode::InvalidState,
                "factor references a missing or inactive state",
                graph_version_);
        }
    }
    return SubmitResult{SubmitCode::Accepted, {}, graph_version_};
}

}  // namespace fork_spatial::runtime
