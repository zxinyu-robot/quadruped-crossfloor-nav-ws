#ifndef FORK_SPATIAL_RUNTIME_MODEL_H
#define FORK_SPATIAL_RUNTIME_MODEL_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace fork_spatial::runtime {

using Timestamp = std::int64_t;
using FactorId = std::uint64_t;

enum class StateType {
    Pose,
    Velocity,
    ImuBias,
    Landmark,
    SensorExtrinsic,
    ClockOffset,
    SubmapPose,
};

enum class StateLifecycle {
    Requested,
    Initialized,
    Active,
    Fixed,
    Marginalized,
    Archived,
};

struct StateKey {
    std::uint32_t robot_id{};
    std::uint32_t session_id{};
    StateType type{StateType::Pose};
    Timestamp timestamp_ns{};
    std::uint32_t instance{};

    bool operator==(const StateKey&) const = default;
};

struct StateKeyHash {
    std::size_t operator()(const StateKey& key) const noexcept;
};

struct StateValue {
    // 后端无关的最小表示；具体流形解释由 StateType 和优化后端共同决定。
    std::vector<double> parameters;
};

struct StateRecord {
    StateKey key;
    StateValue value;
    StateLifecycle lifecycle{StateLifecycle::Requested};
    std::uint64_t version{};
};

enum class RobustKernelType { None, Huber, Cauchy };

struct NoiseModel {
    // 第一阶段仅接受对角协方差，后续可扩展为完整信息矩阵。
    std::vector<double> diagonal_covariance;
};

struct RobustKernel {
    RobustKernelType type{RobustKernelType::None};
    double scale{1.0};
};

struct FactorProposal {
    FactorId id{};
    std::string source;
    std::string factor_type;
    Timestamp measurement_time_ns{};
    std::vector<StateKey> states;
    std::vector<double> measurement;
    NoiseModel noise;
    RobustKernel robust_kernel;
};

struct StateRequest {
    StateKey key;
    StateValue initial_value;
    StateLifecycle initial_lifecycle{StateLifecycle::Active};
};

struct FactorBatch {
    std::vector<StateRequest> state_requests;
    std::vector<FactorProposal> factors;
};

struct AddState {
    StateRecord state;
};

struct TransitionState {
    StateKey key;
    StateLifecycle target;
};

struct AddFactor {
    FactorProposal factor;
};

using GraphOperation = std::variant<AddState, TransitionState, AddFactor>;

struct GraphTransaction {
    std::uint64_t sequence{};
    std::vector<GraphOperation> operations;
};

struct EstimateSnapshot {
    std::uint64_t graph_version{};
    Timestamp created_at_ns{};
    std::vector<StateRecord> states;
};

enum class SubmitCode {
    Accepted,
    DuplicateFactor,
    InvalidState,
    InvalidNoise,
    LateObservation,
    BackendRejected,
};

struct SubmitResult {
    SubmitCode code{SubmitCode::Accepted};
    std::string message;
    std::uint64_t graph_version{};

    explicit operator bool() const noexcept { return code == SubmitCode::Accepted; }
};

}  // namespace fork_spatial::runtime

#endif
