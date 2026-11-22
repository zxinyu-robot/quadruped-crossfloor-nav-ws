#ifndef FORK_SPATIAL_COGNITION_BEHAVIOR_MEMORY_H
#define FORK_SPATIAL_COGNITION_BEHAVIOR_MEMORY_H

#include <cstdint>
#include <string>
#include <vector>

#include "cognition/semantic_token.h"

namespace fork_spatial::cognition {

enum class MemoryResetReason {
    TaskSwitch,
    LocalizationReset,
    FloorChange,
    Manual,
};

struct EsnConfig {
    std::size_t input_dim{0};
    std::size_t reservoir_dim{64};
    double spectral_radius{0.9};
    double leak_rate{0.3};
    double input_scale{0.5};
    double density{0.1};
    std::uint32_t washout_steps{5};
    std::uint64_t seed{42};
};

struct MemorySnapshot {
    std::vector<double> state;
    std::uint64_t step{};
    std::uint64_t epoch{};
};

// ESN 只维护近期时序动态 x_t：储备池权重固定、不学习；
// 读出层与策略更新属于端侧策略模块（EdgePolicy）。
class EchoStateMemory {
public:
    explicit EchoStateMemory(EsnConfig config);

    const std::vector<double>& update(const std::vector<double>& input);
    void reset(MemoryResetReason reason);

    bool warmed_up() const noexcept { return steps_since_reset_ >= config_.washout_steps; }
    const std::vector<double>& state() const noexcept { return state_; }
    std::uint64_t epoch() const noexcept { return epoch_; }
    MemoryResetReason last_reset_reason() const noexcept { return last_reset_; }
    std::size_t input_dim() const noexcept { return config_.input_dim; }
    std::size_t reservoir_dim() const noexcept { return config_.reservoir_dim; }

    MemorySnapshot snapshot() const;
    bool restore(const MemorySnapshot& snapshot);

private:
    EsnConfig config_;
    std::vector<double> w_in_;   // reservoir_dim × input_dim
    std::vector<double> w_res_;  // reservoir_dim × reservoir_dim
    std::vector<double> state_;
    std::uint64_t step_{};
    std::uint64_t epoch_{};
    std::uint32_t steps_since_reset_{};
    MemoryResetReason last_reset_{MemoryResetReason::Manual};
};

enum class LastAction : std::uint8_t {
    None,
    Explore,
    SwitchFrontier,
    Wait,
    Recover,
    ReturnHome,
};

struct ExecutionFeedback {
    LastAction last_action{LastAction::None};
    bool succeeded{false};
    bool vetoed{false};
    double progress_m{0.0};
};

inline constexpr std::size_t kBehaviorInputDim = 12;

// 固定维度的 ESN 输入：token 摘要 + 上一动作 + 执行反馈。
std::vector<double> make_behavior_input(const std::vector<SemanticToken>& tokens,
                                        const ExecutionFeedback& feedback);

}  // namespace fork_spatial::cognition

#endif
