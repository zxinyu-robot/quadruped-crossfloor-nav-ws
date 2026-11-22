#ifndef FORK_SPATIAL_COGNITION_EDGE_POLICY_H
#define FORK_SPATIAL_COGNITION_EDGE_POLICY_H

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "cognition/behavior_memory.h"
#include "contract/spatial_state.h"
#include "cognition/semantic_token.h"

namespace fork_spatial::cognition {

enum class BehaviorAction : std::uint8_t {
    Explore,
    SwitchFrontier,
    Wait,
    Recover,
    ReturnHome,
};
inline constexpr std::size_t kActionCount = 5;

struct PolicyDecision {
    BehaviorAction action{BehaviorAction::Wait};
    std::optional<std::uint64_t> target_token;
    std::uint64_t policy_version{};
    bool fallback{false};
    std::string reason;
    // 契约字段（Runtime → BT）：执行层据此拒绝过期 / 跨代 / 不适用的建议。
    std::string task_id;
    contract::AssetRef context;        // 决策所依据的空间上下文（含 generation）
    contract::Stamp validity;          // 建议有效期
    contract::ReasonCode degrade_reason{contract::ReasonCode::None};  // fallback 的原因
};

// 度量层对决策的执行结果：否决 = 几何不可执行，必须回流给学习。
struct DecisionOutcome {
    PolicyDecision decision;
    bool succeeded{false};
    bool vetoed{false};
    contract::ReasonCode failure_code{contract::ReasonCode::None};
    // 证据链（MotionSLAM → Runtime）：实际执行的策略版本与状态版本，
    // 与 decision 内记录不一致时该样本不得计入 active 统计。
    std::uint64_t executed_policy_version{};
    contract::AssetRef executed_context;
};

struct EdgePolicyConfig {
    double learning_rate{0.05};
    std::size_t experience_capacity{512};
    std::size_t min_validation_samples{20};
    double max_veto_rate{0.2};
    std::uint32_t rollback_failure_streak{5};
};

struct PolicyFeatures {
    std::vector<double> memory;
    std::vector<double> input;
};

// 线性读出策略：score = W · [x_t; u_t]。版本号随参数变化递增。
class ReadoutPolicy {
public:
    ReadoutPolicy() = default;
    ReadoutPolicy(std::size_t feature_dim, std::uint64_t version);

    std::array<double, kActionCount> scores(const std::vector<double>& features) const;
    void reinforce(const std::vector<double>& features, BehaviorAction action, double reward,
                   double learning_rate);
    std::uint64_t version() const noexcept { return version_; }
    void set_version(std::uint64_t v) noexcept { version_ = v; }
    bool empty() const noexcept { return weights_.empty(); }

private:
    std::size_t feature_dim_{};
    std::vector<double> weights_;
    std::uint64_t version_{};
};

struct ValidationStats {
    std::size_t samples{};
    std::size_t successes{};
    std::size_t vetoes{};
    double success_rate() const noexcept { return samples ? double(successes) / samples : 0.0; }
    double veto_rate() const noexcept { return samples ? double(vetoes) / samples : 0.0; }
};

// 端侧策略管理：实时路径只用 active；学习只更新 candidate，
// 旁路比对（候选只算不执行，与实际结果对比）通过后才 promote；连续失败回滚到上一版本；无可用策略回退规则兜底。
class EdgePolicyManager {
public:
    EdgePolicyManager(EdgePolicyConfig config, std::size_t feature_dim);

    // behavior_input 必须与最近一次喂给 memory.update() 的向量一致，
    // 否则 active 读到的特征和 ESN 实际消费的不匹配。
    PolicyDecision decide(const std::vector<SemanticToken>& tokens,
                          const std::vector<double>& behavior_input,
                          const EchoStateMemory& memory);
    void report(const DecisionOutcome& outcome, const PolicyFeatures& features);
    bool try_promote();
    bool rollback();

    std::uint64_t active_version() const noexcept { return active_.version(); }
    std::uint64_t candidate_version() const noexcept { return candidate_.version(); }
    const ValidationStats& active_stats() const noexcept { return active_stats_; }
    const ValidationStats& candidate_stats() const noexcept { return candidate_stats_; }
    std::size_t experience_size() const noexcept { return experience_.size(); }

    static PolicyDecision rule_fallback(const std::vector<SemanticToken>& tokens, std::string reason);

private:
    std::vector<double> concat(const std::vector<double>& memory, const std::vector<double>& input) const;

    EdgePolicyConfig config_;
    std::size_t feature_dim_;
    ReadoutPolicy active_;
    ReadoutPolicy candidate_;
    std::optional<ReadoutPolicy> previous_;
    ValidationStats active_stats_;
    ValidationStats candidate_stats_;
    std::deque<std::pair<DecisionOutcome, std::vector<double>>> experience_;
    std::uint32_t failure_streak_{};
    std::uint64_t next_version_{1};
};

}  // namespace fork_spatial::cognition

#endif
