#include "cognition/edge_policy.h"

#include <algorithm>
#include <cmath>

namespace fork_spatial::cognition {
namespace {

std::optional<std::uint64_t> best_frontier(const std::vector<SemanticToken>& tokens,
                                           std::optional<std::uint64_t> exclude = std::nullopt) {
    std::optional<std::uint64_t> best;
    double best_priority = -1.0;
    for (const auto& t : tokens) {
        if (t.kind == TokenKind::Frontier && t.token_id != exclude && t.attributes.priority > best_priority) {
            best_priority = t.attributes.priority;
            best = t.token_id;
        }
    }
    return best;
}

double max_risk(const std::vector<SemanticToken>& tokens) {
    double r = 0.0;
    for (const auto& t : tokens) {
        if (t.kind == TokenKind::Risk) {
            r = std::max(r, t.attributes.risk);
        }
    }
    return r;
}

}  // namespace

ReadoutPolicy::ReadoutPolicy(std::size_t feature_dim, std::uint64_t version)
    : feature_dim_(feature_dim), weights_(feature_dim * kActionCount, 0.0), version_(version) {}

std::array<double, kActionCount> ReadoutPolicy::scores(const std::vector<double>& features) const {
    std::array<double, kActionCount> out{};
    const std::size_t n = std::min(features.size(), feature_dim_);
    for (std::size_t a = 0; a < kActionCount; ++a) {
        double acc = 0.0;
        for (std::size_t j = 0; j < n; ++j) {
            acc += weights_[a * feature_dim_ + j] * features[j];
        }
        out[a] = acc;
    }
    return out;
}

void ReadoutPolicy::reinforce(const std::vector<double>& features, BehaviorAction action, double reward,
                              double learning_rate) {
    const std::size_t a = static_cast<std::size_t>(action);
    const std::size_t n = std::min(features.size(), feature_dim_);
    for (std::size_t j = 0; j < n; ++j) {
        weights_[a * feature_dim_ + j] += learning_rate * reward * features[j];
    }
}

EdgePolicyManager::EdgePolicyManager(EdgePolicyConfig config, std::size_t feature_dim)
    : config_(config), feature_dim_(feature_dim) {}

std::vector<double> EdgePolicyManager::concat(const std::vector<double>& memory,
                                              const std::vector<double>& input) const {
    std::vector<double> f;
    f.reserve(feature_dim_);
    f.insert(f.end(), memory.begin(), memory.end());
    f.insert(f.end(), input.begin(), input.end());
    f.resize(feature_dim_, 0.0);
    return f;
}

PolicyDecision EdgePolicyManager::rule_fallback(const std::vector<SemanticToken>& tokens, std::string reason) {
    PolicyDecision d;
    d.fallback = true;
    d.reason = std::move(reason);
    if (max_risk(tokens) > 0.7) {
        d.action = BehaviorAction::Recover;
    } else if (auto f = best_frontier(tokens)) {
        d.action = BehaviorAction::Explore;
        d.target_token = f;
    } else {
        d.action = BehaviorAction::Wait;
    }
    return d;
}

PolicyDecision EdgePolicyManager::decide(const std::vector<SemanticToken>& tokens,
                                         const std::vector<double>& behavior_input,
                                         const EchoStateMemory& memory) {
    if (active_.empty()) {
        return rule_fallback(tokens, "no_active_policy");
    }
    if (!memory.warmed_up()) {
        return rule_fallback(tokens, "memory_washout");
    }
    const auto features = concat(memory.state(), behavior_input);
    const auto s = active_.scores(features);
    const auto best = static_cast<BehaviorAction>(std::distance(s.begin(), std::max_element(s.begin(), s.end())));
    PolicyDecision d;
    d.action = best;
    d.policy_version = active_.version();
    if (best == BehaviorAction::Explore) {
        d.target_token = best_frontier(tokens);
    }
    if (best == BehaviorAction::SwitchFrontier) {
        const auto first = best_frontier(tokens);
        d.target_token = best_frontier(tokens, first);
    }
    if ((best == BehaviorAction::Explore || best == BehaviorAction::SwitchFrontier) && !d.target_token) {
        return rule_fallback(tokens, "no_frontier_target");
    }
    d.reason = "active_policy";
    return d;
}

void EdgePolicyManager::report(const DecisionOutcome& outcome, const PolicyFeatures& features) {
    auto f = concat(features.memory, features.input);
    if (!outcome.decision.fallback && outcome.decision.policy_version == active_.version()) {
        ++active_stats_.samples;
        active_stats_.successes += outcome.succeeded ? 1 : 0;
        active_stats_.vetoes += outcome.vetoed ? 1 : 0;
        failure_streak_ = outcome.succeeded ? 0 : failure_streak_ + 1;
        if (failure_streak_ >= config_.rollback_failure_streak) {
            rollback();
        }
    }
    // 学习只作用于候选策略，不阻塞、不修改实时 active。
    if (candidate_.empty()) {
        candidate_ = ReadoutPolicy(feature_dim_, next_version_++);
    }
    // 旁路比对必须先预测再训练，否则统计自我证实。
    const auto predicted = candidate_.scores(f);
    const auto pick = static_cast<BehaviorAction>(
        std::distance(predicted.begin(), std::max_element(predicted.begin(), predicted.end())));
    if (pick == outcome.decision.action) {
        ++candidate_stats_.samples;
        candidate_stats_.successes += outcome.succeeded ? 1 : 0;
        candidate_stats_.vetoes += outcome.vetoed ? 1 : 0;
    }
    const double reward = outcome.vetoed ? -1.0 : (outcome.succeeded ? 1.0 : -0.5);
    candidate_.reinforce(f, outcome.decision.action, reward, config_.learning_rate);

    experience_.emplace_back(outcome, std::move(f));
    while (experience_.size() > config_.experience_capacity) {
        experience_.pop_front();
    }
}

bool EdgePolicyManager::try_promote() {
    if (candidate_.empty() || candidate_stats_.samples < config_.min_validation_samples) {
        return false;
    }
    if (candidate_stats_.veto_rate() > config_.max_veto_rate) {
        return false;
    }
    if (!active_.empty() && active_stats_.samples > 0 &&
        candidate_stats_.success_rate() < active_stats_.success_rate()) {
        return false;
    }
    if (!active_.empty()) {
        previous_ = active_;
    }
    active_ = candidate_;
    active_stats_ = candidate_stats_;
    candidate_ = ReadoutPolicy(feature_dim_, next_version_++);
    for (const auto& [outcome, f] : experience_) {
        const double reward = outcome.vetoed ? -1.0 : (outcome.succeeded ? 1.0 : -0.5);
        candidate_.reinforce(f, outcome.decision.action, reward, config_.learning_rate);
    }
    candidate_stats_ = {};
    failure_streak_ = 0;
    return true;
}

bool EdgePolicyManager::rollback() {
    failure_streak_ = 0;
    if (previous_) {
        active_ = *previous_;
        previous_.reset();
    } else {
        active_ = ReadoutPolicy();
    }
    active_stats_ = {};
    return true;
}

}  // namespace fork_spatial::cognition
