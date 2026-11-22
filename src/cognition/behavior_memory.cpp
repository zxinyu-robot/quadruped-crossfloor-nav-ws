#include "cognition/behavior_memory.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace fork_spatial::cognition {
namespace {

double power_iteration_radius(const std::vector<double>& w, std::size_t n) {
    std::vector<double> v(n, 1.0 / std::sqrt(static_cast<double>(n)));
    std::vector<double> next(n);
    double radius = 0.0;
    for (int it = 0; it < 100; ++it) {
        for (std::size_t i = 0; i < n; ++i) {
            double acc = 0.0;
            for (std::size_t j = 0; j < n; ++j) {
                acc += w[i * n + j] * v[j];
            }
            next[i] = acc;
        }
        double norm = 0.0;
        for (double x : next) {
            norm += x * x;
        }
        norm = std::sqrt(norm);
        if (norm < 1e-12) {
            return 0.0;
        }
        for (std::size_t i = 0; i < n; ++i) {
            v[i] = next[i] / norm;
        }
        radius = norm;
    }
    return radius;
}

}  // namespace

EchoStateMemory::EchoStateMemory(EsnConfig config)
    : config_(config),
      w_in_(config.reservoir_dim * config.input_dim),
      w_res_(config.reservoir_dim * config.reservoir_dim, 0.0),
      state_(config.reservoir_dim, 0.0) {
    std::mt19937_64 rng(config_.seed);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::bernoulli_distribution keep(std::clamp(config_.density, 0.0, 1.0));
    for (auto& w : w_in_) {
        w = uni(rng) * config_.input_scale;
    }
    for (auto& w : w_res_) {
        w = keep(rng) ? uni(rng) : 0.0;
    }
    const double radius = power_iteration_radius(w_res_, config_.reservoir_dim);
    if (radius > 1e-12) {
        const double scale = config_.spectral_radius / radius;
        for (auto& w : w_res_) {
            w *= scale;
        }
    }
}

const std::vector<double>& EchoStateMemory::update(const std::vector<double>& input) {
    const std::size_t n = config_.reservoir_dim;
    const std::size_t m = std::min(input.size(), config_.input_dim);
    std::vector<double> next(n);
    for (std::size_t i = 0; i < n; ++i) {
        double acc = 0.0;
        for (std::size_t j = 0; j < m; ++j) {
            acc += w_in_[i * config_.input_dim + j] * input[j];
        }
        for (std::size_t j = 0; j < n; ++j) {
            acc += w_res_[i * n + j] * state_[j];
        }
        next[i] = (1.0 - config_.leak_rate) * state_[i] + config_.leak_rate * std::tanh(acc);
    }
    state_ = std::move(next);
    ++step_;
    ++steps_since_reset_;
    return state_;
}

void EchoStateMemory::reset(MemoryResetReason reason) {
    std::fill(state_.begin(), state_.end(), 0.0);
    steps_since_reset_ = 0;
    ++epoch_;
    last_reset_ = reason;
}

MemorySnapshot EchoStateMemory::snapshot() const { return {state_, step_, epoch_}; }

bool EchoStateMemory::restore(const MemorySnapshot& snapshot) {
    if (snapshot.state.size() != config_.reservoir_dim) {
        return false;
    }
    state_ = snapshot.state;
    step_ = snapshot.step;
    epoch_ = snapshot.epoch;
    steps_since_reset_ = config_.washout_steps;
    return true;
}

std::vector<double> make_behavior_input(const std::vector<SemanticToken>& tokens,
                                        const ExecutionFeedback& feedback) {
    std::vector<double> u(kBehaviorInputDim, 0.0);
    double frontier_count = 0.0;
    for (const auto& t : tokens) {
        switch (t.kind) {
            case TokenKind::Scene:
                u[0] = t.attributes.traversability;
                u[1] = t.attributes.risk;
                break;
            case TokenKind::Risk:
                u[2] = t.attributes.risk;
                break;
            case TokenKind::Goal:
                u[3] = t.attributes.priority;
                break;
            case TokenKind::Frontier:
                frontier_count += 1.0;
                u[4] = std::max(u[4], t.attributes.priority);
                u[5] = std::max(u[5], t.attributes.information_gain);
                break;
            case TokenKind::Execution:
                break;
        }
    }
    u[6] = std::tanh(frontier_count / 5.0);
    u[7] = static_cast<double>(feedback.last_action) / 5.0;
    u[8] = feedback.succeeded ? 1.0 : 0.0;
    u[9] = feedback.vetoed ? 1.0 : 0.0;
    u[10] = std::tanh(feedback.progress_m);
    u[11] = 1.0;  // bias
    return u;
}

}  // namespace fork_spatial::cognition
