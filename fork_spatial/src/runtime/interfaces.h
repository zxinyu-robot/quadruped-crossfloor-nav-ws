#ifndef FORK_SPATIAL_RUNTIME_INTERFACES_H
#define FORK_SPATIAL_RUNTIME_INTERFACES_H

#include "runtime/model.h"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace fork_spatial::runtime {

struct MeasurementEnvelope {
    std::string sensor_id;
    std::string schema;
    Timestamp timestamp_ns{};
    std::vector<std::byte> payload;
};

class ISensorAdapter {
public:
    virtual ~ISensorAdapter() = default;
    [[nodiscard]] virtual std::string sensor_id() const = 0;
    virtual bool decode(
        std::span<const std::byte> input,
        MeasurementEnvelope& output,
        std::string& error) = 0;
};

class IFactorProvider {
public:
    virtual ~IFactorProvider() = default;
    [[nodiscard]] virtual std::string name() const = 0;
    virtual FactorBatch consume(const MeasurementEnvelope& measurement) = 0;
};

// 实现必须保证 apply() 的原子性：失败时不得保留部分图修改。
class IOptimizerBackend {
public:
    virtual ~IOptimizerBackend() = default;
    virtual bool apply(const GraphTransaction& transaction, std::string& error) = 0;
    virtual EstimateSnapshot optimize(std::uint64_t graph_version) = 0;
    // 仅计算边缘化先验，不修改后端；实际变更由随后的 apply() 原子提交。
    virtual bool marginalize(
        std::span<const StateKey> states,
        FactorProposal& prior,
        std::string& error) = 0;
};

class IWindowPolicy {
public:
    virtual ~IWindowPolicy() = default;
    [[nodiscard]] virtual std::vector<StateKey> select_for_marginalization(
        std::span<const StateRecord> states,
        Timestamp watermark_ns) const = 0;
};

class FixedLagWindowPolicy final : public IWindowPolicy {
public:
    explicit FixedLagWindowPolicy(Timestamp lag_ns) : lag_ns_(lag_ns) {}

    [[nodiscard]] std::vector<StateKey> select_for_marginalization(
        std::span<const StateRecord> states,
        Timestamp watermark_ns) const override {
        std::vector<StateKey> selected;
        const Timestamp cutoff = watermark_ns - lag_ns_;
        for (const StateRecord& state : states) {
            if (state.lifecycle == StateLifecycle::Active &&
                state.key.timestamp_ns < cutoff &&
                state.key.type != StateType::SensorExtrinsic &&
                state.key.type != StateType::ClockOffset) {
                selected.push_back(state.key);
            }
        }
        return selected;
    }

private:
    Timestamp lag_ns_;
};

}  // namespace fork_spatial::runtime

#endif
