#ifndef FORK_SPATIAL_RUNTIME_STATE_STORE_H
#define FORK_SPATIAL_RUNTIME_STATE_STORE_H

#include "runtime/model.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fork_spatial::runtime {

class StateStore {
public:
    bool add(StateRecord state, std::string& error);
    bool transition(const StateKey& key, StateLifecycle target, std::string& error);

    [[nodiscard]] const StateRecord* find(const StateKey& key) const;
    [[nodiscard]] bool factor_eligible(const StateKey& key) const;
    [[nodiscard]] std::vector<StateRecord> records() const;

private:
    static bool can_transition(StateLifecycle from, StateLifecycle to);

    std::unordered_map<StateKey, StateRecord, StateKeyHash> states_;
};

}  // namespace fork_spatial::runtime

#endif
