#include "runtime/state_store.h"

#include <functional>
#include <utility>

namespace fork_spatial::runtime {
namespace {

void hash_combine(std::size_t& seed, std::size_t value) {
    seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

}  // namespace

std::size_t StateKeyHash::operator()(const StateKey& key) const noexcept {
    std::size_t seed = 0;
    hash_combine(seed, std::hash<std::uint32_t>{}(key.robot_id));
    hash_combine(seed, std::hash<std::uint32_t>{}(key.session_id));
    hash_combine(seed, std::hash<int>{}(static_cast<int>(key.type)));
    hash_combine(seed, std::hash<Timestamp>{}(key.timestamp_ns));
    hash_combine(seed, std::hash<std::uint32_t>{}(key.instance));
    return seed;
}

bool StateStore::add(StateRecord state, std::string& error) {
    if (states_.contains(state.key)) {
        error = "state key already exists";
        return false;
    }
    states_.emplace(state.key, std::move(state));
    return true;
}

bool StateStore::transition(
    const StateKey& key, StateLifecycle target, std::string& error) {
    const auto iterator = states_.find(key);
    if (iterator == states_.end()) {
        error = "state key does not exist";
        return false;
    }
    if (!can_transition(iterator->second.lifecycle, target)) {
        error = "invalid state lifecycle transition";
        return false;
    }
    iterator->second.lifecycle = target;
    ++iterator->second.version;
    return true;
}

const StateRecord* StateStore::find(const StateKey& key) const {
    const auto iterator = states_.find(key);
    return iterator == states_.end() ? nullptr : &iterator->second;
}

bool StateStore::factor_eligible(const StateKey& key) const {
    const StateRecord* state = find(key);
    return state != nullptr &&
           (state->lifecycle == StateLifecycle::Active ||
            state->lifecycle == StateLifecycle::Fixed);
}

std::vector<StateRecord> StateStore::records() const {
    std::vector<StateRecord> result;
    result.reserve(states_.size());
    for (const auto& [key, state] : states_) {
        (void)key;
        result.push_back(state);
    }
    return result;
}

bool StateStore::can_transition(StateLifecycle from, StateLifecycle to) {
    if (from == to) {
        return true;
    }
    switch (from) {
        case StateLifecycle::Requested:
            return to == StateLifecycle::Initialized;
        case StateLifecycle::Initialized:
            return to == StateLifecycle::Active || to == StateLifecycle::Archived;
        case StateLifecycle::Active:
            return to == StateLifecycle::Fixed ||
                   to == StateLifecycle::Marginalized;
        case StateLifecycle::Fixed:
            return to == StateLifecycle::Active ||
                   to == StateLifecycle::Marginalized;
        case StateLifecycle::Marginalized:
            return to == StateLifecycle::Archived;
        case StateLifecycle::Archived:
            return false;
    }
    return false;
}

}  // namespace fork_spatial::runtime
