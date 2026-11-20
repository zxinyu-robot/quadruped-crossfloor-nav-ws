#include "legged_nav/spatial_context.hpp"

#include <cmath>

namespace legged_nav {

SpatialEntry& SpatialStateStore::update(const std::string& key, const SpatialState* state,
                                        const std::map<std::string, double>* goal, long long ttl_ns,
                                        long long now_ns_) {
    ++revision_;
    auto& e = entries_[key];
    e.key = key;
    if (state) e.state = *state;
    if (goal) e.goal = *goal;
    if (ttl_ns >= 0) e.ttl_ns = ttl_ns;
    e.revision = revision_;
    e.updated_ns = now_ns_ >= 0 ? now_ns_ : now_ns();
    return e;
}

std::map<std::string, SpatialEntry> SpatialStateStore::entries(long long now_ns_) const {
    long long now = now_ns_ >= 0 ? now_ns_ : now_ns();
    std::map<std::string, SpatialEntry> out;
    for (const auto& [k, e] : entries_)
        if (e.is_valid(now)) out.emplace(k, e);
    return out;
}

double StreamContext::dist(const std::tuple<double, double, double>& a,
                           const std::tuple<double, double, double>& b) const {
    return std::sqrt(std::pow(std::get<0>(a) - std::get<0>(b), 2) +
                     std::pow(std::get<1>(a) - std::get<1>(b), 2) +
                     std::pow(std::get<2>(a) - std::get<2>(b), 2));
}

bool StreamContext::push(const VisionToken& token) {
    if (!fast_.empty()) {
        double best = std::numeric_limits<double>::max();
        for (const auto& t : fast_) best = std::min(best, dist(t.pose, token.pose));
        if (best < dedup_dist_m_) {
            ++dropped_dup_;
            return false;
        }
    }
    VisionToken t = token;
    t.kv_fresh = true;
    ++kv_cached_;
    fast_.push_back(t);
    while (fast_.size() > window_) {
        compress(fast_.front());
        fast_.pop_front();
    }
    if (slow_.last_pose) slow_.odom_m += dist(*slow_.last_pose, token.pose);
    slow_.last_pose = token.pose;
    return true;
}

void StreamContext::compress(const VisionToken& evicted) {
    ++slow_.compressed_count;
    size_t start = 0;
    const std::string& s = evicted.summary;
    while (start <= s.size()) {
        size_t comma = s.find(',', start);
        std::string tag =
            s.substr(start, comma == std::string::npos ? s.size() - start : comma - start);
        if (!tag.empty()) ++slow_.scene_histogram[tag];
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
}

std::optional<VisionToken> StreamContext::compress_oldest() {
    if (fast_.empty()) return std::nullopt;
    VisionToken evicted = fast_.front();
    fast_.pop_front();
    compress(evicted);
    return evicted;
}

std::map<std::string, double> StreamContext::stats() const {
    return {
        {"fast_len", static_cast<double>(fast_.size())},
        {"window", static_cast<double>(window_)},
        {"kv_cached", static_cast<double>(kv_cached_)},
        {"dedup_dropped", static_cast<double>(dropped_dup_)},
        {"slow_compressed", static_cast<double>(slow_.compressed_count)},
        {"slow_odom_m", slow_.odom_m},
        {"kv_reuse_ratio", kv_cached_ ? 1.0 - static_cast<double>(fast_.size()) / kv_cached_ : 0.0},
    };
}

}  // namespace legged_nav
