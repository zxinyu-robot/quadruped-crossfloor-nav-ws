#include "cognition/semantic_token.h"

#include <algorithm>
#include <cmath>

namespace fork_spatial::cognition {
namespace {

bool finite_unit(double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; }

bool attributes_valid(const SemanticAttributes& a) {
    return finite_unit(a.traversability) && finite_unit(a.goal_relevance) &&
           finite_unit(a.information_gain) && finite_unit(a.risk) && finite_unit(a.priority);
}

double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

constexpr std::uint64_t kSceneTokenId = 1;
constexpr std::uint64_t kGoalTokenId = 2;
constexpr std::uint64_t kRiskTokenId = 3;
constexpr std::uint64_t kFrontierTokenBase = 1'000;

}  // namespace

TokenStore::TokenStore(TokenStoreConfig config) : config_(config) {}

UpsertCode TokenStore::upsert(SemanticToken token) {
    if (!attributes_valid(token.attributes) || !finite_unit(token.confidence) ||
        token.confidence < config_.min_confidence || token.ttl_ns <= 0 ||
        token.resource.frame_id.empty()) {
        return UpsertCode::RejectedInvalid;
    }
    // 墓碑按 owner 隔离：只挡被逐出会话自身的更旧延迟观测复活；
    // 编码器跨会话复用 token_id，新会话不受旧墓碑约束
    auto tomb = tombstone_.find(token.token_id);
    const bool tomb_match =
        tomb != tombstone_.end() &&
        tomb->second.robot_id == token.resource.robot_id &&
        tomb->second.session_id == token.resource.session_id;
    if (tomb_match && token.resource.revision < tomb->second.revision) {
        return UpsertCode::RejectedStaleRevision;
    }
    auto it = tokens_.find(token.token_id);
    // 同 revision 乱序到达时按观测时戳取新者，防止旧 anchor/priority 回退
    if (it != tokens_.end() &&
        (token.resource.revision < it->second.resource.revision ||
         (token.resource.revision == it->second.resource.revision &&
          token.stamp_ns < it->second.stamp_ns))) {
        return UpsertCode::RejectedStaleRevision;
    }
    const bool inserted = it == tokens_.end();
    token.status = TokenStatus::Valid;
    // 同 owner 复活时延续 version；新 owner（新会话/新机器人）是独立谱系重计
    token.version = inserted
                        ? (tomb_match ? tomb->second.version + 1 : 1)
                        : it->second.version + 1;
    // 墓碑保留：每 id 仅一条、逐出时覆写，不随次数堆积；保留它才能在活 token
    // 属于新会话时继续挡住旧会话低 revision 的延迟观测
    stale_since_.erase(token.token_id);
    tokens_[token.token_id] = std::move(token);
    ++store_version_;
    return inserted ? UpsertCode::Inserted : UpsertCode::Updated;
}

std::size_t TokenStore::tick(Timestamp now_ns) {
    std::size_t changed = 0;
    for (auto it = tokens_.begin(); it != tokens_.end();) {
        const auto id = it->first;
        auto& token = it->second;
        if (token.status == TokenStatus::Valid &&
            now_ns - token.stamp_ns > token.ttl_ns) {
            token.status = TokenStatus::Stale;
            stale_since_[id] = now_ns;
            ++changed;
            ++it;
        } else if (token.status == TokenStatus::Stale &&
                   now_ns - stale_since_[id] > config_.eviction_grace_ns) {
            // 逐出即删除：记墓碑防旧观测复活并续 version，同时避免历史 token 无限堆积
            tombstone_[id] = {token.resource.robot_id, token.resource.session_id,
                              token.resource.revision, token.version};
            stale_since_.erase(id);
            it = tokens_.erase(it);
            ++changed;
        } else {
            ++it;
        }
    }
    if (changed > 0) {
        ++store_version_;
    }
    return changed;
}

std::size_t TokenStore::invalidate_session(std::uint32_t robot_id, std::uint32_t session_id) {
    std::size_t changed = 0;
    for (auto it = tokens_.begin(); it != tokens_.end();) {
        if (it->second.resource.robot_id == robot_id &&
            it->second.resource.session_id == session_id) {
            const auto& r = it->second.resource;
            tombstone_[it->first] = {r.robot_id, r.session_id, r.revision, it->second.version};
            stale_since_.erase(it->first);
            it = tokens_.erase(it);
            ++changed;
        } else {
            ++it;
        }
    }
    if (changed > 0) {
        ++store_version_;
    }
    return changed;
}

std::size_t TokenStore::invalidate_frame(const std::string& frame_id) {
    std::size_t changed = 0;
    for (auto it = tokens_.begin(); it != tokens_.end();) {
        if (it->second.resource.frame_id == frame_id) {
            const auto& r = it->second.resource;
            tombstone_[it->first] = {r.robot_id, r.session_id, r.revision, it->second.version};
            stale_since_.erase(it->first);
            it = tokens_.erase(it);
            ++changed;
        } else {
            ++it;
        }
    }
    if (changed > 0) {
        ++store_version_;
    }
    return changed;
}

std::size_t TokenStore::invalidate_generation(const contract::AssetRef& current) {
    std::size_t changed = 0;
    for (auto& [id, token] : tokens_) {
        const auto& r = token.resource;
        const bool same_asset = r.robot_id == current.robot_id &&
                                r.session_id == current.session_id &&
                                r.asset_id == current.asset_id;
        if (token.status != TokenStatus::Evicted && same_asset &&
            r.generation < current.generation) {
            token.status = TokenStatus::Evicted;
            stale_since_.erase(id);
            ++changed;
        }
    }
    if (changed > 0) {
        ++store_version_;
    }
    return changed;
}

std::optional<SemanticToken> TokenStore::find(std::uint64_t token_id) const {
    auto it = tokens_.find(token_id);
    if (it == tokens_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<SemanticToken> TokenStore::valid_tokens() const {
    std::vector<SemanticToken> out;
    for (const auto& [id, token] : tokens_) {
        if (token.status == TokenStatus::Valid) {
            out.push_back(token);
        }
    }
    std::sort(out.begin(), out.end(),
              [](const auto& a, const auto& b) { return a.token_id < b.token_id; });
    return out;
}

RuleBasedTokenEncoder::RuleBasedTokenEncoder(RuleEncoderConfig config) : config_(config) {}

std::vector<SemanticToken> RuleBasedTokenEncoder::encode(const GeometricObservation& obs) {
    std::vector<SemanticToken> out;
    const double loc_conf = obs.localization_valid ? clamp01(obs.localization_quality) : 0.0;
    auto make = [&](std::uint64_t id, TokenKind kind) {
        SemanticToken t;
        t.token_id = id;
        t.kind = kind;
        t.resource = obs.map;
        t.stamp_ns = obs.stamp_ns;
        t.ttl_ns = config_.token_ttl_ns;
        t.confidence = loc_conf;
        return t;
    };

    auto scene = make(kSceneTokenId, TokenKind::Scene);
    scene.attributes.traversability = clamp01(1.0 - obs.occupancy_ratio);
    scene.attributes.risk = clamp01(obs.occupancy_ratio);
    scene.attributes.priority = 0.5;
    out.push_back(scene);

    auto risk = make(kRiskTokenId, TokenKind::Risk);
    risk.attributes.risk = clamp01((1.0 - loc_conf) * 0.5 + 0.15 * obs.recent_failures);
    risk.attributes.priority = risk.attributes.risk;
    out.push_back(risk);

    double best_gain = 0.0;
    for (const auto& f : obs.frontiers) {
        best_gain = std::max(best_gain, f.information_gain);
    }

    for (const auto& f : obs.frontiers) {
        auto tok = make(kFrontierTokenBase + f.id, TokenKind::Frontier);
        const double dist = std::hypot(f.x - obs.robot_x, f.y - obs.robot_y, f.z - obs.robot_z);
        const double proximity = std::exp(-dist / config_.distance_scale_m);
        tok.anchor = {f.x, f.y, f.z};
        tok.attributes.information_gain = best_gain > 0.0 ? clamp01(f.information_gain / best_gain) : 0.0;
        tok.attributes.traversability = clamp01(f.traversability);
        tok.attributes.priority = clamp01(0.6 * tok.attributes.information_gain + 0.4 * proximity);
        out.push_back(tok);
    }

    if (obs.goal) {
        auto goal = make(kGoalTokenId, TokenKind::Goal);
        const auto& g = *obs.goal;
        const double dist = std::hypot(g[0] - obs.robot_x, g[1] - obs.robot_y, g[2] - obs.robot_z);
        goal.anchor = {g[0], g[1], g[2]};
        goal.attributes.goal_relevance = 1.0;
        goal.attributes.priority = clamp01(0.5 + 0.5 * std::exp(-dist / config_.distance_scale_m));
        out.push_back(goal);
    }
    return out;
}

}  // namespace fork_spatial::cognition
