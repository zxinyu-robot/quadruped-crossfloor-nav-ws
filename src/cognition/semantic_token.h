#ifndef FORK_SPATIAL_COGNITION_SEMANTIC_TOKEN_H
#define FORK_SPATIAL_COGNITION_SEMANTIC_TOKEN_H

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "contract/spatial_state.h"
#include "runtime/model.h"

namespace fork_spatial::cognition {

using runtime::Timestamp;

enum class TokenKind {
    Scene,
    Frontier,
    Goal,
    Risk,
    Execution,
};

enum class TokenStatus {
    Valid,
    Stale,
    Evicted,
};

// 语义价值只影响决策偏好，不构成几何可执行许可（由度量层否决）。
struct SemanticAttributes {
    double traversability{0.0};
    double goal_relevance{0.0};
    double information_gain{0.0};
    double risk{0.0};
    double priority{0.0};
};

struct ResourceRef {
    std::uint32_t robot_id{};
    std::uint32_t session_id{};
    std::string asset_id;
    std::uint64_t revision{};
    std::string frame_id;
    std::uint64_t generation{};  // 上下文代数，见 contract::AssetRef
    std::uint64_t localization_epoch{};
    bool pose_anchored{false};

    contract::AssetRef as_asset_ref() const {
        contract::AssetRef ref;
        ref.robot_id = robot_id;
        ref.session_id = session_id;
        ref.asset_id = asset_id;
        ref.revision = revision;
        ref.generation = generation;
        ref.frame_id = frame_id;
        ref.localization_epoch = localization_epoch;
        ref.pose_anchored = pose_anchored;
        return ref;
    }
};

struct SemanticToken {
    std::uint64_t token_id{};
    TokenKind kind{TokenKind::Scene};
    SemanticAttributes attributes;
    ResourceRef resource;
    std::vector<double> anchor;  // 目标/frontier 的锚点坐标（resource.frame_id 下）
    Timestamp stamp_ns{};
    Timestamp ttl_ns{};
    double confidence{0.0};
    TokenStatus status{TokenStatus::Valid};
    std::uint64_t version{};
    std::uint32_t vocab_version{1};  // 词表版本：词表先当契约定（枚举），不先学；变更不改资产身份
    std::string source;              // 传感器/估计器/模型/人工等来源
    std::string verification_status; // unverified / verified / invalid
    std::string evidence_ref;        // rosbag、fragment log 或恢复证据引用
};

enum class UpsertCode {
    Inserted,
    Updated,
    RejectedStaleRevision,
    RejectedInvalid,
};

struct TokenStoreConfig {
    Timestamp eviction_grace_ns{5'000'000'000};
    double min_confidence{0.05};
};

// 资源生命周期：Valid →(ttl 过期)→ Stale →(grace 过期)→ Evicted；
// 会话重置 / 坐标系失效时整体作废。
class TokenStore {
public:
    explicit TokenStore(TokenStoreConfig config = {});

    UpsertCode upsert(SemanticToken token);
    std::size_t tick(Timestamp now_ns);
    std::size_t invalidate_session(std::uint32_t robot_id, std::uint32_t session_id);
    std::size_t invalidate_frame(const std::string& frame_id);
    // 代数切换（定位重置 / 跨楼层 / 换图）：同 asset 下 generation 落后的 Token 全部失效，
    // 不论 TTL 是否到期。
    std::size_t invalidate_generation(const contract::AssetRef& current);

    std::optional<SemanticToken> find(std::uint64_t token_id) const;
    std::vector<SemanticToken> valid_tokens() const;
    std::uint64_t store_version() const noexcept { return store_version_; }

private:
    TokenStoreConfig config_;
    std::unordered_map<std::uint64_t, SemanticToken> tokens_;
    std::unordered_map<std::uint64_t, Timestamp> stale_since_;
    // 逐出墓碑：id → {owner, 最后 revision, 最后 version}。
    // 只挡同一 (robot_id, session_id) 的更旧延迟观测复活；
    // 编码器跨会话复用 token_id，新会话不受旧墓碑约束。
    struct Tombstone {
        std::uint32_t robot_id{};
        std::uint32_t session_id{};
        std::uint64_t revision{};
        std::uint64_t version{};
    };
    std::unordered_map<std::uint64_t, Tombstone> tombstone_;
    std::uint64_t store_version_{};
};

struct FrontierObservation {
    std::uint64_t id{};
    double x{}, y{}, z{};
    double information_gain{};
    double traversability{};
};

struct GeometricObservation {
    ResourceRef map;
    Timestamp stamp_ns{};
    bool localization_valid{false};
    double localization_quality{0.0};
    double occupancy_ratio{0.0};
    double robot_x{}, robot_y{}, robot_z{};
    std::optional<std::array<double, 3>> goal;
    std::vector<FrontierObservation> frontiers;
    std::uint32_t recent_failures{};
};

class ITokenEncoder {
public:
    virtual ~ITokenEncoder() = default;
    virtual std::vector<SemanticToken> encode(const GeometricObservation& observation) = 0;
};

struct RuleEncoderConfig {
    Timestamp token_ttl_ns{2'000'000'000};
    double distance_scale_m{10.0};
};

// 规则化基线编码器：几何事实 → 语义价值 token；可替换为学习型编码器。
class RuleBasedTokenEncoder final : public ITokenEncoder {
public:
    explicit RuleBasedTokenEncoder(RuleEncoderConfig config = {});
    std::vector<SemanticToken> encode(const GeometricObservation& observation) override;

private:
    RuleEncoderConfig config_;
};

}  // namespace fork_spatial::cognition

#endif
