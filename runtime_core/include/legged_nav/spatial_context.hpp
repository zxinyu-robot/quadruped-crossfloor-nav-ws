#pragma once
/// @file spatial_context.hpp
/// @brief 统一空间状态契约管理 + StreamVLN 式 Fast-Slow 流式上下文。

#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>

#include "legged_nav/executor.hpp"
#include "legged_nav/spatial_state.hpp"

namespace legged_nav {

/// 空间状态条目：版本化 + 有效期 + 置信度。
struct SpatialEntry {
    std::string key;  ///< pose / map / goal / task_state
    SpatialState state;
    std::optional<std::map<std::string, double>> goal;
    int revision = 0;
    long long ttl_ns = 500'000'000LL;
    long long updated_ns = 0;

    bool is_valid(long long now_ns_) const {
        return state.valid && (now_ns_ - updated_ns) <= ttl_ns;
    }
};

/// 版本化空间状态存储：快照 + 增量更新；失效契约不下发。
class SpatialStateStore {
   public:
    SpatialEntry& update(const std::string& key, const SpatialState* state = nullptr,
                         const std::map<std::string, double>* goal = nullptr, long long ttl_ns = -1,
                         long long now_ns_ = -1);

    /// 只返回未过期且有效的条目。
    std::map<std::string, SpatialEntry> entries(long long now_ns_ = -1) const;
    std::map<std::string, SpatialEntry> snapshot() const { return entries_; }
    int revision() const { return revision_; }

   private:
    std::map<std::string, SpatialEntry> entries_;
    int revision_ = 0;
};

/// 一帧视觉观测的 Token 单元（fast 窗口条目）。
struct VisionToken {
    std::string frame_id;
    std::string summary;                      ///< 语义摘要（场景/地标/危险）
    std::tuple<double, double, double> pose;  ///< LIO 位姿（世界系，米）
    long long timestamp_ns = 0;
    bool kv_fresh = true;  ///< true=增量需推理
};

/// 压缩后的历史记忆：场景直方图 + 地标集合 + 里程累计。
struct SlowMemory {
    int compressed_count = 0;
    std::map<std::string, int> scene_histogram;
    std::set<std::string> landmark_set;
    double odom_m = 0.0;
    std::optional<std::tuple<double, double, double>> last_pose;
};

/// Fast-Slow Context：滑动窗口 + KV 增量复用 + LIO 位姿 Token 去重。
class StreamContext {
   public:
    explicit StreamContext(size_t window = 8, double dedup_dist_m = 0.3)
        : window_(window), dedup_dist_m_(dedup_dist_m) {}

    /// 摄入一帧观测 Token；true=入窗（KV 增量），false=去重丢弃。
    bool push(const VisionToken& token);

    /// 显式压缩最旧条目进 slow 记忆。
    std::optional<VisionToken> compress_oldest();

    /// 当前推理上下文（slow 摘要不占 token 预算）。
    std::vector<VisionToken> context_tokens() const {
        return std::vector<VisionToken>(fast_.begin(), fast_.end());
    }

    /// 上下文开销统计。
    std::map<std::string, double> stats() const;
    const SlowMemory& slow() const { return slow_; }

   private:
    double dist(const std::tuple<double, double, double>& a,
                const std::tuple<double, double, double>& b) const;
    void compress(const VisionToken& evicted);

    size_t window_;
    double dedup_dist_m_;
    std::deque<VisionToken> fast_;
    SlowMemory slow_;
    size_t kv_cached_ = 0;
    size_t dropped_dup_ = 0;
};

}  // namespace legged_nav
