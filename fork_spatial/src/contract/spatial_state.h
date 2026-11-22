#ifndef FORK_SPATIAL_CONTRACT_SPATIAL_STATE_H
#define FORK_SPATIAL_CONTRACT_SPATIAL_STATE_H

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace fork_spatial::contract {

// 空间状态契约：三个仓库（MotionSLAM_ws / legged-navigation-runtime / 本仓库）
// 共用的"身份·版本·时间·可信度·原因码"。只定义字段与规则，不定义传输。
// 各仓库现有字段到本契约的映射见 docs/contracts/spatial_state.md。

// ---------- 1. 身份与版本链 ----------
// revision = 资产内容版本（MotionSLAM map_version / layer_version）。
// generation = 有效期代数（MotionSLAM context_generation / SHM generation）；
//              generation 变化意味着旧状态、旧子目标、旧轨迹全部作废。
// frame_id 必填、无默认值：Go2 默认 world、规划器默认 map，不能隐式假定。
struct AssetRef {
    std::uint32_t robot_id{};
    std::uint32_t session_id{};
    std::string asset_id;          // 建议编码 building/floor/map/tile
    std::uint64_t revision{};
    std::uint64_t generation{};
    std::string frame_id;
    std::string asset_type;        // occupancy / octvox / tile / pose_graph …（可选）
    std::uint64_t content_hash{};  // 已提交内容的校验，0 = 未提供
    // localization_epoch 与 generation 分开：generation 变 = 换图/楼层切换，该 asset 下
    // 全部状态作废；localization_epoch 变 = 定位重置，只作废位姿锚定的状态。
    std::uint64_t localization_epoch{};
    bool pose_anchored{false};     // 位姿锚定（子目标/轨迹/位姿 Token），定位重置后需重新锚定

    bool operator==(const AssetRef&) const = default;
};

// 同一 asset_id 下，生命周期上 b 是否仍可消费为 a 的同代状态。
[[nodiscard]] bool same_generation(const AssetRef& a, const AssetRef& b) noexcept;
[[nodiscard]] bool validate(const AssetRef& ref, std::string& error);

// ---------- 1b. 增量同步 ----------
// 身份(asset_id) / 版本(revision) / 传输编码(codec) 三者严格分开：Token 流只是编码，
// 不承担资产身份。增量必须声明基线版本；消费方基线不符不得盲目应用，先补齐或要快照。
struct DeltaRef {
    std::string asset_id;
    std::uint64_t base_revision{};
    std::uint64_t target_revision{};
    std::uint64_t generation{};
    std::string codec_id;      // raw_diff / quantized / token_stream …
    std::uint64_t checksum{};  // 解码后校验，0 = 未提供
};

enum class DeltaDisposition : std::uint8_t {
    Apply,            // base == 本地 revision
    AlreadyApplied,   // target <= 本地 revision
    MissingDeltas,    // 本地落后于 base 或与区间交叠 → 补齐缺失增量或请求快照
    StaleGeneration,  // asset / generation 不匹配 → 丢弃并重同步
};
[[nodiscard]] DeltaDisposition dispose(const DeltaRef& delta, const AssetRef& local) noexcept;

// ---------- 2. 时间 ----------
enum class ClockDomain : std::uint8_t {
    Monotonic,   // 端侧单机节拍（steady_clock / ROS steady）
    WallUtc,     // 跨机/跨进程对时后的 UTC
};

struct Stamp {
    std::int64_t stamp_ns{};
    std::int64_t valid_until_ns{};   // 0 = 不设有效期
    ClockDomain clock{ClockDomain::Monotonic};

    [[nodiscard]] bool expired_at(std::int64_t now_ns) const noexcept {
        return valid_until_ns != 0 && now_ns > valid_until_ns;
    }
};

// ---------- 3. 执行令牌与任务契约（Intent Execution Contract）----------
// 绑定机器人的可追溯链：每条命令可查谁签发(owner)、第几任(epoch)、基于哪版状态、何时过期。
// epoch 是防过期控制者的关键：接管一次加一，低 epoch 的命令一律拒绝（随机 UUID 做不到这点）。
struct ExecutionLease {
    std::string task_id;
    std::string owner_id;      // 签发者：边缘节点 / 云 / 本机端侧调度
    std::uint32_t robot_id{};
    std::uint64_t epoch{};
    std::int64_t issued_ns{};
    std::int64_t expiry_ns{};           // 0 = 不设
    std::int64_t start_deadline_ns{};   // 最晚开始执行时间，0 = 不限
    std::int64_t execution_timeout_ns{};// 开始执行后的最长运行时长，0 = 不限
    ClockDomain clock{ClockDomain::Monotonic};
    std::int64_t max_skew_ns{};         // 签发方与执行方时钟允许的最大偏差
    AssetRef state;            // 基于哪版空间状态签发

    ExecutionLease() = default;
    // PR #7 兼容构造：旧调用只提供任务、租约与状态引用。
    ExecutionLease(std::string task, std::string owner, std::uint32_t robot,
                   std::uint64_t lease_epoch, std::int64_t issued,
                   std::int64_t expiry, AssetRef lease_state)
        : task_id(std::move(task)), owner_id(std::move(owner)), robot_id(robot),
          epoch(lease_epoch), issued_ns(issued), expiry_ns(expiry), state(std::move(lease_state)) {}

    bool operator==(const ExecutionLease&) const = default;
};

enum class LeaseDisposition : std::uint8_t {
    Accept,
    WrongRobot,
    StaleEpoch,     // 比当前已接受的 epoch 低 → 旧执行者，拒绝
    Expired,
    StateMismatch,  // 签发所依据的状态代数与本地不符
    ClockMismatch,  // 时钟域不一致，无法比较过期时间
    DeadlineMissed, // 未在 start_deadline 开始，或运行超过 execution_timeout
};
[[nodiscard]] LeaseDisposition dispose(const ExecutionLease& lease, std::uint32_t robot_id,
                                       std::uint64_t current_epoch, const AssetRef& local,
                                       ClockDomain local_clock,
                                       std::int64_t now_ns) noexcept;
[[nodiscard]] LeaseDisposition dispose_running(const ExecutionLease& lease,
                                               std::uint32_t robot_id,
                                               std::uint64_t current_epoch,
                                               const AssetRef& local,
                                               ClockDomain local_clock,
                                               std::int64_t started_ns,
                                               std::int64_t now_ns) noexcept;
// PR #7 compatibility overload: existing monotonic callers need no migration step.
[[nodiscard]] LeaseDisposition dispose(const ExecutionLease& lease, std::uint32_t robot_id,
                                       std::uint64_t current_epoch, const AssetRef& local,
                                       std::int64_t now_ns) noexcept;

// 下行任务 = intent_id（追踪）+ idempotency_key（去重）+ 租约（授权）。
// 幂等规则：同键同内容 → 返回原任务/原结果；同键不同内容 → 拒绝并记录冲突。
struct IntentEnvelope {
    std::string intent_id;
    std::string idempotency_key;
    std::uint32_t schema_version{1};
    ExecutionLease lease;

    bool operator==(const IntentEnvelope&) const = default;
};

enum class IdempotencyDisposition : std::uint8_t {
    New,                // 新任务
    DuplicateSame,      // 同键同内容 → 返回原任务/原结果
    ConflictDifferent,  // 同键不同内容 → 拒绝（ReasonCode::IdempotencyConflict）
};
// stored 为已保存的同键请求；nullptr = 无记录。
[[nodiscard]] IdempotencyDisposition dispose_idempotency(
    const IntentEnvelope& incoming, const IntentEnvelope* stored) noexcept;

// 上行数据分类：弱网传输调度器按类别定优先级，类内再按熵/字节/时效排序。
// DurableEvent 是关键事件（急停/严重故障/任务终止/执行权变化/进入恢复），
// 不能被后来的最新值覆盖，否则 ESN 状态语言与证据链丢历史。
enum class UplinkClass : std::uint8_t {
    DurableEvent,
    LatestValue,
    Bulk,
};

enum class FeedbackStatus : std::uint8_t {
    Accepted,
    Running,
    Blocked,
    Recovering,
    Succeeded,
    Failed,
    Canceled,
    Preempted,
};

// ---------- 4. 原因码（唯一一套，替代 mission::FailureCode 与 failure_code 字符串） ----------
enum class ReasonCode : std::uint8_t {
    None,
    NoWaypoints,
    PlanFailed,            // 单次规划失败
    PlanFailedRepeated,    // 连续规划失败超阈值
    ReplanTimeout,         // 规划器在期限内未产出
    Stuck,                 // 位姿无进展
    LocalizationLost,      // 定位失效
    LocalizationJump,      // 漂移 / 跳变超阈值
    RelocalizationTimeout, // 重定位搜索或配准超时
    FloorTransition,       // 楼层 / 地图切换中
    AssetStale,            // generation 不匹配，旧状态被拒
    LinkLost,              // 断链（仅上报；端侧继续本地自主）
    SafetyVeto,            // 度量层否决（含类别无关的障碍、原玻璃分支）
    ResyncRequired,        // 增量基线缺失 / 校验失败，需补齐或快照
    Timeout,               // 任务级超时
    Canceled,
    LeaseExpired,          // 执行令牌过期
    DeadlineMissed,        // 错过开始截止或运行超时
    Preempted,             // 被更高优先级任务抢占
    IdempotencyConflict,   // 同幂等键不同内容
};
[[nodiscard]] std::string_view name(ReasonCode code) noexcept;

// ---------- 5. 空间状态可信度 ----------
// 合并：MotionSLAM 规划§6 的 NORMAL_3D/DEGRADED_2D/RELOCALIZING/VERIFYING/HOLD、
// task_runtime::SafetyMode、legged-navigation RecoveryState、Go2 /lio/* 话题。
enum class LocalizationMode : std::uint8_t {
    Normal3D,
    Degraded2D,
    Relocalizing,
    Verifying,
    Hold,
};
[[nodiscard]] std::string_view name(LocalizationMode mode) noexcept;

// 可信度决定 Runtime 还给执行层多少授权；这是"可信度驱动任务恢复"的核心规则。
enum class ExecutionAuthority : std::uint8_t {
    Full,        // Normal3D：正常派发
    Restricted,  // Degraded2D：限速 / 限域，仍可推进
    Suspended,   // Relocalizing / Verifying：StopTrajectory，任务保留，等恢复
    Revoked,     // Hold：收回授权，任务进 Holding，需显式恢复
};
[[nodiscard]] ExecutionAuthority authority_for(LocalizationMode mode) noexcept;

struct LocalizationHealth {
    LocalizationMode mode{LocalizationMode::Normal3D};
    double quality{1.0};            // [0,1]，喂 Token.confidence / localization_quality
    double pose_jump_m{0.0};        // ← LNR pose_jump_m / Go2 drift_estimate
    double alignment_fitness{0.0};  // ← LNR alignment_fitness / Go2 /lio/relocation/fitness
    std::uint32_t healthy_frames{}; // ← LNR healthy_frames_to_resume 计数
    bool stance_ok{true};           // ← Go2 /local_map/stance_ok
    // 重定位进度（LNR Searching→Aligning→Verifying 需要）：
    bool candidate_found{false};    // ← Go2 /lio/relocation 候选 / LNR descriptor_candidate_found
    bool alignment_complete{false}; // ← 精配准已结束（无论成败）
    bool alignment_success{false};  // ← 精配准收敛（fitness 达标另由 alignment_fitness 判）
    ReasonCode reason{ReasonCode::None};

    [[nodiscard]] ExecutionAuthority authority() const noexcept { return authority_for(mode); }
    [[nodiscard]] bool usable() const noexcept {
        return mode == LocalizationMode::Normal3D || mode == LocalizationMode::Degraded2D;
    }
};

// 一条带可信度和版本的空间状态观测——所有上行消息的最小公共头。
struct SpatialStateHeader {
    AssetRef asset;
    Stamp stamp;
    LocalizationHealth health;
};

// 版本/时效/可信度不匹配分三种处理，不是一律拒绝：
//   Reverify = 定位重置等仍可确定性重锚定的 → 允许重新验证/重规划；
//   Hold     = 无法确认匹配（定位不可用/处于 Hold）→ 暂停并进入恢复；
//   Reject   = 语义失效（generation 不符/已过期）→ 拒绝。
enum class AdmitDisposition : std::uint8_t {
    Accept,
    Reverify,
    Hold,
    Reject,
};
struct AdmitResult {
    AdmitDisposition disposition{AdmitDisposition::Accept};
    ReasonCode reason{ReasonCode::None};

    // 保留 PR #7 调用方的原因码比较方式，同时让新调用方读取处理动作。
    friend constexpr bool operator==(const AdmitResult& result, ReasonCode code) noexcept {
        return result.reason == code;
    }
    friend constexpr bool operator==(ReasonCode code, const AdmitResult& result) noexcept {
        return result == code;
    }
};
[[nodiscard]] AdmitResult admit(const SpatialStateHeader& header,
                                const AssetRef& expected,
                                std::int64_t now_ns) noexcept;

// ---------- 6. 恢复结果 ----------
// "可信度 → 恢复动作 → 恢复结果验证"的最后一段；喂给经验库。
struct RecoveryOutcome {
    ReasonCode trigger{ReasonCode::None};  // 进入恢复的原因
    std::string action;                     // 实际执行的恢复动作
    std::int64_t duration_ns{0};            // 恢复耗时
    double cost{0.0};                       // 恢复代价（时间/行程折算的标量，先占位）
    bool resumed{false};                    // 恢复后状态是否满足继续执行条件
    AssetRef post_state;                    // 恢复后用于继续执行的状态版本
};

}  // namespace fork_spatial::contract

#endif
