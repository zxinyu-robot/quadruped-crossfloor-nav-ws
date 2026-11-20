#pragma once
/// @file transport.hpp
/// @brief 通信中间件抽象：Cyclone DDS + SHM + DDSI-RTPS/UDP 组合传输。
/// 通道按数据域命名；QoS 分级（高频传感 best_effort，意图/反馈/控制 reliable）。
/// InProcessTransport 离线/CI 可测；Cyclone DDS 后端为部署侧实现（链接 cyclonedds C 库）。

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "legged_nav/executor.hpp"
#include "legged_nav/json.hpp"

namespace legged_nav {

enum class QoS { BEST_EFFORT, RELIABLE };

struct Channel {
    std::string name;
    QoS qos;
    std::string schema;
    std::string desc;
};

/// 数据链路通道表：多源传感器 / 空间状态 / 模型推理 / 决策 / 运动控制 / 反馈。
const std::map<std::string, Channel>& channels();

struct Message {
    std::string channel;
    JsonValue payload;
    long long timestamp_ns = 0;
};

/// 扫描 payload 任意层级的禁控字段；返回错误码或空串。
std::string scan_forbidden_payload(const JsonValue& v);

/// 传输后端协议。
class Transport {
   public:
    virtual ~Transport() = default;
    /// 发布；返回 std::nullopt 或 REJECT_* 拒绝码。
    virtual std::optional<std::string> publish(const std::string& channel,
                                               const JsonValue& payload) = 0;
    virtual void subscribe(const std::string& channel, std::function<void(const Message&)> cb) = 0;
};

/// 进程内后端：同步分发 + 留痕（合成数据·单元测试级）。
class InProcessTransport : public Transport {
   public:
    std::optional<std::string> publish(const std::string& channel,
                                       const JsonValue& payload) override;
    void subscribe(const std::string& channel, std::function<void(const Message&)> cb) override {
        subs_[channel].push_back(std::move(cb));
    }
    const std::vector<Message>& log() const { return log_; }

   private:
    std::map<std::string, std::vector<std::function<void(const Message&)>>> subs_;
    std::vector<Message> log_;
};

}  // namespace legged_nav
