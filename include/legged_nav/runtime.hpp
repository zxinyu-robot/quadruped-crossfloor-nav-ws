#pragma once
/// @file runtime.hpp
/// @brief 端侧任务运行时：TaskIntent → 准入 → 状态门控 → 技能编排 → 执行 → 反馈。
/// 模型/上层决策只允许到达技能层；禁控字段在契约校验阶段即拒。
/// 线程安全：单实例非线程安全，调用方串行化 submit/cancel。

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "legged_nav/admission.hpp"
#include "legged_nav/executor.hpp"
#include "legged_nav/feedback.hpp"
#include "legged_nav/spatial_state.hpp"
#include "legged_nav/task_intent.hpp"

namespace legged_nav {

enum class TaskStatus {
    ACCEPTED,
    RUNNING,
    HOLDING,
    SUCCEEDED,
    FAILED,
    CANCELED,
    REJECTED,
};

const char* status_str(TaskStatus s);

struct TaskHandle {
    TaskIntent intent;
    TaskStatus status = TaskStatus::ACCEPTED;
    size_t skill_index = 0;
    std::string failure_reason;
    long long accepted_ns = 0;
    long long finished_ns = 0;
};

class TaskRuntime {
   public:
    /// @param forwarder 执行适配（借用所有权，生命周期须长于本对象）。
    /// @param clock_ns 时钟注入（测试可控）。
    explicit TaskRuntime(
        Forwarder& forwarder, IntentOptions opt = {},
        std::map<std::string, std::vector<std::pair<double, double>>> forbidden_areas = {},
        StateGate gate = {}, std::function<long long()> clock_ns = now_ns);

    /// 接收 TaskIntent JSON；返回最新 ExecutionFeedback（终态或受理事件）。
    std::map<std::string, std::string> submit(const std::string& intent_json,
                                              const SpatialState* state = nullptr);
    std::map<std::string, std::string> submit(const JsonValue& raw,
                                              const SpatialState* state = nullptr);

    /// 取消活动任务；幂等。
    std::map<std::string, std::string> cancel(const std::string& task_id,
                                              const std::string& reason = "user_cancel");

    /// 全部反馈事件留痕（可审计）。
    const std::vector<std::map<std::string, std::string>>& events() const { return events_; }
    const TaskHandle* active() const { return active_.get(); }

   private:
    void emit(const TaskHandle& h, const std::string& event, const std::string& detail = "",
              const std::map<std::string, std::string>& progress = {});
    void fail(TaskHandle& h, const std::string& reason);

    Forwarder& forwarder_;
    IntentOptions opt_;
    std::map<std::string, std::vector<std::pair<double, double>>> forbidden_areas_;
    StateGate gate_;
    std::function<long long()> clock_;
    TaskAdmission admission_;
    std::unique_ptr<TaskHandle> active_;
    std::vector<std::map<std::string, std::string>> events_;
};

}  // namespace legged_nav
