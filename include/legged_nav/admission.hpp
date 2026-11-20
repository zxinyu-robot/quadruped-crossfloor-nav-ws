#pragma once
/// @file admission.hpp
/// @brief 任务准入：task_id 幂等 + 抢占（有界历史）。

#include <deque>
#include <optional>
#include <string>
#include <unordered_set>

#include "legged_nav/task_intent.hpp"

namespace legged_nav {

enum class AdmitDecision { ACCEPT, IDEMPOTENT, PREEMPT };

/// decide 返回 AdmitDecision 或 REJECT_* 字符串。
using AdmitResult = std::variant<AdmitDecision, std::string>;

class TaskAdmission {
   public:
    explicit TaskAdmission(size_t history = 256) : history_(history) {}

    /// 幂等/抢占/忙拒裁决。
    /// - 同 task_id 活动任务 → IDEMPOTENT；
    /// - 受理过的 task_id → "REJECT_DUPLICATE_TASK_ID"；
    /// - 忙态：活动任务 preemptible 且新任务优先级更高 → PREEMPT，否则 REJECT_BUSY。
    AdmitResult decide(const TaskIntent& intent, const TaskIntent* active) const;

    /// 受理登记；历史容量有界（FIFO 淘汰）。
    void record(const std::string& task_id);

   private:
    size_t history_;
    std::unordered_set<std::string> seen_;
    std::deque<std::string> order_;
};

}  // namespace legged_nav
