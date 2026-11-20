#include "legged_nav/admission.hpp"

namespace legged_nav {

AdmitResult TaskAdmission::decide(const TaskIntent& intent, const TaskIntent* active) const {
    if (active && intent.task_id == active->task_id) return AdmitDecision::IDEMPOTENT;
    if (seen_.count(intent.task_id)) return std::string("REJECT_DUPLICATE_TASK_ID");
    if (active) {
        if (active->preemptible && intent.priority > active->priority)
            return AdmitDecision::PREEMPT;
        return std::string("REJECT_BUSY");
    }
    return AdmitDecision::ACCEPT;
}

void TaskAdmission::record(const std::string& task_id) {
    seen_.insert(task_id);
    order_.push_back(task_id);
    while (order_.size() > history_) {
        seen_.erase(order_.front());
        order_.pop_front();
    }
}

}  // namespace legged_nav
