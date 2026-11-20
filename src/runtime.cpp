#include "legged_nav/runtime.hpp"

#include "legged_nav/json.hpp"
#include "legged_nav/skills.hpp"

namespace legged_nav {

const char* status_str(TaskStatus s) {
    switch (s) {
        case TaskStatus::ACCEPTED:
            return "accepted";
        case TaskStatus::RUNNING:
            return "running";
        case TaskStatus::HOLDING:
            return "holding";
        case TaskStatus::SUCCEEDED:
            return "succeeded";
        case TaskStatus::FAILED:
            return "failed";
        case TaskStatus::CANCELED:
            return "canceled";
        case TaskStatus::REJECTED:
            return "rejected";
    }
    return "unknown";
}

TaskRuntime::TaskRuntime(
    Forwarder& forwarder, IntentOptions opt,
    std::map<std::string, std::vector<std::pair<double, double>>> forbidden_areas, StateGate gate,
    std::function<long long()> clock)
    : forwarder_(forwarder),
      opt_(std::move(opt)),
      forbidden_areas_(std::move(forbidden_areas)),
      gate_(gate),
      clock_(std::move(clock)) {}

void TaskRuntime::emit(const TaskHandle& h, const std::string& event, const std::string& detail,
                       const std::map<std::string, std::string>& progress) {
    events_.push_back(
        build_feedback(h.intent.task_id, status_str(h.status), event, progress, detail, clock_()));
}

void TaskRuntime::fail(TaskHandle& h, const std::string& reason) {
    h.status = TaskStatus::FAILED;
    h.failure_reason = reason;
    h.finished_ns = clock_();
    emit(h, "failed", reason);
    active_.reset();
}

std::map<std::string, std::string> TaskRuntime::submit(const std::string& intent_json,
                                                       const SpatialState* state) {
    auto v = parse_json(intent_json);
    JsonValue raw = v ? std::move(*v) : JsonValue();
    return submit(raw, state);
}

std::map<std::string, std::string> TaskRuntime::submit(const JsonValue& raw,
                                                       const SpatialState* state) {
    long long now = clock_();
    auto result = validate_task_intent(raw, opt_);
    if (std::holds_alternative<std::string>(result)) {
        std::string tid = raw.is_obj() && raw.get("task_id") && raw.get("task_id")->is_str()
                              ? raw.get("task_id")->str()
                              : "";
        auto fb = build_feedback(tid, status_str(TaskStatus::REJECTED), "rejected", {},
                                 std::get<std::string>(result), now);
        events_.push_back(fb);
        return fb;
    }

    TaskHandle h{std::get<TaskIntent>(result), TaskStatus::ACCEPTED, 0, "", now, 0};

    auto area_err = check_forbidden_areas(h.intent, forbidden_areas_);
    if (area_err) {
        h.status = TaskStatus::REJECTED;
        emit(h, "rejected", *area_err);
        return events_.back();
    }

    auto decision = admission_.decide(h.intent, active_ ? &active_->intent : nullptr);
    if (auto* d = std::get_if<AdmitDecision>(&decision)) {
        if (*d == AdmitDecision::IDEMPOTENT) {
            emit(h, "idempotent");
            return events_.back();
        }
        if (*d == AdmitDecision::PREEMPT) {
            active_->status = TaskStatus::CANCELED;
            forwarder_.abort("preempted_by:" + h.intent.task_id);
            emit(*active_, "preempted");
            active_.reset();
        }
        // ACCEPT fallthrough
    } else {
        h.status = TaskStatus::REJECTED;
        emit(h, "rejected", std::get<std::string>(decision));
        return events_.back();
    }

    admission_.record(h.intent.task_id);
    emit(h, "accepted");

    if (state) {
        auto lock = gate_.check(*state, h.intent, now);
        if (lock) {
            h.status = TaskStatus::HOLDING;
            emit(h, "holding", *lock);
            return events_.back();
        }
    }

    active_ = std::make_unique<TaskHandle>(h);
    active_->status = TaskStatus::RUNNING;
    emit(*active_, "started");

    long long deadline_ns = active_->intent.deadline_s > 0
                                ? now + static_cast<long long>(active_->intent.deadline_s * 1e9)
                                : std::numeric_limits<long long>::max();

    for (size_t i = 0; i < active_->intent.skills.size(); ++i) {
        active_->skill_index = i;
        std::vector<ActionStep> steps;
        try {
            steps = compile_skill(active_->intent.skills[i]);
        } catch (const SkillError& e) {
            fail(*active_, std::string("skill_compile:") + e.what());
            return events_.back();
        }
        for (const auto& step : steps) {
            if (clock_() > deadline_ns) {
                fail(*active_, "deadline_exceeded");
                forwarder_.abort("deadline");
                return events_.back();
            }
            auto r = forwarder_.execute(step, deadline_ns);
            if (!r.ok) {
                fail(*active_, "step_failed:" + step.kind + ":" + r.detail);
                forwarder_.abort(r.detail);
                return events_.back();
            }
            emit(*active_, "progress", "",
                 {{"skill_index", std::to_string(i)},
                  {"skill", active_->intent.skills[i].name},
                  {"step", step.kind}});
        }
    }

    active_->status = TaskStatus::SUCCEEDED;
    active_->finished_ns = clock_();
    emit(*active_, "succeeded");
    active_.reset();
    return events_.back();
}

std::map<std::string, std::string> TaskRuntime::cancel(const std::string& task_id,
                                                       const std::string& reason) {
    if (!active_ || active_->intent.task_id != task_id) {
        return build_feedback(task_id, "idle", "cancel_noop", {}, "no_active_task", clock_());
    }
    active_->status = TaskStatus::CANCELED;
    active_->finished_ns = clock_();
    forwarder_.abort(reason);
    emit(*active_, "canceled", reason);
    active_.reset();
    return events_.back();
}

}  // namespace legged_nav
