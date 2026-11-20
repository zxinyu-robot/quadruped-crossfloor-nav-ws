#pragma once
/// @file feedback.hpp
/// @brief ExecutionFeedback（上行）payload 构造。

#include <map>
#include <string>

#include "legged_nav/executor.hpp"

namespace legged_nav {

/// 构建 schema=motionslam.execution_feedback.v1 的反馈对象（string→string 平铺；
/// progress 用 "k=v" 平铺以保留简单可序列化结构）。
std::map<std::string, std::string> build_feedback(
    const std::string& task_id, const std::string& state, const std::string& event,
    const std::map<std::string, std::string>& progress, const std::string& detail,
    long long timestamp_ns);

}  // namespace legged_nav
