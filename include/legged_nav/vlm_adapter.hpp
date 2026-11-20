#pragma once
/// @file vlm_adapter.hpp
/// @brief VLM 视觉语义适配层：RGB 帧 + 任务语言 → 语义观测。
/// StubVLMBackend 离线替身（单元测试级）；QwenVLBackend 需 GPU/Jetson 推理运行时，
/// 本仓仅提供接口签名，部署侧链接推理运行时后启用。

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "legged_nav/executor.hpp"

namespace legged_nav {

struct SemanticObservation {
    std::string scene_class;
    double confidence = 0.0;
    std::vector<std::string> landmarks;
    std::vector<std::string> hazards;
    bool target_visible = false;
    std::string model_id;
    std::string frame_id;
    long long timestamp_ns = 0;
    std::string raw_text;
};

/// 视觉语义后端协议：只返回结构化语义观测，不产出运动指令。
class VLMBackend {
   public:
    virtual ~VLMBackend() = default;
    virtual SemanticObservation observe(const std::string& frame, const std::string& task_prompt,
                                        const std::string& frame_id = "") = 0;
    virtual std::string model_id() const = 0;
};

/// 离线替身：预设或回调输出。
class StubVLMBackend : public VLMBackend {
   public:
    explicit StubVLMBackend(
        std::function<SemanticObservation(const std::string&, const std::string&)> fn)
        : fn_(std::move(fn)) {}
    explicit StubVLMBackend(SemanticObservation preset) : preset_(std::move(preset)) {}

    SemanticObservation observe(const std::string& frame, const std::string& task_prompt,
                                const std::string& frame_id) override {
        SemanticObservation obs = fn_ ? fn_(frame, task_prompt) : preset_;
        obs.model_id = "stub";
        obs.frame_id = frame_id;
        if (!obs.timestamp_ns) obs.timestamp_ns = now_ns();
        return obs;
    }
    std::string model_id() const override { return "stub"; }

   private:
    std::function<SemanticObservation(const std::string&, const std::string&)> fn_;
    SemanticObservation preset_;
};

/// Qwen2.5-VL-7B 真推理后端接口（参考模型，部署侧实现；量化部署 INT4-AWQ/TensorRT）。
/// 构造即抛异常，标注本仓不含权重与算力依赖。
class QwenVLBackend : public VLMBackend {
   public:
    explicit QwenVLBackend(std::string model_path) : model_path_(std::move(model_path)) {
        throw std::runtime_error(
            "QwenVLBackend 需 GPU/Jetson 推理运行时（部署侧）；离线链路请用 StubVLMBackend");
    }
    SemanticObservation observe(const std::string&, const std::string&,
                                const std::string&) override {
        return {};
    }
    std::string model_id() const override { return model_path_; }

   private:
    std::string model_path_;
};

}  // namespace legged_nav
