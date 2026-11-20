"""VLM 视觉语义适配层：RGB 帧 + 任务语言 → 语义观测 JSON。

后端可插拔：
- StubVLMBackend：预设输出的替身，离线 CPU 可测整条链路；
- QwenVLBackend：Qwen2.5-VL-7B 真推理参考模型（需 GPU/Jetson + transformers/torch）。
接口与 stub 完全一致，替换不改链路代码。

证据口径：stub 跑通验证的是链路正确性，不代表模型视觉感知能力；
真推理证据需在 GPU/Jetson 上另行标注。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Protocol


@dataclass
class SemanticObservation:
    """VLM 对单帧视觉证据的语义解析结果（上行给决策层）。"""

    scene_class: str = ""          # 场景类别（如 kitchen / corridor）
    confidence: float = 0.0        # 场景置信度 0~1
    landmarks: list[str] = field(default_factory=list)   # 可见地标
    hazards: list[str] = field(default_factory=list)     # 可见危险/障碍标签
    target_visible: bool = False   # 任务目标是否在视野中
    model_id: str = ""             # 推理后端标识（stub / qwen2.5-vl-7b-awq ...）
    frame_id: str = ""
    timestamp_ns: int = 0
    raw_text: str = ""             # 模型原始输出（可审计）

    def to_dict(self) -> dict[str, Any]:
        return {
            "scene_class": self.scene_class,
            "confidence": self.confidence,
            "landmarks": self.landmarks,
            "hazards": self.hazards,
            "target_visible": self.target_visible,
            "model_id": self.model_id,
            "frame_id": self.frame_id,
            "timestamp_ns": self.timestamp_ns or time.time_ns(),
            "raw_text": self.raw_text,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any], *, model_id: str = "") -> SemanticObservation:
        return cls(
            scene_class=str(d.get("scene_class", "")),
            confidence=float(d.get("confidence", 0.0)),
            landmarks=[str(x) for x in d.get("landmarks", [])],
            hazards=[str(x) for x in d.get("hazards", [])],
            target_visible=bool(d.get("target_visible", False)),
            model_id=model_id or str(d.get("model_id", "")),
            frame_id=str(d.get("frame_id", "")),
            timestamp_ns=int(d.get("timestamp_ns", 0)),
            raw_text=str(d.get("raw_text", d.get("raw", ""))),
        )


class VLMBackend(Protocol):
    """视觉语义后端协议。实现方只返回结构化语义观测，不产出运动指令。"""

    model_id: str

    def observe(self, frame: Any, task_prompt: str, frame_id: str = "") -> SemanticObservation:
        """frame 可为图像字节、文件路径或后端自定义句柄。"""
        ...


@dataclass
class StubVLMBackend:
    """离线替身：按预设（或回调）返回语义观测。

    preset: SemanticObservation | dict | Callable[[frame, task_prompt], obs]
    """

    preset: Any = None
    model_id: str = "stub"

    def observe(self, frame: Any, task_prompt: str, frame_id: str = "") -> SemanticObservation:
        if callable(self.preset):
            obs = self.preset(frame, task_prompt)
        elif isinstance(self.preset, SemanticObservation):
            obs = self.preset
        elif isinstance(self.preset, dict):
            obs = SemanticObservation.from_dict(self.preset)
        else:
            obs = SemanticObservation()
        obs.model_id = self.model_id
        obs.frame_id = frame_id
        obs.timestamp_ns = obs.timestamp_ns or time.time_ns()
        return obs


class QwenVLBackend:
    """Qwen2.5-VL-7B 真推理后端（需 GPU/Jetson；本仓 CI 不加载权重）。

    model_id 如实写实际加载的权重名（如 qwen2.5-vl-7b-awq / qwen2.5-vl-7b），
    避免把"接口存在"写成"7B 已跑通"。
    """

    PROMPT_TEMPLATE = (
        "你是四足机器人端侧语义感知模块。任务：{task}\n"
        "观察当前 RGB 帧，只输出一个 JSON 对象，字段："
        "scene_class(场景类别)、confidence(0~1)、landmarks(可见地标列表)、"
        "hazards(可见障碍/危险标签列表)、target_visible(任务目标是否在视野中)。"
    )

    def __init__(self, model_path: str = "Qwen/Qwen2.5-VL-7B-Instruct",
                 device: str = "auto", model_id: str = "") -> None:
        self.model_path = model_path
        self.device = device
        self.model_id = model_id or model_path.rsplit("/", 1)[-1]
        self._model = None
        self._processor = None

    def _load(self) -> None:
        if self._model is not None:
            return
        try:
            import torch  # noqa: F401
            from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
        except ImportError as e:
            raise RuntimeError(
                "QwenVLBackend 需要 transformers + torch（GPU/Jetson）；"
                "离线链路测试请用 StubVLMBackend"
            ) from e
        self._processor = AutoProcessor.from_pretrained(self.model_path)
        self._model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            self.model_path, torch_dtype="auto", device_map=self.device
        )

    def observe(self, frame: Any, task_prompt: str, frame_id: str = "") -> SemanticObservation:
        self._load()
        import torch
        from PIL import Image

        if isinstance(frame, (str, bytes)):
            import io

            image = (
                Image.open(io.BytesIO(frame)) if isinstance(frame, bytes) else Image.open(frame)
            )
        else:
            image = frame
        image = image.convert("RGB")

        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": self.PROMPT_TEMPLATE.format(task=task_prompt)},
                ],
            }
        ]
        text = self._processor.apply_chat_template(messages, tokenize=False,
                                                 add_generation_prompt=True)
        inputs = self._processor(text=[text], images=[image], return_tensors="pt").to(
            self._model.device
        )
        with torch.no_grad():
            out = self._model.generate(**inputs, max_new_tokens=256)
        raw = self._processor.batch_decode(
            out[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True
        )[0]

        obs = self._parse(raw)
        obs.model_id = self.model_id
        obs.frame_id = frame_id
        obs.timestamp_ns = time.time_ns()
        obs.raw_text = raw
        return obs

    @staticmethod
    def _parse(raw: str) -> SemanticObservation:
        try:
            start = raw.find("{")
            end = raw.rfind("}") + 1
            data = json.loads(raw[start:end])
        except (ValueError, json.JSONDecodeError):
            return SemanticObservation(raw_text=raw)
        return SemanticObservation.from_dict(data)

