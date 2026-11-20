"""运行时契约常量（自动生成，请勿手改）。

来源：contracts/runtime_contract.json · 生成器：scripts/gen_contract.py
与 C++ 头文件 include/legged_nav/contract_gen.hpp 同源。
"""
from __future__ import annotations

VERSION = 1
INTENT_SCHEMA = "motionslam.task_intent.v1"
FEEDBACK_SCHEMA = "motionslam.execution_feedback.v1"
TRIM_STRINGS = True
COERCE_NON_STRING = False

ALLOWED_PLATFORMS = frozenset({
    "unitree_go2_edu",
    "unitree_go2",
})

ALLOWED_SKILLS = frozenset({
    "Navigate",
    "ReturnHome",
    "ExploreFrontier",
    "Stand",
    "Stop",
})

FORBIDDEN_CONTROL_KEYS = frozenset({
    "cmd_vel",
    "/cmd_vel",
    "twist",
    "Twist",
    "initial_path",
    "/initial_path",
    "/motion/command",
})

SKILL_KINDS = {
    "Navigate": "navigate_to",
    "ReturnHome": "return_home",
    "ExploreFrontier": "explore",
    "Stand": "stand",
    "Stop": "stop",
}

SKILL_TIMEOUTS_S = {
    "Navigate": 120.0,
    "ReturnHome": 180.0,
    "ExploreFrontier": 300.0,
    "Stand": 5.0,
    "Stop": 2.0,
}

SKILL_PARAMS = {
    "ExploreFrontier": {"strategy": "frontier"},
}
