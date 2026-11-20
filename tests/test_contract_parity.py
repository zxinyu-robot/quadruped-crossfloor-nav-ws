"""TaskIntent 契约对照：与 C++ 权威实现读取同一份语料（tests/contract_cases.json）。

两条链路（C++ `validate_task_intent_text` / Python `validate_task_intent`）对同一批
边界输入必须给出完全相同的判定，避免双实现漂移。
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from legged_nav.task_intent import validate_task_intent

CORPUS = json.loads(
    (Path(__file__).parent / "contract_cases.json").read_text(encoding="utf-8")
)["cases"]


@pytest.mark.parametrize("case", CORPUS, ids=[c["name"] for c in CORPUS])
def test_contract_case_matches_expectation(case):
    intent, err = validate_task_intent(json.loads(case["intent"]))
    got = "OK" if err is None else err
    assert got == case["expect"]
    assert (intent is None) == (err is not None)


def test_corpus_covers_rejects_and_accepts():
    """语料自身要同时覆盖放行与拒绝，避免退化成单向断言。"""
    expects = {c["expect"] for c in CORPUS}
    assert "OK" in expects
    assert any(e.startswith("REJECT_") for e in expects)
