#!/usr/bin/env python3
"""FrontierScorer：候选前沿打分 + 四种候选选择策略（实验扩展）。

策略（由 BT 参数 frontier_strategy 选择）:
  nearest   最近前沿：欧氏距离最小
  infogain  信息增益：未知栅格占比最大
  semantic  语义先验：graph_bias 区域先验最大
  weighted  规则加权：0.35*近距 + 0.35*信息增益 + 0.30*语义先验
消融（weighted 去掉单一项后重归一）:
  weighted_no_dist / weighted_no_info / weighted_no_sem
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from subgraph_snapshot import FrontierView

DEFAULT_WEIGHTS = {"geom": 0.5, "semantic": 0.5}
POLICY_WEIGHTS = {"dist": 0.35, "info": 0.35, "sem": 0.30}
ABLATIONS = {"weighted_no_dist": "dist", "weighted_no_info": "info", "weighted_no_sem": "sem"}
STRATEGIES = ("nearest", "infogain", "semantic", "weighted", *ABLATIONS)


@dataclass
class FrontierScore:
    frontier_id: str
    x: float
    y: float
    z: float
    geom_score: float
    semantic_score: float
    total_score: float
    distance_m: float = 0.0
    info_gain: float = 0.0
    semantic_prior: float = 0.0


def _frontier_node_id(frontier_id: str) -> str:
    if frontier_id.startswith("kf_"):
        return f"node_{frontier_id}"
    return frontier_id


def graph_bias(frontier: FrontierView, directive: Optional[dict[str, Any]]) -> float:
    if not directive:
        return 0.0
    score = 0.0
    fid = str(frontier.frontier_id)
    candidates = {str(x) for x in directive.get("candidate_frontier_ids") or []}
    if fid in candidates:
        score += 1.0

    bias = directive.get("graph_bias") or {}
    preferred = {str(x) for x in bias.get("preferred_node_ids") or []}
    avoid = {str(x) for x in bias.get("avoid_node_ids") or []}
    node_id = _frontier_node_id(fid)
    if node_id in preferred or fid in preferred:
        score += 0.5
    if node_id in avoid or fid in avoid:
        score -= 1.0
    return score


def score_frontier(
    frontier: FrontierView,
    directive: Optional[dict[str, Any]],
    *,
    weights: Optional[dict[str, float]] = None,
) -> FrontierScore:
    w = dict(DEFAULT_WEIGHTS)
    if directive:
        bias = directive.get("graph_bias") or {}
        raw_w = bias.get("weights") or {}
        if isinstance(raw_w, dict):
            for key in ("geom", "semantic"):
                if key in raw_w:
                    try:
                        w[key] = float(raw_w[key])
                    except (TypeError, ValueError):
                        pass
    if weights:
        w.update(weights)

    geom = max(0.0, min(1.0, float(frontier.geom_score)))
    sem = graph_bias(frontier, directive)
    sem_norm = max(0.0, min(1.0, (sem + 1.0) / 2.0))
    total = w["geom"] * geom + w["semantic"] * sem_norm
    return FrontierScore(
        frontier_id=frontier.frontier_id,
        x=frontier.x,
        y=frontier.y,
        z=frontier.z,
        geom_score=geom,
        semantic_score=sem,
        total_score=total,
        info_gain=max(0.0, min(1.0, float(getattr(frontier, "info_gain", 0.0) or 0.0))),
    )


def rank_frontiers(
    frontiers: list[FrontierView],
    directive: Optional[dict[str, Any]],
    *,
    exclude_ids: Optional[set[str]] = None,
) -> list[FrontierScore]:
    blocked = exclude_ids or set()
    scored = [
        score_frontier(f, directive)
        for f in frontiers
        if f.frontier_id not in blocked
    ]
    scored.sort(key=lambda s: s.total_score, reverse=True)
    return scored


def select_best_frontier(
    frontiers: list[FrontierView],
    directive: Optional[dict[str, Any]],
    *,
    exclude_ids: Optional[set[str]] = None,
) -> Optional[FrontierScore]:
    ranked = rank_frontiers(frontiers, directive, exclude_ids=exclude_ids)
    return ranked[0] if ranked else None


def _prior_regions(directive: Optional[dict[str, Any]]) -> list[tuple[float, float, float, float]]:
    if not directive:
        return []
    bias = directive.get("graph_bias") or {}
    out: list[tuple[float, float, float, float]] = []
    for item in bias.get("preferred_regions") or []:
        if not isinstance(item, dict):
            continue
        try:
            out.append(
                (
                    float(item["x"]),
                    float(item["y"]),
                    float(item.get("sigma", 2.0)),
                    float(item.get("weight", 1.0)),
                )
            )
        except (KeyError, TypeError, ValueError):
            continue
    return out


def semantic_prior(frontier: FrontierView, directive: Optional[dict[str, Any]]) -> float:
    """语义先验 ∈ [0,1]：区域高斯先验 / 显式偏好节点 / 候选命中。"""
    if not directive:
        return 0.0
    fid = str(frontier.frontier_id)
    bias = directive.get("graph_bias") or {}
    pri = 0.0
    if fid in {str(x) for x in directive.get("candidate_frontier_ids") or []}:
        pri = 0.8
    preferred = {str(x) for x in bias.get("preferred_node_ids") or []}
    if fid in preferred or _frontier_node_id(fid) in preferred:
        pri = max(pri, 1.0)
    for px, py, sigma, weight in _prior_regions(directive):
        d = math.hypot(frontier.x - px, frontier.y - py)
        g = max(0.0, min(1.0, weight)) * math.exp(-(d * d) / (2.0 * max(1e-3, sigma) ** 2))
        pri = max(pri, g)
    return max(0.0, min(1.0, pri))


def _weights_for(strategy: str) -> dict[str, float]:
    w = dict(POLICY_WEIGHTS)
    drop = ABLATIONS.get(strategy)
    if drop:
        w[drop] = 0.0
    total = sum(w.values())
    if total <= 0.0:
        return {"dist": 1.0, "info": 0.0, "sem": 0.0}
    return {k: v / total for k, v in w.items()}


def select_by_strategy(
    frontiers: list[FrontierView],
    directive: Optional[dict[str, Any]],
    *,
    strategy: str = "weighted",
    robot_xy: Optional[tuple[float, float]] = None,
    exclude_ids: Optional[set[str]] = None,
    weights: Optional[dict[str, float]] = None,
) -> Optional[FrontierScore]:
    """按策略排序候选并返回最优；语义/信息/距离三项都是可观测量的函数。"""
    blocked = exclude_ids or set()
    cands = [f for f in frontiers if str(f.frontier_id) not in blocked]
    if not cands:
        return None
    strategy = strategy if strategy in STRATEGIES else "weighted"
    rx, ry = robot_xy if robot_xy is not None else (0.0, 0.0)

    scored: list[FrontierScore] = []
    for f in cands:
        dist = math.hypot(f.x - rx, f.y - ry)
        info = max(0.0, min(1.0, float(getattr(f, "info_gain", 0.0) or 0.0)))
        sem = semantic_prior(f, directive)
        prox = 1.0 / (1.0 + dist)
        scored.append(
            FrontierScore(
                frontier_id=str(f.frontier_id),
                x=f.x,
                y=f.y,
                z=f.z,
                geom_score=prox,
                semantic_score=sem,
                total_score=0.0,
                distance_m=dist,
                info_gain=info,
                semantic_prior=sem,
            )
        )

    if strategy == "nearest":
        for s in scored:
            s.total_score = s.geom_score
        scored.sort(key=lambda s: (s.distance_m, -s.info_gain))
    elif strategy == "infogain":
        for s in scored:
            s.total_score = s.info_gain
        scored.sort(key=lambda s: (-s.info_gain, s.distance_m))
    elif strategy == "semantic":
        for s in scored:
            s.total_score = s.semantic_prior
        scored.sort(key=lambda s: (-s.semantic_prior, s.distance_m))
    else:
        w = _weights_for(strategy)
        if weights:
            w = dict(weights)
        for s in scored:
            s.total_score = (
                w.get("dist", 0.0) * s.geom_score
                + w.get("info", 0.0) * s.info_gain
                + w.get("sem", 0.0) * s.semantic_prior
            )
        scored.sort(key=lambda s: (-s.total_score, s.distance_m))
    return scored[0]
