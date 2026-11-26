#!/usr/bin/env python3
"""聚合 20 目标 x N 策略的 episode 结果 → 对比表 + 消融。

指标定义：
  成功率      = 到达目标 0.6m 内的 episode 占比
  SPL        = mean_i( success_i * L_i / max(P_i, L_i) )，L=地图真值最短路，P=实际里程
  决策耗时    = frontier_selected 事件里 select_by_strategy 的墙钟耗时（ms），全体 P50/P95
  行为切换    = episode 内 BT phase 变化次数（phase_sequence 长度-1）
  无效重规划率 = 至少出现一次规划失败(pct_path_failed/replan_timeout/plan_fail_hold)
                的 frontier 派发次数 / 总派发次数
"""
from __future__ import annotations

import glob
import json
import os
import statistics
import sys
from collections import defaultdict

FAIL_EVENTS = {"pct_path_failed", "replan_timeout", "plan_fail_hold"}


def pct(vals, q):
    if not vals:
        return float("nan")
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def load(runs_dir):
    out = []
    for path in glob.glob(os.path.join(runs_dir, "*", "*", "result.json")):
        with open(path) as f:
            out.append(json.load(f))
    return out


def invalid_replans(res):
    events = res["events"]
    dispatches = [i for i, e in enumerate(events) if e.get("event") == "frontier_selected"]
    invalid = 0
    for k, idx in enumerate(dispatches):
        end = dispatches[k + 1] if k + 1 < len(dispatches) else len(events)
        window = events[idx + 1:end]
        if any(e.get("event") in FAIL_EVENTS for e in window):
            invalid += 1
    return invalid, len(dispatches)


def summarize(results):
    by = defaultdict(list)
    for r in results:
        by[r["strategy"]].append(r)
    rows = {}
    for strat, rs in by.items():
        succ = [r for r in rs if r["success"]]
        spls = []
        for r in rs:
            L = r["optimal_m"]
            P = max(r["traveled_m"], 1e-6)
            spls.append((L / max(P, L)) if r["success"] else 0.0)
        dms = [float(d.get("decision_ms", 0.0)) for r in rs for d in r["decisions"]]
        switches = [max(0, len(r["phase_sequence"]) - 1) for r in rs]
        inv = sum(invalid_replans(r)[0] for r in rs)
        tot = sum(invalid_replans(r)[1] for r in rs)
        rows[strat] = {
            "n": len(rs),
            "success_rate": len(succ) / len(rs),
            "spl": statistics.fmean(spls) if spls else 0.0,
            "dec_p50_ms": pct(dms, 0.50),
            "dec_p95_ms": pct(dms, 0.95),
            "decisions": len(dms),
            "switches": statistics.fmean(switches) if switches else 0.0,
            "invalid_rate": (inv / tot) if tot else 0.0,
            "dispatches": tot,
            "travel_m": statistics.fmean([r["traveled_m"] for r in rs]),
            "time_s": statistics.fmean([r["t_arrival_s"] or 0.0 for r in rs]),
        }
    return rows


def main():
    runs_dir = sys.argv[1] if len(sys.argv) > 1 else "/root/exp/runs"
    rows = summarize(load(runs_dir))
    main_strats = [s for s in ("nearest", "infogain", "semantic", "weighted") if s in rows]
    abl = [s for s in rows if s.startswith("weighted_no_")]
    print("MAIN")
    print(json.dumps({s: rows[s] for s in main_strats}, indent=1, ensure_ascii=False))
    if abl:
        print("ABLATION")
        print(json.dumps({s: rows[s] for s in ["weighted"] + abl if s in rows},
                         indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
