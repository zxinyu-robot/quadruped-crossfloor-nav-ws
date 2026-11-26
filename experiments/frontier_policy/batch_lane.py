#!/usr/bin/env python3
"""单 lane 串行跑一批 episode；lane 独占一个 ROS_DOMAIN_ID。"""
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, "/root/exp")
import mapdef  # noqa: E402

MAIN = ["nearest", "infogain", "semantic", "weighted"]
ABL = ["weighted_no_dist", "weighted_no_info", "weighted_no_sem"]


def jobs(mode: str):
    strats = MAIN if mode == "main" else (MAIN + ABL if mode == "all" else ABL)
    out = []
    for s in strats:
        for gid, _x, _y, _r in mapdef.GOALS:
            out.append((s, gid))
    return out


def main() -> int:
    lane = int(sys.argv[1])
    lanes = int(sys.argv[2])
    mode = sys.argv[3] if len(sys.argv) > 3 else "all"
    max_time = sys.argv[4] if len(sys.argv) > 4 else "90"
    domain = 70 + lane
    todo = jobs(mode)[lane::lanes]
    for strat, gid in todo:
        out = f"/root/exp/runs/{strat}/{gid}/result.json"
        if os.path.exists(out):
            print(f"[lane{lane}] skip {strat}/{gid}", flush=True)
            continue
        t0 = time.time()
        cp = subprocess.run(
            ["python3", "/root/exp/runner.py", "--strategy", strat, "--goal-id", gid,
             "--domain", str(domain), "--max-time", max_time, "--max-visits", "8"],
            capture_output=True, text=True, timeout=300,
        )
        tag = "OK" if cp.returncode == 0 else "FAIL"
        tail = (cp.stdout or cp.stderr or "").strip().splitlines()[-1:] or [""]
        print(f"[lane{lane}] {tag} {strat}/{gid} {time.time()-t0:.0f}s {tail[0][:160]}", flush=True)
        subprocess.run("sleep 2", shell=True)
    print(f"[lane{lane}] done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
