#!/usr/bin/env python3
"""单个 episode 运行器：起栈 → 固定起点/固定地图 → 武装语义搜索 → 记录 → 落盘。

用法:
  python3 runner.py --strategy nearest --goal-id g06 --domain 61 --out /root/exp/runs
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, "/root/exp")
import mapdef  # noqa: E402

MAP_YAML = "/root/exp/config/map_rooms.yaml"
WS_SETUP = ["/opt/ros/jazzy/setup.bash", "/ws/install/setup.bash",
            "/ws/scan_planner_ws/install/setup.bash", "/root/exp/ws/install/setup.bash"]


def prior_region_for(goal_id: str):
    for gid, _x, _y, room in mapdef.GOALS:
        if gid == goal_id:
            cx, cy = mapdef.ROOMS[room]
            return {"x": cx, "y": cy, "sigma": 2.5, "weight": 1.0}
    raise SystemExit(f"unknown goal {goal_id}")


def goal_xy(goal_id: str):
    for gid, x, y, _room in mapdef.GOALS:
        if gid == goal_id:
            return x, y
    raise SystemExit(f"unknown goal {goal_id}")


class Recorder(Node):
    def __init__(self) -> None:
        super().__init__("exp_episode_recorder")
        self.odom = None
        self.events: list[dict] = []
        self.mission_started_at = None
        self.create_subscription(Odometry, "/lio/robo/odom", self._on_odom, 50)
        self.create_subscription(String, "/demo/mission/event", self._on_event, 50)
        self.ev_pub = self.create_publisher(String, "/demo/mission/event", 10)
        self.dir_pub = self.create_publisher(String, "/semantic/directive", 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if body.get("event") == "task_accepted":
            return
        body["_t"] = time.time()
        self.events.append(body)
        if body.get("event") == "mission_started" and self.mission_started_at is None:
            self.mission_started_at = body["_t"]

    def publish_event(self, body: dict) -> None:
        msg = String(data=json.dumps(body))
        for _ in range(6):
            self.ev_pub.publish(msg)
            time.sleep(0.05)

    def publish_prior(self, region: dict) -> None:
        msg = String(data=json.dumps({"graph_bias": {"preferred_regions": [region]}}))
        for _ in range(3):
            self.dir_pub.publish(msg)
            time.sleep(0.05)

    def xy(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return p.x, p.y


def bash_cmd(cmd: str) -> str:
    src = " && ".join(f"source {p}" for p in WS_SETUP if os.path.exists(p))
    return f"{src} && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && {cmd}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--strategy", required=True)
    ap.add_argument("--goal-id", required=True)
    ap.add_argument("--domain", type=int, required=True)
    ap.add_argument("--out", default="/root/exp/runs")
    ap.add_argument("--max-time", type=float, default=110.0)
    ap.add_argument("--max-visits", type=int, default=10)
    ap.add_argument("--warmup", type=float, default=26.0)
    ap.add_argument("--success-radius", type=float, default=1.0)
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    rclpy.init()
    rec = Recorder()

    gx, gy = goal_xy(args.goal_id)
    region = prior_region_for(args.goal_id)
    outdir = os.path.join(args.out, args.strategy, args.goal_id)
    os.makedirs(outdir, exist_ok=True)
    overlay = os.path.join(outdir, "params.yaml")
    with open(overlay, "w") as f:
        f.write(
            "scan_planner_node:\n"
            "  ros__parameters:\n"
            "    grid_map.obstacle_height_max: 3.0\n"
            "    manager.max_vel: 0.5\n"
            "    manager.max_acc: 2.5\n"
            "    manager.max_jerk: 8.0\n"
            "    optimization.max_vel: 0.5\n"
            "    optimization.max_acc: 2.5\n"
            "    optimization.max_jerk: 8.0\n"
            "    optimization.acc_tolerance: 1.5\n"
            "    optimization.vel_tolerance: 0.3\n"
            "closed_loop_controller:\n"
            "  ros__parameters:\n"
            "    tracking.max_vx: 0.5\n"
            "    tracking.max_vy: 0.4\n"
            "    tracking.max_yaw_rate: 1.5\n"
            "demo_scan_bt_orchestrator:\n"
            "  ros__parameters:\n"
            f"    frontier_strategy: {args.strategy}\n"
            f"    search_max_time_s: {args.max_time:.1f}\n"
            f"    search_max_frontier_visits: {args.max_visits}\n"
            "    search_max_distance_m: 40.0\n"
            "    max_subgoal_dist_m: 12.0\n"
        )

    launch_cmd = (
        "ros2 launch motionslam_bringup demo_scan_stack.launch.py "
        "with_sensors:=false with_lio:=false with_mock_robot:=true "
        "with_semantic_objnav:=true with_glass_aware:=false with_foxglove:=false "
        "with_edge_uplink:=false with_thin_harness:=false "
        f"vkernel_params_file:={MAP_YAML} "
        f"extra_params_file:={overlay} "
        "demo_profile:=demo_short autostart_lifecycle:=true"
    )
    pub_cmd = (
        "ros2 run motionslam_bringup subgraph_publisher_node.py --ros-args "
        "-p enabled:=false -p frontier_source:=grid -p publish_hz:=2.0 "
        "-p frontier_max_count:=10 -p frontier_grid_max_dist_m:=12.0 "
        "-p frontier_sensor_range_m:=5.0"
    )
    launch_log = open(os.path.join(outdir, "launch.log"), "w")
    pub_log = open(os.path.join(outdir, "publisher.log"), "w")
    env = dict(os.environ)
    launch = subprocess.Popen(
        ["bash", "-lc", bash_cmd(launch_cmd)],
        stdout=launch_log, stderr=subprocess.STDOUT, env=env, start_new_session=True, cwd=outdir,
    )
    pub = subprocess.Popen(
        ["bash", "-lc", bash_cmd(pub_cmd)],
        stdout=pub_log, stderr=subprocess.STDOUT, env=env, start_new_session=True, cwd=outdir,
    )

    t_start = time.time()
    trace: list[tuple[float, float, float]] = []
    success = False
    t_arrival = None
    t_signal = None
    armed = False
    try:
        while time.time() - t_start < args.warmup + args.max_time + 30.0:
            rclpy.spin_once(rec, timeout_sec=0.05)
            now = time.time()
            xy = rec.xy()
            if xy is not None:
                trace.append((now - t_start, xy[0], xy[1]))
            if xy is not None and not armed and now - t_start > args.warmup:
                rec.publish_prior(region)
                rec.publish_event(
                    {
                        "event": "task_accepted",
                        "command_id": f"exp_{args.strategy}_{args.goal_id}",
                        "language_query": f"find target {args.goal_id}",
                        "stage_ids": ["SEARCH_SEMANTIC_TARGET"],
                    }
                )
                armed = True
                t_signal = now
            if armed and xy is not None:
                if math.hypot(xy[0] - gx, xy[1] - gy) <= args.success_radius:
                    success = True
                    t_arrival = now
                    break
                if rec.mission_started_at is not None and now - rec.mission_started_at > args.max_time:
                    break
                if now - t_signal > args.max_time + 45.0:
                    break
            time.sleep(0.05)
    finally:
        for proc in (launch, pub):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass
        time.sleep(6)
        for proc in (launch, pub):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        launch_log.close()
        pub_log.close()

    decisions = [e for e in rec.events if e.get("event") == "frontier_selected"]
    phases = [e.get("phase") for e in rec.events if e.get("phase")]
    phase_seq = [phases[0]] if phases else []
    for ph in phases[1:]:
        if ph != phase_seq[-1]:
            phase_seq.append(ph)

    arrivals = [t for t, _x, _y in trace]
    travel = 0.0
    cut = len(trace)
    if success and t_arrival is not None:
        cut = max(1, sum(1 for t in arrivals if t + t_start <= t_arrival))
    for a, b in zip(trace[:cut], trace[1:cut]):
        travel += math.hypot(b[1] - a[1], b[2] - a[2])

    result = {
        "strategy": args.strategy,
        "goal_id": args.goal_id,
        "goal": [gx, gy],
        "room": next(r for gid, _x, _y, r in mapdef.GOALS if gid == args.goal_id),
        "prior_region": region,
        "success": bool(success),
        "t_arrival_s": None if t_arrival is None else round(t_arrival - (rec.mission_started_at or t_signal or t_start), 3),
        "traveled_m": round(travel, 3),
        "optimal_m": round(mapdef.shortest_path_len(mapdef.START, (gx, gy)), 3),
        "decisions": decisions,
        "events": rec.events,
        "phase_sequence": phase_seq,
        "trace_hz": round(len(trace) / max(1e-6, (time.time() - t_start)), 2),
        "domain": args.domain,
    }
    with open(os.path.join(outdir, "result.json"), "w") as f:
        json.dump(result, f, ensure_ascii=False)
    print(json.dumps({k: result[k] for k in ("strategy", "goal_id", "success", "traveled_m",
                                             "optimal_m", "t_arrival_s")}, ensure_ascii=False))
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
