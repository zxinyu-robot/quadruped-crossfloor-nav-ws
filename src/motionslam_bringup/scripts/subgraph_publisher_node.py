#!/usr/bin/env python3
"""SubgraphPublisher：1Hz + BT 事件触发，上行 sparse_subgraph.v1 JSON."""
from __future__ import annotations

import json
import math
from collections import deque
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String, UInt64

_script_dir = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
if _script_dir not in __import__("sys").path:
    __import__("sys").path.insert(0, _script_dir)

from data_layer_types import DataLayerId  # noqa: E402
from edge_uplink_client import EdgeTcpUplinkClient  # noqa: E402
from frontier_export import FrontierExportConfig, export_frontiers_from_keyframes  # noqa: E402
from subgraph_snapshot import (  # noqa: E402
    FrontierView,
    KeyframeView,
    SubgraphBuildInput,
    build_sparse_subgraph_v1,
    quat_yaw,
)


BT_TRIGGER_EVENTS = frozenset(
    {
        "replan_timeout",
        "subgoal_retry",
        "plan_fail_hold",
        "plan_fail_recovery_done",
        "pgo_applied_redispatch",
        "subgoal_dispatched",
        "search_frontier_reached",
        "search_exhausted",
    }
)


class SubgraphPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("subgraph_publisher")
        self.declare_parameter("enabled", True)
        self.declare_parameter("edge_host", "127.0.0.1")
        self.declare_parameter("subgraph_port", 9877)
        self.declare_parameter("session_id", "demo_live")
        self.declare_parameter("floor_id", "floor_01")
        self.declare_parameter("tile_id", "main")
        self.declare_parameter("map_id", "demo_live_map")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("queue_max", 8)
        self.declare_parameter("connect_timeout_s", 2.0)
        self.declare_parameter("send_timeout_s", 3.0)
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("keyframe_poses_topic", "/lio/backend/keyframe_poses")
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("pgo_state_topic", "/lio/backend/pgo_state")
        self.declare_parameter("mission_event_topic", "/demo/mission/event")
        self.declare_parameter("use_bt_subgoal_as_frontier", False)
        self.declare_parameter("frontier_min_dist_m", 1.0)
        self.declare_parameter("frontier_max_dist_m", 12.0)
        self.declare_parameter("frontier_max_count", 8)
        self.declare_parameter("frontier_ray_count", 8)
        self.declare_parameter("frontier_ray_dist_m", 3.0)
        # 实验用：由点云在线建 2D 占据图并提取 frontier（stub=旧射线近似）
        self.declare_parameter("frontier_source", "stub")  # stub | grid
        self.declare_parameter("frontier_cloud_topic", "/lio/cloud_world")
        self.declare_parameter("frontier_grid_res_m", 0.10)
        self.declare_parameter("frontier_grid_min_xy", -1.0)
        self.declare_parameter("frontier_grid_size_m", 12.0)
        self.declare_parameter("frontier_grid_z_lo", 0.25)
        self.declare_parameter("frontier_grid_z_hi", 1.35)
        self.declare_parameter("frontier_sensor_range_m", 12.0)
        self.declare_parameter("frontier_grid_info_radius_m", 2.0)
        self.declare_parameter("frontier_grid_clearance_m", 0.45)
        self.declare_parameter("frontier_grid_min_dist_m", 0.8)
        self.declare_parameter("frontier_grid_max_dist_m", 12.0)
        self.declare_parameter("frontier_grid_min_cluster_cells", 2)
        self.declare_parameter("frontier_grid_min_separation_m", 1.5)
        self.declare_parameter("frontier_snapshot_topic", "/demo/mission/frontier_snapshot")
        self.declare_parameter("log_each_uplink", True)
        self.declare_parameter("layer_event_topic", "/edge/data_layer/layer_event")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._session_id = str(self.get_parameter("session_id").value)
        self._floor_id = str(self.get_parameter("floor_id").value)
        self._tile_id = str(self.get_parameter("tile_id").value)
        self._map_id = str(self.get_parameter("map_id").value)
        self._use_bt_frontier = bool(self.get_parameter("use_bt_subgoal_as_frontier").value)
        self._log_each = bool(self.get_parameter("log_each_uplink").value)
        self._frontier_cfg = FrontierExportConfig(
            min_dist_m=float(self.get_parameter("frontier_min_dist_m").value),
            max_dist_m=float(self.get_parameter("frontier_max_dist_m").value),
            max_frontiers=int(self.get_parameter("frontier_max_count").value),
            ray_count=int(self.get_parameter("frontier_ray_count").value),
            ray_dist_m=float(self.get_parameter("frontier_ray_dist_m").value),
        )
        self._last_frontiers: list[FrontierView] = []
        self._frontier_source = str(self.get_parameter("frontier_source").value).lower()
        self._g_res = float(self.get_parameter("frontier_grid_res_m").value)
        self._g_xy0 = float(self.get_parameter("frontier_grid_min_xy").value)
        self._g_size = float(self.get_parameter("frontier_grid_size_m").value)
        self._g_n = max(8, int(round(self._g_size / self._g_res)))
        self._g_zlo = float(self.get_parameter("frontier_grid_z_lo").value)
        self._g_zhi = float(self.get_parameter("frontier_grid_z_hi").value)
        self._g_range = float(self.get_parameter("frontier_sensor_range_m").value)
        self._g_info_r = float(self.get_parameter("frontier_grid_info_radius_m").value)
        self._g_clear = float(self.get_parameter("frontier_grid_clearance_m").value)
        self._g_min_dist = float(self.get_parameter("frontier_grid_min_dist_m").value)
        self._g_max_dist = float(self.get_parameter("frontier_grid_max_dist_m").value)
        self._g_min_cells = int(self.get_parameter("frontier_grid_min_cluster_cells").value)
        self._g_min_sep = float(self.get_parameter("frontier_grid_min_separation_m").value)
        self._g_free = np.zeros((self._g_n, self._g_n), dtype=bool)
        self._g_hit = np.zeros((self._g_n, self._g_n), dtype=bool)
        self._g_updates = 0

        self._version = 0
        self._pending_trigger = "periodic"
        self._odom: Optional[Odometry] = None
        self._keyframes: list[KeyframeView] = []
        self._context_generation = 0
        self._pgo_state = "IDLE"
        self._last_mission_subgoal: Optional[tuple[float, float, float]] = None

        odom_topic = str(self.get_parameter("odom_topic").value)
        kf_topic = str(self.get_parameter("keyframe_poses_topic").value)
        gen_topic = str(self.get_parameter("context_generation_topic").value)
        pgo_topic = str(self.get_parameter("pgo_state_topic").value)
        event_topic = str(self.get_parameter("mission_event_topic").value)

        self._client: Optional[EdgeTcpUplinkClient] = None
        if self._enabled:
            host = str(self.get_parameter("edge_host").value)
            port = int(self.get_parameter("subgraph_port").value)
            self._client = EdgeTcpUplinkClient(
                host=host,
                port=port,
                queue_max=int(self.get_parameter("queue_max").value),
                connect_timeout_s=float(self.get_parameter("connect_timeout_s").value),
                send_timeout_s=float(self.get_parameter("send_timeout_s").value),
                on_error=lambda msg: self.get_logger().warn(msg),
            )

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(PoseArray, kf_topic, self._on_keyframes, 10)
        self.create_subscription(UInt64, gen_topic, self._on_context_generation, 10)
        self.create_subscription(String, pgo_topic, self._on_pgo_state, 10)
        self.create_subscription(String, event_topic, self._on_mission_event, 10)
        if self._frontier_source == "grid":
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("frontier_cloud_topic").value),
                self._on_cloud,
                qos_profile_sensor_data,
            )

        hz = max(0.2, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._on_timer)

        self._event_pub = self.create_publisher(String, "/edge/uplink/subgraph_event", 10)
        self._frontier_pub = self.create_publisher(
            String, str(self.get_parameter("frontier_snapshot_topic").value), 10
        )
        self._layer_event_pub = self.create_publisher(
            String, str(self.get_parameter("layer_event_topic").value), 10
        )

        self.get_logger().info(
            f"SubgraphPublisher enabled={self._enabled} "
            f"→ {self.get_parameter('edge_host').value}:"
            f"{self.get_parameter('subgraph_port').value} @ {hz:.1f}Hz"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_keyframes(self, msg: PoseArray) -> None:
        out: list[KeyframeView] = []
        for i, pose in enumerate(msg.poses):
            p = pose.position
            q = pose.orientation
            yaw = quat_yaw(q.x, q.y, q.z, q.w)
            out.append(KeyframeView(i, p.x, p.y, p.z, yaw))
        self._keyframes = out

    def _on_context_generation(self, msg: UInt64) -> None:
        self._context_generation = int(msg.data)

    def _on_pgo_state(self, msg: String) -> None:
        self._pgo_state = msg.data.strip().upper() or "IDLE"

    def _on_mission_event(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = str(body.get("event", ""))
        if event in BT_TRIGGER_EVENTS:
            self._pending_trigger = "bt_replan"
        if event == "subgoal_dispatched" and self._use_bt_frontier:
            x = body.get("x")
            y = body.get("y")
            z = body.get("z")
            if x is not None and y is not None:
                self._last_mission_subgoal = (float(x), float(y), float(z or 0.35))

    def _build_frontiers(
        self,
        robot_x: float,
        robot_y: float,
        robot_z: float,
        robot_yaw: float,
    ) -> list[FrontierView]:
        if self._use_bt_frontier and self._last_mission_subgoal is not None:
            x, y, z = self._last_mission_subgoal
            return [
                FrontierView(
                    frontier_id="bt_next_subgoal",
                    x=x,
                    y=y,
                    z=z if z > 0 else robot_z,
                    geom_score=1.0,
                )
            ]
        if self._frontier_source == "grid":
            return self._grid_frontiers(robot_x, robot_y, robot_z)
        return export_frontiers_from_keyframes(
            robot_x,
            robot_y,
            robot_z,
            robot_yaw,
            self._keyframes,
            config=self._frontier_cfg,
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        """点云 → 2D 占据图（每 1° 方位取最近命中，未命中按量程），射线扫出 free。"""
        if self._odom is None:
            return
        try:
            raw = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            if raw is None or len(raw) == 0:
                return
            if raw.dtype.names:
                pts = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32)
            else:
                pts = np.asarray(raw, dtype=np.float32)
        except Exception as exc:  # pragma: no cover - 诊断用
            self._cloud_err = getattr(self, "_cloud_err", 0) + 1
            if self._cloud_err < 3:
                self.get_logger().error(f"cloud parse failed: {exc}")
            return
        p = self._odom.pose.pose.position
        rx, ry = float(p.x), float(p.y)

        sel = (pts[:, 2] > self._g_zlo) & (pts[:, 2] < self._g_zhi)
        xy = pts[sel][:, :2]
        if len(xy) == 0:
            return
        dx = xy[:, 0] - rx
        dy = xy[:, 1] - ry
        rng = np.hypot(dx, dy)
        keep = (rng > 0.15) & (rng <= self._g_range)
        if not keep.any():
            return
        dx, dy, rng = dx[keep], dy[keep], rng[keep]

        nb = 360
        bins = (((np.arctan2(dy, dx) + np.pi) / (2.0 * np.pi)) * nb).astype(int) % nb
        min_r = np.full(nb, self._g_range)  # 未命中方位按量程
        np.minimum.at(min_r, bins, rng)

        ang = -np.pi + (np.arange(nb) + 0.5) * (2.0 * np.pi / nb)
        ex = rx + min_r * np.cos(ang)
        ey = ry + min_r * np.sin(ang)

        ci = np.floor((ex - self._g_xy0) / self._g_res).astype(int)
        cj = np.floor((ey - self._g_xy0) / self._g_res).astype(int)
        ok = (ci >= 0) & (ci < self._g_n) & (cj >= 0) & (cj < self._g_n)
        ci, cj, min_r = ci[ok], cj[ok], min_r[ok]
        if len(ci) == 0:
            return

        i0 = int(np.floor((rx - self._g_xy0) / self._g_res))
        j0 = int(np.floor((ry - self._g_xy0) / self._g_res))
        i0 = max(0, min(self._g_n - 1, i0))
        j0 = max(0, min(self._g_n - 1, j0))

        steps = np.maximum(np.abs(ci - i0), np.abs(cj - j0))
        max_steps = int(steps.max())
        if max_steps == 0:
            self._g_updates += 1
            return
        t = np.linspace(0.0, 1.0, max_steps + 1)[None, :]
        si = np.round(i0 + (ci - i0)[:, None] * t).astype(np.int32)
        sj = np.round(j0 + (cj - j0)[:, None] * t).astype(np.int32)
        valid = np.arange(max_steps + 1)[None, :] < steps[:, None]
        fi = si[valid]
        fj = sj[valid]
        np.clip(fi, 0, self._g_n - 1, out=fi)
        np.clip(fj, 0, self._g_n - 1, out=fj)
        self._g_free[fj, fi] = True

        hit = min_r < (self._g_range - 1e-3)  # 只有真正观测到墙的方位才记 occupied
        if hit.any():
            hi, hj = ci[hit], cj[hit]
            self._g_hit[hj, hi] = True
            self._g_free[hj, hi] = False
        self._g_updates += 1

    def _grid_frontiers(self, rx: float, ry: float, rz: float) -> list[FrontierView]:
        """frontier 候选：free 与 unknown 相邻的像元，按信息增益排序 + 最小间距抑制。"""
        if self._g_updates == 0:
            return []
        free = self._g_free
        unknown = ~self._g_free & ~self._g_hit
        near_unknown = np.zeros_like(unknown)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                near_unknown |= np.roll(np.roll(unknown, dy, 0), dx, 1)
        frontier = free & near_unknown
        ys, xs = np.nonzero(frontier)
        if len(ys) == 0:
            return []

        n = self._g_n

        def integral(mask: np.ndarray) -> np.ndarray:
            c = mask.astype(np.int32).cumsum(0).cumsum(1)
            return np.pad(c, ((1, 0), (1, 0)))

        ii_unknown = integral(unknown)
        ii_hit = integral(self._g_hit)

        def box_sum(ii: np.ndarray, y0, y1, x0, x1):
            return ii[y1, x1] - ii[y0, x1] - ii[y1, x0] + ii[y0, x0]

        clear_cells = max(1, int(round(self._g_clear / self._g_res)))
        info_cells = max(1, int(round(self._g_info_r / self._g_res)))

        cy0 = np.clip(ys - clear_cells, 0, n)
        cy1 = np.clip(ys + clear_cells + 1, 0, n)
        cx0 = np.clip(xs - clear_cells, 0, n)
        cx1 = np.clip(xs + clear_cells + 1, 0, n)
        clear_ok = box_sum(ii_hit, cy0, cy1, cx0, cx1) == 0

        iy0 = np.clip(ys - info_cells, 0, n)
        iy1 = np.clip(ys + info_cells + 1, 0, n)
        ix0 = np.clip(xs - info_cells, 0, n)
        ix1 = np.clip(xs + info_cells + 1, 0, n)
        unk = box_sum(ii_unknown, iy0, iy1, ix0, ix1).astype(np.float64)
        area = np.maximum(1, (iy1 - iy0) * (ix1 - ix0)).astype(np.float64)
        info = np.clip(unk / area, 0.0, 1.0)

        wx = self._g_xy0 + (xs + 0.5) * self._g_res
        wy = self._g_xy0 + (ys + 0.5) * self._g_res
        dist = np.hypot(wx - rx, wy - ry)
        ok = clear_ok & (dist >= self._g_min_dist) & (dist <= self._g_max_dist)
        if not ok.any():
            return []

        idx = np.nonzero(ok)[0]
        order = idx[np.lexsort((dist[idx], -info[idx]))]
        z = max(float(rz), 0.35)
        out: list[FrontierView] = []
        for k in order.tolist():
            px, py, pr = float(wx[k]), float(wy[k]), float(info[k])
            if pr <= 0.0:
                continue
            if any(math.hypot(px - f.x, py - f.y) < self._g_min_sep for f in out):
                continue
            out.append(
                FrontierView(
                    frontier_id="fx_%d_%d" % (int(round(px * 100)), int(round(py * 100))),
                    x=round(px, 3),
                    y=round(py, 3),
                    z=z,
                    geom_score=1.0 / (1.0 + float(dist[k])),
                    info_gain=pr,
                )
            )
            if len(out) >= self._frontier_cfg.max_frontiers:
                break

        self._dbg = getattr(self, "_dbg", 0) + 1
        if self._dbg % 20 == 1:
            self.get_logger().info(
                "grid free=%d hit=%d frontier=%d cands=%d picked=%d"
                % (int(free.sum()), int(self._g_hit.sum()), int(frontier.sum()),
                   int(ok.sum()), len(out))
            )
        return out

    def _publish_frontier_snapshot(self, frontiers: list[FrontierView], version: int) -> None:
        body = {
            "version": version,
            "frontiers": [
                {
                    "frontier_id": f.frontier_id,
                    "x": f.x,
                    "y": f.y,
                    "z": f.z,
                    "geom_score": f.geom_score,
                    "info_gain": f.info_gain,
                }
                for f in frontiers
            ],
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._frontier_pub.publish(msg)

    def _snapshot(self, trigger_reason: str) -> Optional[dict[str, Any]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = quat_yaw(q.x, q.y, q.z, q.w)
        frontiers = self._build_frontiers(p.x, p.y, max(p.z, 0.3), yaw)
        self._last_frontiers = frontiers
        self._version += 1
        payload = build_sparse_subgraph_v1(
            SubgraphBuildInput(
                version=self._version,
                session_id=self._session_id,
                floor_id=self._floor_id,
                tile_id=self._tile_id,
                map_id=self._map_id,
                trigger_reason=trigger_reason,
                robot_x=p.x,
                robot_y=p.y,
                robot_z=p.z,
                robot_yaw=yaw,
                keyframes=self._keyframes,
                frontiers=frontiers,
                context_generation=self._context_generation,
                pgo_state=self._pgo_state,
            )
        )
        self._publish_frontier_snapshot(frontiers, self._version)
        return payload

    def _publish_event(self, event: str, **extra: Any) -> None:
        body = {"event": event, "version": self._version, **extra}
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._event_pub.publish(msg)

    def _publish_layer_event(self, event: str, **extra: Any) -> None:
        body = {
            "layer_id": DataLayerId.TOPOLOGY_SUBGRAPH.value,
            "event": event,
            "layer_version": self._version,
            "context_generation": self._context_generation,
            **extra,
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._layer_event_pub.publish(msg)

    def _on_timer(self) -> None:
        if not self._enabled or self._client is None:
            # 实验模式：只本地发布 frontier 快照，不做边端上行
            if self._frontier_source == "grid":
                self._pending_trigger = "periodic"
                self._snapshot("periodic")
            return
        trigger = self._pending_trigger
        self._pending_trigger = "periodic"
        snap = self._snapshot(trigger)
        if snap is None:
            return
        ok = self._client.enqueue(snap)
        if self._log_each:
            self.get_logger().info(
                f"SUBGRAPH_UPLINK enqueue v={snap['version']} "
                f"nodes={len(snap['nodes'])} frontiers={len(snap['frontiers'])} "
                f"trigger={trigger} ok={ok}"
            )
        self._publish_event(
            "SUBGRAPH_UPLINK_OK" if ok else "SUBGRAPH_UPLINK_FAIL",
            trigger=trigger,
            nodes=len(snap["nodes"]),
            frontiers=len(snap["frontiers"]),
        )
        self._publish_layer_event(
            "published" if ok else "publish_failed",
            error="" if ok else "uplink_queue_full",
        )

    def destroy_node(self) -> None:
        if self._client is not None:
            self._client.stop()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SubgraphPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
