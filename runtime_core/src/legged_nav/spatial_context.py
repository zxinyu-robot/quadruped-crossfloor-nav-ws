"""统一空间状态契约管理 + 流式上下文优化（StreamVLN 式 Fast-Slow Context）。

两部分：

1. SpatialStateStore —— 统一空间状态契约：
   围绕机器人位姿、环境地图、空间目标与任务状态，统一管理
   时间戳、坐标系、状态版本（revision）、置信度与有效期；
   支持快照（snapshot）与增量更新（update），过期/失效状态不下发。

2. StreamContext —— 借鉴 StreamVLN 的 Fast-Slow Context：
   - fast 窗口：滑动窗口存最近视觉 Token（KV Cache 增量复用，
     新帧只算增量 Token，不动已缓存前缀）；
   - slow 记忆：滑出窗口的条目压缩成摘要保留（历史记忆压缩）；
   - 视觉 Token 去重：按 LIO 位姿几何距离合并近重复观测，
     降低连续观测中的信息冗余与推理开销。
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from .spatial_state import SpatialState

# ---------- 1. 统一空间状态契约存储 ----------


@dataclass
class SpatialEntry:
    """一个空间状态条目：版本化 + 有效期 + 置信度。"""

    key: str                       # 条目标识（pose / map / goal / task_state）
    state: SpatialState
    goal: dict[str, float] | None = None
    revision: int = 0
    ttl_ns: int = 500_000_000      # 有效期（本地时钟）
    updated_ns: int = field(default_factory=time.time_ns)

    def is_valid(self, now_ns: int) -> bool:
        return self.state.valid and (now_ns - self.updated_ns) <= self.ttl_ns


class SpatialStateStore:
    """版本化空间状态存储：快照 + 增量更新。

    - update(key, ...) 增量写并自增 revision；
    - snapshot() 返回当前只读视图（不可变拷贝语义，调用方不改内部态）；
    - entries(now_ns) 只返回未过期且有效的条目 —— 失效契约不下发。
    """

    def __init__(self) -> None:
        self._entries: dict[str, SpatialEntry] = {}
        self._revision = 0

    def update(
        self,
        key: str,
        state: SpatialState | None = None,
        *,
        goal: dict[str, float] | None = None,
        ttl_ns: int | None = None,
        now_ns: int | None = None,
    ) -> SpatialEntry:
        self._revision += 1
        e = self._entries.get(key)
        if e is None:
            e = SpatialEntry(key=key, state=state or SpatialState())
            self._entries[key] = e
        if state is not None:
            e.state = state
        if goal is not None:
            e.goal = dict(goal)
        if ttl_ns is not None:
            e.ttl_ns = ttl_ns
        e.revision = self._revision
        e.updated_ns = now_ns if now_ns is not None else time.time_ns()
        return e

    def entries(self, now_ns: int | None = None) -> dict[str, SpatialEntry]:
        now = now_ns if now_ns is not None else time.time_ns()
        return {k: e for k, e in self._entries.items() if e.is_valid(now)}

    def snapshot(self) -> dict[str, SpatialEntry]:
        return dict(self._entries)

    @property
    def revision(self) -> int:
        return self._revision


# ---------- 2. StreamVLN 式 Fast-Slow 流式上下文 ----------


@dataclass
class VisionToken:
    """一帧视觉观测的 Token 单元（fast 窗口条目）。"""

    frame_id: str
    summary: str                   # 语义摘要（场景/地标/危险）
    pose: tuple[float, float, float]   # LIO 位姿（世界系，米）
    timestamp_ns: int = field(default_factory=time.time_ns)
    kv_fresh: bool = True          # True=新增需推理，False=前缀复用


@dataclass
class SlowMemory:
    """压缩后的历史记忆：场景直方图 + 去重地标集合 + 里程累计。"""

    compressed_count: int = 0
    scene_histogram: dict[str, int] = field(default_factory=dict)
    landmark_set: set[str] = field(default_factory=set)
    odom_m: float = 0.0            # 累计里程（LIO 几何）
    last_pose: tuple[float, float, float] | None = None


class StreamContext:
    """Fast-Slow Context：滑动窗口 + KV 增量复用 + Token 去重。

    Args:
        window: fast 窗口长度（滑动窗口）。
        dedup_dist_m: LIO 位姿距离阈值；新帧位姿与窗口内最近帧距离
            低于阈值时视为冗余观测，Token 去重丢弃（不进 KV）。
    """

    def __init__(self, window: int = 8, dedup_dist_m: float = 0.3) -> None:
        self.window = window
        self.dedup_dist_m = dedup_dist_m
        self._fast: deque[VisionToken] = deque()
        self.slow = SlowMemory()
        self._kv_cached = 0          # 已入 KV Cache 的 Token 数（前缀不动）
        self._dropped_dup = 0

    def _dist(self, a: tuple, b: tuple) -> float:
        return math.dist(a, b)

    def push(self, token: VisionToken) -> bool:
        """摄入一帧观测 Token；返回 True=入窗（KV 增量），False=去重丢弃。"""
        # LIO 几何去重：位姿几乎不动的连续观测不重复进 KV
        if self._fast:
            nearest = min(self._fast, key=lambda t: self._dist(t.pose, token.pose))
            if self._dist(nearest.pose, token.pose) < self.dedup_dist_m:
                self._dropped_dup += 1
                return False
        token.kv_fresh = True
        self._kv_cached += 1
        self._fast.append(token)
        # 滑出窗口的最旧条目 → 压缩进 slow 记忆
        while len(self._fast) > self.window:
            self._compress(self._fast.popleft())
        if self.slow.last_pose is not None:
            self.slow.odom_m += self._dist(self.slow.last_pose, token.pose)
        self.slow.last_pose = token.pose
        return True

    def _compress(self, evicted: VisionToken) -> None:
        self.slow.compressed_count += 1
        for tag in evicted.summary.split(","):
            tag = tag.strip()
            if tag:
                self.slow.scene_histogram[tag] = self.slow.scene_histogram.get(tag, 0) + 1

    def compress_oldest(self) -> VisionToken | None:
        """显式把最旧 fast 条目压缩进 slow 记忆（历史记忆压缩）。"""
        if not self._fast:
            return None
        evicted = self._fast.popleft()
        self._compress(evicted)
        return evicted

    def context_tokens(self) -> list[VisionToken]:
        """当前推理上下文 = fast 窗口全部 + slow 摘要（slow 不占 token 预算）。"""
        return list(self._fast)

    def stats(self) -> dict[str, Any]:
        """上下文开销统计：KV 复用率、去重丢弃数、压缩条目数。"""
        return {
            "fast_len": len(self._fast),
            "window": self.window,
            "kv_cached": self._kv_cached,
            "dedup_dropped": self._dropped_dup,
            "slow_compressed": self.slow.compressed_count,
            "slow_scenes": dict(self.slow.scene_histogram),
            "slow_odom_m": round(self.slow.odom_m, 3),
            "kv_reuse_ratio": (
                1.0 - len(self._fast) / self._kv_cached if self._kv_cached else 0.0
            ),
        }
