"""空间状态契约：机器人位姿 + 定位置信度 + 地图版本的有效性检查。

端侧闭环只消费"有效且未过期"的空间状态；无效状态触发安全互锁，
任务进入 hold，而不是带着过期定位继续走。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .task_intent import TaskIntent


def yaw_from_axis_angle(rx: float, ry: float, rz: float) -> float:
    """轴角旋转（rx, ry, rz，弧度）→ yaw（度）。

    ARKitScenes `.traj` 旋转列是轴角表示：方向为旋转轴、模为旋转角。
    用 Rodrigues 公式取旋转矩阵第一列在 xy 平面投影得 yaw。
    零向量输入返回 0.0。
    """
    import math

    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta == 0.0:
        return 0.0
    kx, ky, kz = rx / theta, ry / theta, rz / theta
    c, s = math.cos(theta), math.sin(theta)
    # Rodrigues：R 第一列 = (c + kx²(1-c), kx·ky(1-c) + kz·s, ...)
    r00 = c + kx * kx * (1.0 - c)
    r10 = kx * ky * (1.0 - c) + kz * s
    return math.degrees(math.atan2(r10, r00))


def _row_len(row: object) -> int:
    try:
        return len(row)  # type: ignore[arg-type]
    except TypeError:
        return -1


def blocking_hazards(depth_mm: object, hazards: list[str],
                     corridor_frac: float = 0.4,
                     threshold_mm: float = 900.0) -> list[str]:
    """深度图前向走廊内的最小距离判定障碍是否阻挡通行。

    depth_mm: 二维行序列（嵌套 list/tuple，或任何支持 len() + 索引的 array-like，
    单位毫米）；不依赖 numpy。corridor_frac 取图像中央宽度比例作为通行区域；
    最小深度 < threshold_mm 时 hazards 才返回阻挡。无深度图、行宽不齐或深度
    无效时返回全部 hazards（保守降级：不信任无法判距的场景）。
    """
    h = _row_len(depth_mm)
    if h <= 0:
        return list(hazards)
    w = _row_len(depth_mm[0])  # type: ignore[index]
    if w <= 0:
        return list(hazards)
    cw = max(1, int(w * corridor_frac))
    c0 = (w - cw) // 2
    min_depth: float | None = None
    for row in depth_mm:  # type: ignore[union-attr]
        if _row_len(row) < c0 + cw:
            return list(hazards)
        for c in range(c0, c0 + cw):
            d = row[c]  # type: ignore[index]
            if not isinstance(d, (int, float)) or isinstance(d, bool):
                return list(hazards)
            if d > 0 and (min_depth is None or d < min_depth):
                min_depth = d
    if min_depth is None:
        return list(hazards)
    if min_depth >= threshold_mm:
        return []
    return list(hazards)


@dataclass
class SpatialState:
    """机器人当前空间状态（端侧本地时钟 ns，world 坐标系，米/度）。"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0
    map_id: str = ""
    revision: int = 0
    localization_confidence: float = 0.0
    valid: bool = False
    timestamp_ns: int = 0


class StateGate:
    """空间状态门控：有效期、地图版本一致性、置信度下限。

    Args:
        ttl_ns: 状态有效期（本地时钟）。过期即失效。
        min_confidence: 定位置信度下限（0~1），低于即失效。
    """

    def __init__(self, ttl_ns: int = 500_000_000, min_confidence: float = 0.5) -> None:
        self.ttl_ns = ttl_ns
        self.min_confidence = min_confidence

    def check(self, state: SpatialState, intent: TaskIntent, now_ns: int) -> str | None:
        """返回 None（可放行）或 SAFETY_INTERLOCK:<原因>。"""
        if not state.valid:
            return "SAFETY_INTERLOCK:state_invalid"
        if now_ns - state.timestamp_ns > self.ttl_ns:
            return "SAFETY_INTERLOCK:state_expired"
        if state.localization_confidence < self.min_confidence:
            return "SAFETY_INTERLOCK:low_confidence"
        if intent.map_id and state.map_id and intent.map_id != state.map_id:
            return f"SAFETY_INTERLOCK:map_mismatch:{state.map_id}!={intent.map_id}"
        if state.revision != intent.revision:
            return (
                f"SAFETY_INTERLOCK:revision_mismatch:{state.revision}!={intent.revision}"
            )
        return None
