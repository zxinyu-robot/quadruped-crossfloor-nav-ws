"""固定地图/目标定义 + 地面真值最短路（用于 SPL）。"""
from __future__ import annotations

import heapq
import math

WALLS = [
    (0.0, 0.0, 10.0, 0.0),
    (10.0, 0.0, 10.0, 10.0),
    (10.0, 10.0, 0.0, 10.0),
    (0.0, 10.0, 0.0, 0.0),
    (5.0, 0.0, 5.0, 4.4),
    (5.0, 5.6, 5.0, 10.0),
    (0.0, 5.0, 4.4, 5.0),
    (5.6, 5.0, 10.0, 5.0),
]

START = (1.5, 1.5)

ROOMS = {
    "A": (2.5, 2.5),
    "B": (7.5, 2.5),
    "C": (2.5, 7.5),
    "D": (7.5, 7.5),
}

GOALS = [
    ("g01", 3.6, 1.4, "A"),
    ("g02", 1.4, 3.6, "A"),
    ("g03", 3.6, 3.6, "A"),
    ("g04", 2.6, 4.0, "A"),
    ("g05", 4.2, 2.2, "A"),
    ("g06", 7.0, 2.0, "B"),
    ("g07", 8.4, 1.8, "B"),
    ("g08", 6.8, 3.6, "B"),
    ("g09", 8.5, 3.5, "B"),
    ("g10", 7.6, 2.8, "B"),
    ("g11", 2.0, 7.0, "C"),
    ("g12", 3.5, 6.8, "C"),
    ("g13", 1.8, 8.6, "C"),
    ("g14", 3.6, 8.5, "C"),
    ("g15", 2.6, 7.6, "C"),
    ("g16", 7.0, 7.0, "D"),
    ("g17", 8.4, 6.8, "D"),
    ("g18", 6.8, 8.6, "D"),
    ("g19", 8.5, 8.5, "D"),
    ("g20", 7.6, 7.6, "D"),
]

RES = 0.05
PAD = 0.6
X_MIN, X_MAX = -PAD, 10.0 + PAD
Y_MIN, Y_MAX = -PAD, 10.0 + PAD
NX = int(round((X_MAX - X_MIN) / RES))
NY = int(round((Y_MAX - Y_MIN) / RES))


def _seg_dist(px: float, py: float, seg) -> float:
    x1, y1, x2, y2 = seg
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def build_occ(clearance: float = 0.25):
    occ = [[False] * NX for _ in range(NY)]
    for j in range(NY):
        py = Y_MIN + (j + 0.5) * RES
        for i in range(NX):
            px = X_MIN + (i + 0.5) * RES
            blocked = any(_seg_dist(px, py, w) <= clearance for w in WALLS)
            occ[j][i] = blocked
    return occ


_OCC = None


def _occ():
    global _OCC
    if _OCC is None:
        _OCC = build_occ()
    return _OCC


def to_cell(x: float, y: float) -> tuple[int, int]:
    i = int((x - X_MIN) / RES)
    j = int((y - Y_MIN) / RES)
    return max(0, min(NX - 1, i)), max(0, min(NY - 1, j))


def shortest_path_len(start: tuple[float, float], goal: tuple[float, float]) -> float:
    occ = _occ()
    si, sj = to_cell(*start)
    gi, gj = to_cell(*goal)
    if occ[sj][si] or occ[gj][gi]:
        # 端点落在膨胀墙内时，就近找一个自由像元
        def nearest_free(i, j):
            best = None
            for r in range(1, 12):
                for dj in range(-r, r + 1):
                    for di in range(-r, r + 1):
                        ni, nj = i + di, j + dj
                        if 0 <= ni < NX and 0 <= nj < NY and not occ[nj][ni]:
                            d = di * di + dj * dj
                            if best is None or d < best[0]:
                                best = (d, ni, nj)
                if best:
                    return best[1], best[2]
            return i, j
        si, sj = nearest_free(si, sj)
        gi, gj = nearest_free(gi, gj)
    dist = {(si, sj): 0.0}
    pq = [(0.0, si, sj)]
    neigh = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    while pq:
        d, i, j = heapq.heappop(pq)
        if (i, j) == (gi, gj):
            return d * RES
        if d > dist.get((i, j), float("inf")):
            continue
        for di, dj, c in neigh:
            ni, nj = i + di, j + dj
            if 0 <= ni < NX and 0 <= nj < NY and not occ[nj][ni]:
                nd = d + c
                if nd < dist.get((ni, nj), float("inf")):
                    dist[(ni, nj)] = nd
                    heapq.heappush(pq, (nd, ni, nj))
    return float("inf")


def path_len(pts: list[tuple[float, float]]) -> float:
    total = 0.0
    for a, b in zip(pts, pts[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total
