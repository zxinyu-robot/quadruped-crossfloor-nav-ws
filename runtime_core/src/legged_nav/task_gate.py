"""五级准入闸门：把符号意图与度量状态统一到一次裁决。

框架位置：端侧运行时的**唯一裁决面**。上游（世界空间 / 模型）只能提交 TaskIntent，
下游（BT / 规划 / 控制）只接受通过闸门的意图：

  L1 契约与技能白名单   schema、平台/机器人、技能 ∈ 白名单、无底层控制字段（C6）
  L2 参数有界           目标点有限且在地图边界内、可达距离、deadline/priority 范围（C6）
  L3 状态有效性         位姿置信度 + 有效期 + 地图/代数一致（C1a / C13）
  L4 度量可行性         搜索类技能必须有可用候选；否则降级而不是硬跑
  L5 安全约束           禁区 fail-closed（未知区域一律拒绝）

裁决只有三种结果：ACCEPT / DEGRADED(带模式) / REJECT(带 REJECT_* 或 SAFETY_INTERLOCK:*)。
"语义价值不构成几何可执行许可"在这里落地：L1/L2 只看意图，L3~L5 只信度量状态。
"""
from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field

from .admission import AdmitDecision, TaskAdmission
from .spatial_state import SpatialState, StateGate
from .task_intent import ALLOWED_SKILLS, TaskIntent, check_forbidden_areas

# 需要"有候选才可执行"的技能：搜索型技能的输入是候选前沿，没有候选就是不可执行。
CANDIDATE_SKILLS = ("ExploreFrontier", "Search")  # 契约里搜索技能叫 ExploreFrontier


def _point_in_polygon(x: float, y: float, poly: list[tuple[float, float]]) -> bool:
    """射线法（与 task_intent 内部实现同语义；此处独立以免依赖私有符号）。"""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


class Decision:
    ACCEPT = "ACCEPT"
    DEGRADED = "DEGRADED"
    REJECT = "REJECT"


class DegradeMode:
    STOP_AND_HOLD = "stop_and_hold"      # C13：不可观测时停站/限速，而不是自信地错
    GEOMETRIC_ONLY = "geometric_only"    # 语义层失效时退回纯几何探索
    NO_CANDIDATE = "no_candidate"        # 搜索无候选：保持原位并向上升级


class Level:
    SCHEMA = 1
    BOUNDS = 2
    VALIDITY = 3
    FEASIBILITY = 4
    SAFETY = 5
    ARBITRATION = 6


@dataclass(frozen=True)
class GateLimits:
    """闸门阈值。默认值与 `runtime_contract.json` 的安全默认一致。"""

    min_confidence: float = 0.5
    ttl_ns: int = 500_000_000
    max_goal_dist_m: float = 12.0
    bounds: tuple[float, float, float, float] = (-1.0, -1.0, 11.0, 11.0)
    local_platform: str = ""
    local_robot_id: str = ""
    priority_range: tuple[int, int] = (-100, 100)


@dataclass
class GateResult:
    decision: str
    level: int
    code: str
    detail: str = ""
    degraded_mode: str = ""
    intent: TaskIntent | None = None

    @property
    def ok(self) -> bool:
        return self.decision in (Decision.ACCEPT, Decision.DEGRADED)

    @property
    def accepted(self) -> bool:
        """只有 ACTIVE 执行才需要严格放行的场景用它。"""
        return self.decision == Decision.ACCEPT


@dataclass
class TaskGate:
    """五级闸门 + 任务仲裁（幂等/抢占）。无 ROS 依赖，C++ 权威实现的 Python 对照。"""

    limits: GateLimits = field(default_factory=GateLimits)
    admission: TaskAdmission = field(default_factory=TaskAdmission)
    state_gate: StateGate | None = None

    def __post_init__(self) -> None:
        if self.state_gate is None:
            self.state_gate = StateGate(
                ttl_ns=self.limits.ttl_ns, min_confidence=self.limits.min_confidence
            )

    # ---------------- 五级 ----------------
    def _l1_schema(self, intent: TaskIntent) -> GateResult | None:
        if not intent.skills:
            return GateResult(Decision.REJECT, Level.SCHEMA, "REJECT_NO_SKILL")
        if intent.revision < 0:
            return GateResult(Decision.REJECT, Level.SCHEMA, "REJECT_REVISION_MISSING")
        if self.limits.local_platform and intent.platform != self.limits.local_platform:
            return GateResult(
                Decision.REJECT,
                Level.SCHEMA,
                "REJECT_PLATFORM_MISMATCH",
                f"{intent.platform}!={self.limits.local_platform}",
            )
        if self.limits.local_robot_id and intent.robot_id != self.limits.local_robot_id:
            return GateResult(
                Decision.REJECT,
                Level.SCHEMA,
                "REJECT_ROBOT_MISMATCH",
                f"{intent.robot_id}!={self.limits.local_robot_id}",
            )
        for skill in intent.skills:
            if skill.name not in ALLOWED_SKILLS:
                return GateResult(
                    Decision.REJECT, Level.SCHEMA, "REJECT_UNKNOWN_SKILL", skill.name
                )
        return None

    def _l2_bounds(self, intent: TaskIntent, state: SpatialState | None) -> GateResult | None:
        lo_x, lo_y, hi_x, hi_y = self.limits.bounds
        for skill in intent.skills:
            goal = skill.goal
            if goal is None:
                continue
            if not all(isinstance(v, (int, float)) for v in (goal.x, goal.y, goal.z)):
                return GateResult(Decision.REJECT, Level.BOUNDS, "REJECT_GOAL_NOT_FINITE")
            if not (lo_x <= goal.x <= hi_x and lo_y <= goal.y <= hi_y):
                return GateResult(
                    Decision.REJECT,
                    Level.BOUNDS,
                    "REJECT_GOAL_OUT_OF_BOUNDS",
                    f"({goal.x:.2f},{goal.y:.2f})",
                )
            if state is not None:
                dist = ((goal.x - state.x) ** 2 + (goal.y - state.y) ** 2) ** 0.5
                if dist > self.limits.max_goal_dist_m:
                    return GateResult(
                        Decision.REJECT,
                        Level.BOUNDS,
                        "REJECT_GOAL_TOO_FAR",
                        f"{dist:.2f}m>{self.limits.max_goal_dist_m:.2f}m",
                    )
        lo_p, hi_p = self.limits.priority_range
        if not (lo_p <= intent.priority <= hi_p):
            return GateResult(Decision.REJECT, Level.BOUNDS, "REJECT_PRIORITY_RANGE")
        if intent.deadline_s < 0:
            return GateResult(Decision.REJECT, Level.BOUNDS, "REJECT_DEADLINE_RANGE")
        return None

    def _l3_validity(
        self, intent: TaskIntent, state: SpatialState | None, now_ns: int
    ) -> GateResult | None:
        if state is None:
            return GateResult(
                Decision.DEGRADED,
                Level.VALIDITY,
                "SAFETY_INTERLOCK:no_state",
                "无空间状态",
                DegradeMode.STOP_AND_HOLD,
            )
        reason = self.state_gate.check(state, intent, now_ns)  # type: ignore[union-attr]
        if reason is None:
            return None
        return GateResult(
            Decision.DEGRADED,
            Level.VALIDITY,
            reason,
            "状态过期/置信度不足/代数不一致",
            DegradeMode.STOP_AND_HOLD,
        )

    def _l4_feasibility(
        self, intent: TaskIntent, candidate_count: int
    ) -> GateResult | None:
        needs = any(s.name in CANDIDATE_SKILLS for s in intent.skills)
        if needs and candidate_count <= 0:
            return GateResult(
                Decision.DEGRADED,
                Level.FEASIBILITY,
                "REJECT_NO_CANDIDATE",
                "搜索技能无可用候选前沿",
                DegradeMode.NO_CANDIDATE,
            )
        return None

    def _l5_safety(
        self, intent: TaskIntent, forbidden_areas: dict | None
    ) -> GateResult | None:
        if not forbidden_areas:
            return None
        # ① 意图点名的区域：未知 ID 直接拒（fail closed）
        reason = check_forbidden_areas(intent, forbidden_areas)
        if reason is not None:
            return GateResult(Decision.REJECT, Level.SAFETY, reason, "禁区约束")
        # ② 本地禁行区：不管意图是否点名，目标落在区内即拒（端侧拥有最终否决权）
        for area_id, poly in forbidden_areas.items():
            if not poly or len(poly) < 3:
                continue
            for i, skill in enumerate(intent.skills):
                goal = skill.goal
                if goal is None:
                    continue
                if _point_in_polygon(goal.x, goal.y, poly):
                    return GateResult(
                        Decision.REJECT,
                        Level.SAFETY,
                        f"REJECT_GOAL_IN_FORBIDDEN_AREA:{area_id}@skill{i}",
                        "本地禁行区",
                    )
        return None

    # ---------------- 组合 ----------------
    def evaluate(
        self,
        intent: TaskIntent,
        *,
        state: SpatialState | None,
        now_ns: int,
        candidate_count: int = 0,
        active: TaskIntent | None = None,
        forbidden_areas: dict | None = None,
        record: bool = True,
    ) -> GateResult:
        """按 L1→L5 顺序裁决；首个失败即决定结果。通过后做任务仲裁。"""
        for check in (
            lambda: self._l1_schema(intent),
            lambda: self._l2_bounds(intent, state),
            lambda: self._l3_validity(intent, state, now_ns),
            lambda: self._l4_feasibility(intent, candidate_count),
            lambda: self._l5_safety(intent, forbidden_areas),
        ):
            result = check()
            if result is not None:
                result.intent = intent
                return result

        verdict = self.admission.decide(intent, active)
        if verdict in (AdmitDecision.ACCEPT, AdmitDecision.PREEMPT, AdmitDecision.IDEMPOTENT):
            if record and verdict != AdmitDecision.IDEMPOTENT:
                self.admission.record(intent.task_id)
            return GateResult(Decision.ACCEPT, Level.ARBITRATION, str(verdict), intent=intent)
        return GateResult(Decision.REJECT, Level.ARBITRATION, str(verdict), intent=intent)


def degrade_policy(result: GateResult) -> str:
    """把裁决翻译成 BT 侧动作（唯一解释口径，避免各处各写一套）。"""
    if result.decision == Decision.ACCEPT:
        return "run"
    if result.decision == Decision.REJECT:
        return "reject"
    return {
        DegradeMode.STOP_AND_HOLD: "hold_in_place",
        DegradeMode.GEOMETRIC_ONLY: "geometric_only",
        DegradeMode.NO_CANDIDATE: "hold_in_place",
    }.get(result.degraded_mode, "hold_in_place")


def summarize(results: Iterable[GateResult]) -> dict[str, int]:
    """统计裁决分布（用于 5 级校验的运行时可观测性）。"""
    out: dict[str, int] = {}
    for r in results:
        key = f"L{r.level}:{r.decision}:{r.code}"
        out[key] = out.get(key, 0) + 1
    return out
