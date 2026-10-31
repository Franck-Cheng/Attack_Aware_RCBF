#!/usr/bin/env python3
"""安全过滤器核心 — strict QP / soft QP / min-risk LP / margin computation。

针对二维输入 (v, ω) 的 box 约束 + 线性 CBF rows。
若 scipy 不可用，回退到 grid-search。

v2.1: 统一 U_safe = U_phy ⊖ A(ρ) 计算，消除硬编码参数。
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import math
import numpy as np

try:
    from scipy.optimize import linprog, minimize, Bounds, LinearConstraint

    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

from .barrier_builder import CBFConstraintRow


# ============================================================
# U_safe = U_phy ⊖ A(ρ) — Pontryagin difference 计算
# ============================================================

def compute_safe_bounds(
    v_phy_min: float,
    v_phy_max: float,
    w_phy_max: float,
    a_v_max: float,
    a_w_max: float,
    rho: float = 1.0,
) -> dict:
    """计算攻击安全输入集合 𝒰_safe = 𝒰_phy ⊖ ρ·𝒜.

    Args:
        v_phy_min: 物理线速度下限 [m/s]（通常为负值，表示后退能力）
        v_phy_max: 物理线速度上限 [m/s]
        w_phy_max: 物理角速度幅值上限 [rad/s]（假设对称 ±w_phy_max）
        a_v_max: 线速度攻击幅值上界 [m/s]
        a_w_max: 角速度攻击幅值上界 [rad/s]
        rho: 攻击强度缩放因子

    Returns:
        dict with keys: v_min, v_max, w_min, w_max
    """
    a_v = rho * a_v_max
    a_w = rho * a_w_max

    v_safe_min = v_phy_min + a_v
    v_safe_max = v_phy_max - a_v
    w_safe_min = -w_phy_max + a_w
    w_safe_max = w_phy_max - a_w

    # 安全检查：输入域不应塌缩
    if v_safe_min >= v_safe_max:
        raise ValueError(
            f"U_safe 线速度域塌缩: [{v_safe_min:.3f}, {v_safe_max:.3f}]. "
            f"请减小攻击强度或放宽物理极限。"
        )
    if w_safe_min >= w_safe_max:
        raise ValueError(
            f"U_safe 角速度域塌缩: [{w_safe_min:.3f}, {w_safe_max:.3f}]. "
            f"请减小攻击强度或放宽物理极限。"
        )

    return {
        "v_min": v_safe_min,
        "v_max": v_safe_max,
        "w_min": w_safe_min,
        "w_max": w_safe_max,
    }


# ============================================================
# 轮级变化率约束 — 差速机器人菱形约束
# ============================================================

def compute_wheel_rate_bounds(
    wheel_radius: float,
    wheel_base: float,
    max_wheel_accel: float,
    dt: float,
) -> dict:
    """从轮加速度限制导出 (v, ω) 空间的变化率约束。

    差速运动学:
        φ̇_L = (v - d/2·ω) / r
        φ̇_R = (v + d/2·ω) / r

    轮加速度约束 |φ̈| ≤ α_max 导出:
        |Δv - d/2·Δω| ≤ r·α_max·Δt
        |Δv + d/2·Δω| ≤ r·α_max·Δt

    Args:
        wheel_radius: 轮半径 r [m]
        wheel_base: 轮距 d [m]
        max_wheel_accel: 轮角加速度上限 α_max [rad/s²]
        dt: 控制周期 Δt [s]

    Returns:
        dict with keys:
            A_rate: (4,2) 矩阵 — 4条线性不等式
            b_rate: (4,) 向量 — 右端项
            description: 人类可读约束说明
    """
    r = wheel_radius
    d = wheel_base
    bound = r * max_wheel_accel * dt

    # 4 条线性不等式: A_rate @ [Δv, Δω] ≤ b_rate
    # |Δv - d/2·Δω| ≤ bound  →  -bound ≤ Δv - d/2·Δω ≤ bound
    # |Δv + d/2·Δω| ≤ bound  →  -bound ≤ Δv + d/2·Δω ≤ bound

    half_d = d / 2.0

    A_rate = np.array([
        [1.0, -half_d],    #  Δv - d/2·Δω ≤ bound
        [-1.0, half_d],    # -Δv + d/2·Δω ≤ bound
        [1.0, half_d],     #  Δv + d/2·Δω ≤ bound
        [-1.0, -half_d],   # -Δv - d/2·Δω ≤ bound
    ], dtype=float)

    b_rate = np.array([bound, bound, bound, bound], dtype=float)

    return {
        "A_rate": A_rate,
        "b_rate": b_rate,
        "bound": bound,
        "wheel_radius": r,
        "wheel_base": d,
        "max_wheel_accel": max_wheel_accel,
        "dt": dt,
        "description": (
            f"轮加速度 α_max={max_wheel_accel} rad/s² 导出 "
            f"|Δv ± {half_d:.3f}·Δω| ≤ {bound:.4f} per Δt={dt}s"
        ),
    }


@dataclass
class FilterResult:
    status: str            # "strict" | "soft" | "min_risk"
    u_safe: np.ndarray     # [v, w]
    eta_h: float           # hard-safety margin
    eta_s: float           # task-safety margin (简化版)
    num_constraints: int
    min_clearance: float
    qp_feasible: bool
    soft_slack: float      # soft QP 的总 slack


class SafetyFilterCore:
    """攻击感知 CBF 安全过滤器核心算法。

    与 Python 仿真完全一致的逻辑:
      1. strict QP: min ||u - u_nom||²  s.t. hard rows + task row, u ∈ 𝒰_safe
      2. soft QP:   引入 task slack
      3. min-risk:  仅 hard rows, 最大化 min margin

    U_safe 通过 compute_safe_bounds() 或直接参数传入;
    推荐使用 from_physical_limits() 工厂方法自动计算。
    """

    def __init__(
        self,
        v_safe_min: float = -0.60,
        v_safe_max: float = 0.60,
        w_safe_max: float = 2.20,
        *,
        # 攻击参数
        a_v_max: float = 0.4,
        a_w_max: float = 1.6,
        # 轮级参数（用于变化率约束）
        wheel_radius: float = 0.033,
        wheel_base: float = 0.160,
        max_wheel_accel: float = 1.0,
        enable_rate_constraints: bool = False,
        # QP 权重
        w_v: float = 2.0,
        w_w: float = 0.8,
        w_slack: float = 80.0,
        slack_alpha_max: float = 2.0,
        slack_r_max: float = 1.0,
        v_cap_progress: float = 0.50,
        v_back: float = 0.50,
        v_nom_max: float = 0.50,
    ):
        # U_safe = [-0.60, 0.60] × [-2.20, 2.20] (直设)
        self.v_safe_min = v_safe_min
        self.v_safe_max = v_safe_max
        self.w_safe_max = w_safe_max

        # 攻击参数
        self.a_v_max = a_v_max
        self.a_w_max = a_w_max

        # 轮级变化率约束参数
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_wheel_accel = max_wheel_accel
        self.enable_rate_constraints = enable_rate_constraints

        # QP 权重
        self.w_v = w_v
        self.w_w = w_w
        self.w_slack = w_slack
        self.slack_alpha_max = slack_alpha_max
        self.slack_r_max = slack_r_max
        self.v_cap_progress = v_cap_progress
        self.v_back = v_back
        self.v_nom_max = v_nom_max

        # 上一周期控制输出（用于变化率约束）
        self._prev_u: Optional[np.ndarray] = None

    @classmethod
    def from_physical_limits(
        cls,
        v_phy_min: float,
        v_phy_max: float,
        w_phy_max: float,
        a_v_max: float,
        a_w_max: float,
        rho: float = 1.0,
        **kwargs,
    ) -> "SafetyFilterCore":
        """工厂方法：从物理边界和攻击边界自动计算 𝒰_safe = 𝒰_phy ⊖ ρ·𝒜.

        Args:
            v_phy_min: 物理线速度下限 [m/s]
            v_phy_max: 物理线速度上限 [m/s]
            w_phy_max: 物理角速度上限 [rad/s]（对称 ±w_phy_max）
            a_v_max: 线速度攻击上界 [m/s]
            a_w_max: 角速度攻击上界 [rad/s]
            rho: 攻击强度缩放
            **kwargs: 传递给 __init__ 的其他参数
                (wheel_radius, wheel_base, max_wheel_accel, enable_rate_constraints, etc.)

        Returns:
            SafetyFilterCore 实例，U_safe 已通过 Pontryagin difference 计算
        """
        bounds = compute_safe_bounds(
            v_phy_min=v_phy_min,
            v_phy_max=v_phy_max,
            w_phy_max=w_phy_max,
            a_v_max=a_v_max,
            a_w_max=a_w_max,
            rho=rho,
        )
        return cls(
            v_safe_min=bounds["v_min"],
            v_safe_max=bounds["v_max"],
            w_safe_max=bounds["w_max"],
            a_v_max=a_v_max,
            a_w_max=a_w_max,
            rho=rho,
            v_phy_min=v_phy_min,
            v_phy_max=v_phy_max,
            w_phy_max=w_phy_max,
            **kwargs,
        )

    # ─── task constraint builders ───────────────────────────
    def _heading_constraint(
        self, alpha: float, r: float
    ) -> Tuple[np.ndarray, float]:
        """heading mode task row: 朝目标转向。

        约束: （角速度主导的转向约束）
        对大 r 用饱和避免 1/r 发散。
        """
        a_v = self.a_v_max
        a_w = self.a_w_max
        c_alpha = 1.25
        r_safe = max(r, 0.3)
        r_eff = max(r_safe, 1.0)
        coeff_v = alpha * (math.sin(alpha) / r_eff)
        coeff_w = -alpha
        rhs = (
            -0.5 * c_alpha * alpha * alpha
            + abs(alpha) * (a_w + (a_v / r_eff) * abs(math.sin(alpha)))
        )
        return np.array([coeff_v, coeff_w, -1.0, 0.0], dtype=float), rhs

    def _progress_constraint(
        self, alpha: float, r: float
    ) -> Tuple[np.ndarray, float]:
        """progress mode task row: 朝目标前进。

        约束: cos(α)·v ≥ κ_r · min(r, r_sat) + a_v_max·cos(α)
        r_sat 限制确保大距离下约束在速度范围内可行。
        """
        a_v = self.a_v_max
        kappa_r = 0.55
        r_sat = self.v_nom_max / max(kappa_r, 1e-4)
        r_eff = min(r, r_sat)
        coeff_v = -math.cos(alpha)
        rhs = -(kappa_r * r_eff + a_v * max(math.cos(alpha), 0.0))
        return np.array([coeff_v, 0.0, 0.0, -1.0], dtype=float), rhs

    # ─── feasibility checks ─────────────────────────────────
    def _check_feasible(
        self, A: np.ndarray, b: np.ndarray, lb: np.ndarray, ub: np.ndarray
    ) -> bool:
        """LP feasibility check。"""
        if not _HAS_SCIPY:
            return self._grid_feasible(A, b, lb, ub)

        res = linprog(
            np.zeros(A.shape[1]),
            A_ub=A,
            b_ub=b,
            bounds=list(zip(lb, ub)),
            method="highs",
        )
        return bool(res.success)

    def _grid_feasible(
        self, A: np.ndarray, b: np.ndarray, lb: np.ndarray, ub: np.ndarray
    ) -> bool:
        """Grid search feasibility fallback (仅 2D)。"""
        for v in np.linspace(lb[0], ub[0], 21):
            for w in np.linspace(lb[1], ub[1], 41):
                x = np.array([v, w])
                if A.shape[1] > 2:
                    x = np.append(x, [0.0] * (A.shape[1] - 2))
                if np.all(A @ x <= b + 1e-6):
                    return True
        return False

    # ─── Rate constraint builder ────────────────────────────
    def _build_rate_constraints(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """构建轮级变化率约束 A_rate @ u ≤ b_rate'.

        仅当 enable_rate_constraints=True 且 _prev_u 有效时返回非None。
        b_rate' = b_rate + A_rate @ u_{k-1}

        Returns:
            (A_rate, b_rate_prime) 或 (None, None)
        """
        if not self.enable_rate_constraints or self._prev_u is None:
            return None, None

        dt = 0.1  # 默认控制周期 10Hz，与 Nav2 一致
        rb = compute_wheel_rate_bounds(
            wheel_radius=self.wheel_radius,
            wheel_base=self.wheel_base,
            max_wheel_accel=self.max_wheel_accel,
            dt=dt,
        )

        A_rate = rb["A_rate"]          # (4,2)
        b_rate = rb["b_rate"]          # (4,)
        # b_rate' = b_rate + A_rate @ u_{k-1}
        # A_rate @ u_k ≤ b_rate + A_rate @ u_{k-1}
        b_rate_prime = b_rate + A_rate @ self._prev_u

        return A_rate, b_rate_prime

    # ─── QP solvers ─────────────────────────────────────────
    def _solve_strict_qp(
        self,
        u_nom: np.ndarray,
        A_hard: np.ndarray,
        b_hard: np.ndarray,
        mode: str,
        alpha: float = 0.0,
        r: float = 0.0,
    ) -> Optional[np.ndarray]:
        """严格模式QP：所有约束不可松弛。"""
        # Build full constraint matrix
        A_list = [A_hard]
        b_list = [b_hard]

        # task constraint（使用实例自身的攻击参数，不再硬编码）
        if mode == "heading":
            row, rhs = self._heading_constraint(alpha, r)
        else:
            row, rhs = self._progress_constraint(alpha, r)
        A_list.append(row[:2].reshape(1, -1))
        b_list.append(np.array([rhs]))

        # 轮级变化率约束（可选）
        A_rate, b_rate_p = self._build_rate_constraints()
        if A_rate is not None:
            A_list.append(A_rate)
            b_list.append(b_rate_p)

        A_full = np.vstack(A_list)
        b_full = np.concatenate(b_list)

        lb = np.array([self.v_safe_min, -self.w_safe_max])
        ub = np.array([self.v_safe_max, self.w_safe_max])

        if mode == "heading":
            lb[0] = max(lb[0], -self.v_back)
            ub[0] = min(ub[0], self.v_cap_progress)
        else:
            lb[0] = max(lb[0], 0.0)
            ub[0] = min(ub[0], self.v_nom_max)

        if not _HAS_SCIPY:
            return self._grid_solve_qp(u_nom, A_full, b_full, lb, ub)

        # scipy SLSQP
        def objective(x):
            return 0.5 * self.w_v * (x[0] - u_nom[0]) ** 2 + 0.5 * self.w_w * (x[1] - u_nom[1]) ** 2

        x0 = np.array([
            float(np.clip(u_nom[0], lb[0], ub[0])),
            float(np.clip(u_nom[1], lb[1], ub[1])),
        ])

        res = minimize(
            objective,
            x0=x0,
            method="SLSQP",
            bounds=Bounds(lb, ub),
            constraints=[
                LinearConstraint(A_full, -np.inf * np.ones(len(b_full)), b_full)
            ],
            options={"maxiter": 200, "ftol": 1e-9, "disp": False},
        )

        if res.success and np.max(A_full @ res.x - b_full) < 1e-5:
            return res.x
        return None

    def _solve_soft_qp(
        self,
        u_nom: np.ndarray,
        A_hard: np.ndarray,
        b_hard: np.ndarray,
        mode: str,
        alpha: float = 0.0,
        r: float = 0.0,
    ) -> Optional[Tuple[np.ndarray, float]]:
        """软约束模式：hard rows 不可松弛，task row 允许 slack。"""
        A_list = [np.hstack([A_hard, np.zeros((A_hard.shape[0], 2))])]
        b_list = [b_hard]

        if mode == "heading":
            row, rhs = self._heading_constraint(alpha, r)
        else:
            row, rhs = self._progress_constraint(alpha, r)
        A_list.append(row.reshape(1, -1))
        b_list.append(np.array([rhs]))

        # 轮级变化率约束（可选）：对 [v, w] 施加，slack列填0
        A_rate, b_rate_p = self._build_rate_constraints()
        if A_rate is not None:
            A_rate_expanded = np.hstack([A_rate, np.zeros((A_rate.shape[0], 2))])
            A_list.append(A_rate_expanded)
            b_list.append(b_rate_p)

        A_full = np.vstack(A_list)
        b_full = np.concatenate(b_list)

        lb = np.array([
            self.v_safe_min, -self.w_safe_max, 0.0, 0.0
        ])
        ub = np.array([
            self.v_safe_max, self.w_safe_max,
            self.slack_alpha_max, self.slack_r_max,
        ])

        if mode == "heading":
            lb[0] = max(lb[0], -self.v_back)
            ub[0] = min(ub[0], self.v_cap_progress)
        else:
            lb[0] = max(lb[0], 0.0)
            ub[0] = min(ub[0], self.v_nom_max)

        if not _HAS_SCIPY:
            # grid search with slack
            best_u = None
            best_cost = float("inf")
            best_slack = float("inf")
            for v in np.linspace(lb[0], ub[0], 21):
                for w in np.linspace(lb[1], ub[1], 41):
                    for sa in np.linspace(0, self.slack_alpha_max, 6):
                        for sr in np.linspace(0, self.slack_r_max, 6):
                            x = np.array([v, w, sa, sr])
                            if np.all(A_full @ x <= b_full + 1e-6):
                                cost = (
                                    0.5 * self.w_v * (v - u_nom[0]) ** 2
                                    + 0.5 * self.w_w * (w - u_nom[1]) ** 2
                                    + 0.5 * self.w_slack * (sa ** 2 + sr ** 2)
                                )
                                if cost < best_cost:
                                    best_cost = cost
                                    best_u = x.copy()
                                    best_slack = sa + sr
            if best_u is not None:
                return best_u[:2], best_slack
            return None

        def objective(x):
            return (
                0.5 * self.w_v * (x[0] - u_nom[0]) ** 2
                + 0.5 * self.w_w * (x[1] - u_nom[1]) ** 2
                + 0.5 * self.w_slack * (x[2] ** 2 + x[3] ** 2)
            )

        x0 = np.array([
            float(np.clip(u_nom[0], lb[0], ub[0])),
            float(np.clip(u_nom[1], lb[1], ub[1])),
            0.0,
            0.0,
        ])

        res = minimize(
            objective,
            x0=x0,
            method="SLSQP",
            bounds=Bounds(lb, ub),
            constraints=[
                LinearConstraint(A_full, -np.inf * np.ones(len(b_full)), b_full)
            ],
            options={"maxiter": 200, "ftol": 1e-9, "disp": False},
        )

        if res.success and np.max(A_full @ res.x - b_full) < 1e-5:
            slack = float(res.x[2] + res.x[3])
            return res.x[:2], slack
        return None

    def _grid_solve_qp(
        self,
        u_nom: np.ndarray,
        A: np.ndarray,
        b: np.ndarray,
        lb: np.ndarray,
        ub: np.ndarray,
    ) -> Optional[np.ndarray]:
        """Grid search QP fallback."""
        best_u = None
        best_cost = float("inf")
        for v in np.linspace(lb[0], ub[0], 31):
            for w in np.linspace(lb[1], ub[1], 61):
                x = np.array([v, w])
                if np.all(A @ x <= b + 1e-6):
                    cost = 0.5 * self.w_v * (v - u_nom[0]) ** 2 + 0.5 * self.w_w * (w - u_nom[1]) ** 2
                    if cost < best_cost:
                        best_cost = cost
                        best_u = x.copy()
        return best_u

    # ─── hard-margin LP (η_H) ───────────────────────────────
    def compute_eta_h(
        self,
        rows: List[CBFConstraintRow],
    ) -> Tuple[float, Optional[np.ndarray]]:
        """计算硬安全裕度 η_H = max_{u∈U_safe} min_i (rhs_i - ℓ_i^T u)。

        等价 LP:
            max η  s.t.  a_v v + a_w w + η <= rhs, ∀ rows, [v,w] ∈ U_safe
        """
        if len(rows) == 0:
            return float("inf"), np.zeros(2)

        # 构造: -a_v v - a_w w + η <= rhs → [-a_v, -a_w, 1] @ [v,w,η] <= rhs
        # (与 filter() 中 A_hard 的负系数约定一致)
        A = []
        b = []
        for row in rows:
            A.append([-row.a_v, -row.a_w, 1.0])
            b.append(row.rhs)
        A = np.array(A, dtype=float)
        b = np.array(b, dtype=float)

        lb = [self.v_safe_min, -self.w_safe_max, -1e6]
        ub = [self.v_safe_max, self.w_safe_max, 1e6]
        c = [0.0, 0.0, -1.0]  # minimize -η

        if not _HAS_SCIPY:
            # grid search for η
            best_eta = -float("inf")
            best_u = None
            for v in np.linspace(lb[0], ub[0], 41):
                for w in np.linspace(lb[1], ub[1], 81):
                    margins = [
                        row.rhs + (row.a_v * v + row.a_w * w) for row in rows
                    ]
                    eta = min(margins)
                    if eta > best_eta:
                        best_eta = eta
                        best_u = np.array([v, w])
            return best_eta, best_u

        res = linprog(
            c,
            A_ub=A,
            b_ub=b,
            bounds=list(zip(lb, ub)),
            method="highs",
        )

        if res.success:
            eta = -res.fun  # objective was minimize -η
            u_star = res.x[:2]
            return eta, u_star
        return -float("inf"), None

    # ─── Attack-aware safety utilities ─────────────────
    def can_be_fully_cancelled(self, u: np.ndarray) -> bool:
        """检查命令是否可能被 bounded attack 完全抵消。
        
        若 |v| <= a_v_max 且 |w| <= a_w_max，则存在攻击使 u+a ≈ 0。
        """
        return (
            abs(float(u[0])) <= self.a_v_max + 1e-8
            and abs(float(u[1])) <= self.a_w_max + 1e-8
        )

    def guaranteed_residual(self, u: np.ndarray) -> tuple:
        """计算最坏情形下的保证残余运动。"""
        gv = max(0.0, abs(float(u[0])) - self.a_v_max)
        gw = max(0.0, abs(float(u[1])) - self.a_w_max)
        return float(gv), float(gw)

    def _hard_rows_satisfied(self, u: np.ndarray, A_hard, b_hard) -> bool:
        if len(A_hard) == 0: return True
        return bool(np.max(A_hard @ u[:2] - b_hard) <= 1e-6)

    def robust_motion_recovery(
        self,
        u_nav: np.ndarray,
        A_hard: np.ndarray,
        b_hard: np.ndarray,
        mode: str = "progress",
    ) -> Optional[Tuple[np.ndarray, str, dict]]:
        """鲁棒运动恢复网格搜索。
        
        在 U_safe 中搜索保证净运动且满足所有 hard CBF rows 的命令。
        Nv=21, Nw=41 网格。
        """
        Nv, Nw = 21, 41
        v_vals = np.linspace(self.v_safe_min, self.v_safe_max, Nv)
        w_vals = np.linspace(-self.w_safe_max, self.w_safe_max, Nw)

        min_gv = max(0.02, 0.08 * self.a_v_max)
        min_gw = max(0.03, 0.08 * self.a_w_max)

        best_u = None
        best_score = -float("inf")
        best_info = None

        for v in v_vals:
            for w in w_vals:
                u = np.array([float(v), float(w)])

                if len(A_hard) > 0:
                    if np.max(A_hard @ u - b_hard) > 1e-6:
                        continue

                gv, gw = self.guaranteed_residual(u)

                if gv < min_gv and gw < min_gw:
                    continue

                # 评分：优先保证角速度净运动（可改变局部几何）
                score = (
                    1.00 * gw
                    + 0.25 * gv
                    - 0.05 * ((v - u_nav[0])**2 + (w - u_nav[1])**2)
                )

                if score > best_score:
                    best_score = score
                    best_u = u
                    best_info = {
                        "guaranteed_v": float(gv),
                        "guaranteed_w": float(gw),
                        "score": float(score),
                    }

        if best_u is None:
            return None, "no_recovery_found", {}

        return best_u, "robust_motion_recovery", best_info

    # ─── Robust Predictive Safety Filter (RPSF) ───────────
    def robust_predictive_filter(
        self,
        u_filtered: np.ndarray,
        u_nom: np.ndarray,
        state: np.ndarray,
        lidar_points_world: np.ndarray,
        body_disks: list,
        horizon: int = 16,
        dt: float = 0.05,
        safety_margin: float = 0.03,
        Nv: int = 7,
        Nw: int = 11,
    ) -> Tuple[np.ndarray, str, dict]:
        """对候选 u_sf 做多步预测，拒绝会被任一攻击候选导致碰撞的命令。"""
        # 无 LiDAR 数据时直接返回
        if len(lidar_points_world) == 0:
            return u_filtered, "rpsf_no_lidar", {}
        
        # 攻击候选网格 (5×5=25)
        a_v_vals = [-self.a_v_max, -0.5*self.a_v_max, 0.0, 0.5*self.a_v_max, self.a_v_max]
        a_w_vals = [-self.a_w_max, -0.5*self.a_w_max, 0.0, 0.5*self.a_w_max, self.a_w_max]
        
        # SF 候选网格 (reduced)
        v_sf_vals = np.linspace(self.v_safe_min, self.v_safe_max, Nv)
        w_sf_vals = np.linspace(-self.w_safe_max, self.w_safe_max, Nw)
        
        best_u = u_filtered.copy()
        best_score = -float("inf")
        best_info = {}
        any_safe = False
        
        for v_sf in v_sf_vals:
            for w_sf in w_sf_vals:
                u_sf = np.array([float(v_sf), float(w_sf)])
                safe_for_all = True
                worst_future_h = float("inf")
                
                for a_v in a_v_vals:
                    if not safe_for_all:
                        break
                    for a_w in a_w_vals:
                        u_act_v = float(np.clip(v_sf + a_v, -0.60, 0.60))
                        u_act_w = float(np.clip(w_sf + a_w, -2.20, 2.20))
                        
                        x, y, th = float(state[0]), float(state[1]), float(state[2])
                        min_h = float("inf")
                        
                        for _ in range(horizon):
                            x += dt * u_act_v * math.cos(th)
                            y += dt * u_act_v * math.sin(th)
                            th += dt * u_act_w
                            c, s = math.cos(th), math.sin(th)
                            for dx, dy, rd in body_disks:
                                wx = x + c*dx - s*dy
                                wy = y + s*dx + c*dy
                                if len(lidar_points_world) > 0:
                                    dists = np.sqrt(
                                        (lidar_points_world[:, 0] - wx)**2 +
                                        (lidar_points_world[:, 1] - wy)**2
                                    )
                                    if len(dists) > 0:
                                        h_disk = float(np.min(dists)) - rd - safety_margin
                                        if h_disk < min_h:
                                            min_h = h_disk
                            
                            if min_h < 0:
                                break
                        
                        if min_h < worst_future_h:
                            worst_future_h = min_h
                        if min_h < 0:
                            safe_for_all = False
                            break
                
                if not safe_for_all:
                    continue
                
                any_safe = True
                tracking_cost = ((v_sf - u_nom[0])**2 + (w_sf - u_nom[1])**2)
                score = 1.0 * worst_future_h + 0.3 * v_sf - 0.1 * tracking_cost
                
                if score > best_score:
                    best_score = score
                    best_u = u_sf.copy()
                    best_info = {"worst_future_h": float(worst_future_h), "score": float(score)}
        
        if any_safe:
            return best_u, "rpsf_safe", best_info
        
        # Emergency: find least-dangerous command
        # 修复: 紧急模式禁止倒车 (v ≥ 0)。倒车会与 Nav2 前向意图直接对抗
        # 造成振荡。制动+原地转向是经典安全行为, 不与名义控制对抗。
        best_h = -float("inf")
        best_u_emergency = np.array([0.0, 0.0])
        for v_sf in v_sf_vals:
            if v_sf < 0.0:
                continue  # 紧急模式不倒车
            for w_sf in w_sf_vals:
                min_h_all = float("inf")
                for a_v in a_v_vals:
                    for a_w in a_w_vals:
                        u_act_v = float(np.clip(v_sf + a_v, -0.60, 0.60))
                        u_act_w = float(np.clip(w_sf + a_w, -2.20, 2.20))
                        x, y, th = float(state[0]), float(state[1]), float(state[2])
                        for _ in range(horizon):
                            x += dt * u_act_v * math.cos(th)
                            y += dt * u_act_v * math.sin(th)
                            th += dt * u_act_w
                            c, s = math.cos(th), math.sin(th)
                            for dx, dy, rd in body_disks:
                                wx = x + c*dx - s*dy
                                wy = y + s*dx + c*dy
                                if len(lidar_points_world) > 0:
                                    dists = np.sqrt(
                                        (lidar_points_world[:, 0] - wx)**2 +
                                        (lidar_points_world[:, 1] - wy)**2
                                    )
                                    if len(dists) > 0:
                                        h_disk = float(np.min(dists)) - rd - safety_margin
                                        if h_disk < min_h_all:
                                            min_h_all = h_disk
                if min_h_all > best_h:
                    best_h = min_h_all
                    best_u_emergency = np.array([float(v_sf), float(w_sf)])
        
        return best_u_emergency, "rpsf_emergency", {"best_worst_h": float(best_h)}

    # ─── full filter pipeline ───────────────────────────────
    def filter(
        self,
        u_nom: np.ndarray,
        rows: List[CBFConstraintRow],
        alpha: float = 0.0,
        r: float = 0.0,
        mode: str = "progress",
    ) -> FilterResult:
        """完整的 strict → soft → min-risk 级联过滤。

        Args:
            u_nom: [v, w] 名义控制输入
            rows: CBF 约束行列表
            alpha: 目标方向误差
            r: 到目标距离
            mode: "heading" | "progress"

        Returns:
            FilterResult
        """
        n_rows = len(rows)
        min_h = min((row.h_value for row in rows), default=float("inf"))

        # 无 CBF 约束时，直接限幅放行名义指令
        if n_rows == 0:
            u_clamped = np.array([
                float(np.clip(u_nom[0], self.v_safe_min, self.v_safe_max)),
                float(np.clip(u_nom[1], -self.w_safe_max, self.w_safe_max)),
            ])
            return FilterResult(
                status="strict",
                u_safe=u_clamped,
                eta_h=999.0,
                eta_s=999.0,
                num_constraints=0,
                min_clearance=float("inf"),
                qp_feasible=True,
                soft_slack=0.0,
            )

        # 构造 hard-safe 约束矩阵 (用于 scipy: A_ub @ x <= b_ub)
        # CBF: ḣ = coeff_v·v + coeff_w·ω ≥ -αh
        #   →  -coeff_v·v - coeff_w·ω ≤ αh + |coeff_v|·a_v_max + |coeff_w|·a_w_max
        # A_hard 取负系数! (系数 coeff_v=unit·g_v, 前方障碍 coeff_v=-1,
        # 需 -coeff_v=+1 才能形成 v ≤ rhs 的上界约束)
        A_hard = np.array([[-row.a_v, -row.a_w] for row in rows], dtype=float)
        b_hard = np.array([row.rhs for row in rows], dtype=float)

        # 1) strict QP
        u_strict = self._solve_strict_qp(u_nom, A_hard, b_hard, mode, alpha, r)
        if u_strict is not None:
            self._prev_u = u_strict.copy()
            eta_h, _ = self.compute_eta_h(rows)
            return FilterResult(
                status="strict",
                u_safe=u_strict,
                eta_h=eta_h if not math.isinf(eta_h) else 999.0,
                eta_s=eta_h if not math.isinf(eta_h) else 999.0,
                num_constraints=n_rows,
                min_clearance=min_h,
                qp_feasible=True,
                soft_slack=0.0,
            )

        # 2) soft QP (task slack allowed)
        soft_res = self._solve_soft_qp(u_nom, A_hard, b_hard, mode, alpha, r)
        if soft_res is not None:
            u_soft, slack = soft_res
            self._prev_u = u_soft.copy()
            eta_h, _ = self.compute_eta_h(rows)
            return FilterResult(
                status="soft",
                u_safe=u_soft,
                eta_h=eta_h if not math.isinf(eta_h) else 999.0,
                eta_s=eta_h if not math.isinf(eta_h) else 999.0,
                num_constraints=n_rows,
                min_clearance=min_h,
                qp_feasible=True,
                soft_slack=slack,
            )

        # 3) min-risk: 仅 hard rows, 最大化 min margin
        eta_h, u_minrisk = self.compute_eta_h(rows)
        if u_minrisk is None:
            u_minrisk = np.array([
                float(np.clip(u_nom[0], self.v_safe_min, self.v_safe_max)),
                float(np.clip(u_nom[1], -self.w_safe_max, self.w_safe_max)),
            ])
            eta_h = -float("inf")

        self._prev_u = u_minrisk.copy()

        # 3.5) Robust motion recovery — if QP failed AND command can be cancelled
        if self.can_be_fully_cancelled(u_minrisk):
            rec_u, rec_status, rec_info = self.robust_motion_recovery(
                u_nom, A_hard, b_hard, mode
            )
            if rec_u is not None:
                return FilterResult(
                    status="robust_recovery",
                    u_safe=rec_u,
                    eta_h=eta_h if not math.isinf(eta_h) else -999.0,
                    eta_s=eta_h if not math.isinf(eta_h) else -999.0,
                    num_constraints=n_rows,
                    min_clearance=min_h,
                    qp_feasible=False,
                    soft_slack=0.0,
                )

        return FilterResult(
            status="min_risk",
            u_safe=u_minrisk,
            eta_h=eta_h if not math.isinf(eta_h) else -999.0,
            eta_s=eta_h if not math.isinf(eta_h) else -999.0,
            num_constraints=n_rows,
            min_clearance=min_h,
            qp_feasible=False,
            soft_slack=0.0,
        )
