#!/usr/bin/env python3
"""AUDIT16.1 FINAL: Final Robust Safety Filter — runtime authoritative module.

这是论文最终在 ROS2 运行时使用的唯一核心实现.
单位测试 / ROS runtime 都必须 import 本模块 (同一份代码).

内容:
  - final_filter(): Final hard-safety QP (U1 + hard robust CBF rows only)
  - revalidation: publish 前 psi_i(u_pub) >= -tol
  - true safety infeasibility: 独立 LP over U1
  - MIN_RISK: max eta s.t. psi_i>=eta, u∈U1 (safe_guarantee=false)
  - world-lock perception helpers: q_W = T_WL(t_s) q_L (measurement-time pose)
  - status 常量: SAFE_NOMINAL/SAFE_FILTERED/SAFETY_INFEASIBLE/MIN_RISK/
    DEGRADED_SENSOR/SOLVER_INVALID/OTHER
"""
import math
import numpy as np
from typing import Optional, Tuple, List, Dict

try:
    from scipy.optimize import minimize, Bounds, LinearConstraint, linprog
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False

# ── Final 状态码 (冻结) ───────────────────────────────
SAFE_NOMINAL      = "SAFE_NOMINAL"
SAFE_FILTERED     = "SAFE_FILTERED"
TASK_RELAXED      = "TASK_RELAXED"        # 保留枚举 (Final 不触发: task 在 objective)
SAFETY_INFEASIBLE = "SAFETY_INFEASIBLE"
MIN_RISK          = "MIN_RISK"
DEGRADED_SENSOR   = "DEGRADED_SENSOR"
SOLVER_INVALID    = "SOLVER_INVALID"
OTHER             = "OTHER"

# ── Final 诊断维度 (AUDIT16.1 收尾: 无歧义 MIN_RISK 语义) ──
FEASIBILITY_FEASIBLE     = "FEASIBLE"
FEASIBILITY_INFEASIBLE   = "SAFETY_INFEASIBLE"
CTRL_NOMINAL             = "NOMINAL"
CTRL_FILTERED            = "FILTERED"
CTRL_MIN_RISK            = "MIN_RISK"
CTRL_DEGRADED_STOP       = "DEGRADED_STOP"
CTRL_SOLVER_INVALID      = "SOLVER_INVALID"

# Final attack bound (冻结)
AV_BAR = 0.132
AW_BAR = 0.60
ALPHA_FINAL = 1.5
U1_V_MIN, U1_V_MAX = -0.40, 0.40
U1_W_MAX = 1.80
H_W_V, H_W_W = 20.0, 0.8


def row_upper_form(rows):
    """robust rows -> A_ub u <= b_ub (Final upper form).
    每行: psi_i(u) = ell_v v + ell_w w + alpha h - |ell_v|*0.132 - |ell_w|*0.60 >= 0
    <=>  -ell_v v - ell_w w <= alpha h - |ell_v|*0.132 - |ell_w|*0.60
    而 barrier_builder 已把 rhs = alpha*h - |ell_v|*av - |ell_w|*aw 存入 row.rhs.
    因此 A_ub 行 = [-a_v, -a_w], b_ub = row.rhs.  (方法 A: 直接使用 robustified row)
    """
    A = np.array([[-r.a_v, -r.a_w] for r in rows], dtype=float)
    b = np.array([r.rhs for r in rows], dtype=float)
    return A, b


def psi_from_row(u, row):
    """robust margin 由 row 直接计算 (不 double count): psi = b - A u."""
    return row.rhs - (-row.a_v * u[0] - row.a_w * u[1])


def _objective(u, u_nav, H):
    d = u - u_nav
    return 0.5 * float(d @ H @ d)


def _solve_qp(u_nav, H, A, b, lb, ub):
    """min 0.5(u-u_nav)^T H (u-u_nav) s.t. lb<=u<=ub, A u<=b."""
    x0 = np.array([float(np.clip(u_nav[0], lb[0], ub[0])),
                   float(np.clip(u_nav[1], lb[1], ub[1]))])
    if _HAS_SCIPY:
        res = minimize(lambda x: _objective(x, u_nav, H), x0=x0,
                       method="SLSQP", bounds=Bounds(lb, ub),
                       constraints=[LinearConstraint(A, -np.inf * np.ones(len(b)), b)],
                       options={"maxiter": 300, "ftol": 1e-10, "disp": False})
        if res.success and np.max(A @ res.x - b) < 1e-6:
            return res.x, True
        return None, False
    best_u, best_c = None, float("inf")
    for v in np.linspace(lb[0], ub[0], 61):
        for w in np.linspace(lb[1], ub[1], 121):
            u = np.array([v, w])
            if np.max(A @ u - b) <= 1e-8:
                c = _objective(u, u_nav, H)
                if c < best_c:
                    best_c, best_u = c, u
    return (best_u, True) if best_u is not None else (None, False)


def _feasibility_lp(A, b, lb, ub):
    """独立 LP: 存在 u∈[lb,ub] 使 A u <= b?"""
    if _HAS_SCIPY:
        res = linprog(np.zeros(A.shape[1]), A_ub=A, b_ub=b,
                      bounds=list(zip(lb, ub)), method="highs")
        return bool(res.success)
    for v in np.linspace(lb[0], ub[0], 41):
        for w in np.linspace(lb[1], ub[1], 81):
            u = np.array([v, w])
            if np.max(A @ u - b) <= 1e-8:
                return True
    return False


def _min_risk(A, b, lb, ub, navtol=1e-9):
    """max eta s.t. A u + eta <= b (i.e. psi_i>=eta), u∈U1.
    返回 (u*, eta*)."""
    best_u, best_eta = None, -float("inf")
    for v in np.linspace(lb[0], ub[0], 81):
        for w in np.linspace(lb[1], ub[1], 161):
            u = np.array([v, w])
            m = float(np.min(b - A @ u))
            if m > best_eta:
                best_eta, best_u = m, u
    return best_u, best_eta


def _diagnostic_info(feasible, control_action, safe_guarantee,
                     min_hard_margin, worst_margin, num_rows, safety_infeasible=None):
    """Final 诊断信息构造 (AUDIT16.1 收尾).

    feasibility_status: FEASIBLE | SAFETY_INFEASIBLE (U1 ∩ hard rows 是否空)
    control_action:     NOMINAL | FILTERED | MIN_RISK | SOLVER_INVALID
                        (节点 DEGRADED 分支另行使用 CTRL_DEGRADED_STOP)
    safe_guarantee:     本周期发布的 u_sf 是否满足 robust CBF 保证
    safety_infeasible:  U_SAFE=empty 且实际执行 min-risk
    """
    if safety_infeasible is None:
        safety_infeasible = (not feasible) and (control_action == CTRL_MIN_RISK)
    return {
        "feasible": feasible,
        "feasibility_status": FEASIBILITY_FEASIBLE if feasible else FEASIBILITY_INFEASIBLE,
        "control_action": control_action,
        "safe_guarantee": bool(safe_guarantee),
        "safety_infeasible": bool(safety_infeasible),
        "min_hard_margin": float(min_hard_margin),
        "worst_margin": float(worst_margin),
        "num_rows": num_rows,
    }


def final_filter(core, u_nav, rows, attack_bounds=(AV_BAR, AW_BAR),
                 H=None, tol=1e-6):
    """Final robust safety filter 主入口 (runtime authoritative).

    Returns: (u_sf, status, info)
      info: {feasible, feasibility_status, control_action, safe_guarantee,
             safety_infeasible, min_hard_margin, worst_margin, num_rows}
    """
    av_bar, aw_bar = attack_bounds
    w_v = H[0][0] if H is not None else getattr(core, "w_v", H_W_V)
    w_w = H[1][1] if H is not None else getattr(core, "w_w", H_W_W)
    Hmat = np.diag([float(w_v), float(w_w)])
    u_nav = np.asarray(u_nav, dtype=float)
    lb = np.array([getattr(core, "v_safe_min", U1_V_MIN),
                   -getattr(core, "w_safe_max", U1_W_MAX)], dtype=float)
    ub = np.array([getattr(core, "v_safe_max", U1_V_MAX),
                   getattr(core, "w_safe_max", U1_W_MAX)], dtype=float)

    if len(rows) == 0:
        u_clip = np.clip(u_nav, lb, ub)
        status = SAFE_NOMINAL if np.allclose(u_clip, u_nav, atol=tol) else SAFE_FILTERED
        ctrl = CTRL_NOMINAL if status == SAFE_NOMINAL else CTRL_FILTERED
        return u_clip, status, _diagnostic_info(True, ctrl, True,
                                                float("inf"), float("inf"), 0)

    A, b = row_upper_form(rows)

    # LEVEL 1: true safety feasibility over U1
    feasible = _feasibility_lp(A, b, lb, ub)
    if not feasible:
        u_mr, eta = _min_risk(A, b, lb, ub)
        # U_SAFE=empty → 实际发布 min-risk; status 明确 MIN_RISK (不再仅 SAFETY_INFEASIBLE)
        return u_mr, MIN_RISK, _diagnostic_info(False, CTRL_MIN_RISK, False,
                                                eta, eta, len(rows), safety_infeasible=True)

    # LEVEL 2: QP within safe set
    u_opt, ok = _solve_qp(u_nav, Hmat, A, b, lb, ub)
    if not ok:
        u_mr, eta = _min_risk(A, b, lb, ub)
        # LP 可行但 QP solver 失败 → SOLVER_INVALID (安全保证丢失, 但非 U_SAFE=empty)
        return u_mr, SOLVER_INVALID, _diagnostic_info(True, CTRL_SOLVER_INVALID, False,
                                                      eta, eta, len(rows),
                                                      safety_infeasible=False)

    # Revalidation (publish 前): all hard rows
    margins = b - A @ u_opt
    min_margin = float(np.min(margins))
    if min_margin < -tol:
        # 尝试一次 resolve
        u_opt2, ok2 = _solve_qp(u_nav, Hmat, A, b, lb, ub)
        if ok2:
            margins2 = b - A @ u_opt2
            min_m2 = float(np.min(margins2))
            if min_m2 >= -tol:
                u_opt, min_margin = u_opt2, min_m2
            else:
                u_mr, eta = _min_risk(A, b, lb, ub)
                return u_mr, SOLVER_INVALID, _diagnostic_info(True, CTRL_SOLVER_INVALID,
                                                              False, eta, eta, len(rows),
                                                              safety_infeasible=False)
        else:
            u_mr, eta = _min_risk(A, b, lb, ub)
            return u_mr, SOLVER_INVALID, _diagnostic_info(True, CTRL_SOLVER_INVALID,
                                                          False, eta, eta, len(rows),
                                                          safety_infeasible=False)

    status = SAFE_NOMINAL if np.allclose(u_opt, u_nav, atol=tol) else SAFE_FILTERED
    ctrl = CTRL_NOMINAL if status == SAFE_NOMINAL else CTRL_FILTERED
    return u_opt, status, _diagnostic_info(True, ctrl, True,
                                           min_margin, min_margin, len(rows))


# ── world-lock perception (纯函数, 可离线测试) ────────────
def scan_points_world_at_ts(ranges, angles, range_max, T_WL):
    """scan.frame 系点 (r, phi) 经 T_WL (world->base_scan at scan.header.stamp)
    转 world 并缓存.  T_WL 可为:
      - 4x4 matrix, 或
      - (R, t) 元组, 或
      - state [x, y, theta] (odom pose at scan time; sim 中 world==odom, 且
        base_scan extrinsic 近似为 odom 原点, 因 URDF 已含 base_scan 位姿,
        此处用 odom pose 做 world-lock 旋转平移)
    返回 q_W: (N,2) world 点 (measurement-time world-lock)."""
    valid = (ranges > 0.01) & (ranges < range_max - 1e-3) & np.isfinite(ranges)
    r = np.asarray(ranges, float)[valid]
    a = np.asarray(angles, float)[valid]
    q_L = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)  # base_scan frame

    if isinstance(T_WL, np.ndarray) and T_WL.shape == (4, 4):
        R = T_WL[:2, :2]; t = T_WL[:2, 3]
    elif isinstance(T_WL, (tuple, list)) and len(T_WL) == 2:
        R, t = T_WL[0], T_WL[1]
    else:
        # state [x, y, theta] at scan time
        sx, sy, st = float(T_WL[0]), float(T_WL[1]), float(T_WL[2])
        R = np.array([[math.cos(st), -math.sin(st)],
                      [math.sin(st), math.cos(st)]])
        t = np.array([sx, sy])
    q_W = (R @ q_L.T).T + t
    return q_W, valid


def barrier_rows_from_worldpoints(state, q_W, body_disks, alpha_h,
                                  av_bar, aw_bar, margins):
    """在 control time t_k 用当前 state 计算 barrier (world-locked q_W).
    h_ij(x_k) = ||p_j(x_k) - q_W|| - R_s ; ell 用当前 state 速度系数.
    q_W 是 scan-time world-locked 的障碍点 (测量时锁定, 不再随机器人移动)."""
    from safety_filter.geometry_utils import body_disk_velocity_coeffs
    from safety_filter.barrier_builder import CBFConstraintRow
    rows = []
    min_h = float("inf")
    theta = float(state[2])
    R_theta = np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)]])
    p = np.array(state[:2])
    for disk_idx, (cx, cy, rho_j) in enumerate(body_disks):
        p_j = p + R_theta @ np.array([cx, cy])
        g_v, g_w = body_disk_velocity_coeffs(state, body_disks, disk_idx)
        for q in q_W:
            d = math.hypot(p_j[0]-q[0], p_j[1]-q[1])
            if d < 1e-9:
                unit = np.array([1.0, 0.0])
            else:
                unit = (p_j - q) / d
            h_ij = d - (rho_j + margins["delta"]) - margins.get("point_radius", 0.0)
            min_h = min(min_h, h_ij)
            ell_v = float(np.dot(unit, g_v))
            ell_w = float(np.dot(unit, g_w))
            rhs = alpha_h * h_ij - abs(ell_v) * av_bar - abs(ell_w) * aw_bar
            rows.append(CBFConstraintRow(a_v=ell_v, a_w=ell_w, rhs=rhs, h_value=h_ij,
                                         source_idx=0, disk_idx=disk_idx,
                                         primitive_type="world_locked",
                                         primitive_idx=0))
    return rows, min_h
