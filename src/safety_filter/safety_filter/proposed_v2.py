#!/usr/bin/env python3
"""Proposed V2 — three-tier safety filter  STRICT -> SOFT -> MIN_RISK.

NEW, ISOLATED MODULE.  Nothing in the existing platform is modified or
overwritten in place.  The formal 1300-run campaign used
``final_robust.final_filter`` (feasibility LP -> safety-projection QP ->
min-risk fallback, i.e. TWO levels plus a fallback).  This module is the
strictly-three-tier counterpart requested by the paper.

--------------------------------------------------------------------------
Mathematical definition implemented here
--------------------------------------------------------------------------
Let  U_sf = { v in [v_safe_min, v_safe_max],  w in [-w_safe_max, w_safe_max] }.
Let the robust CBF rows produced by ``barrier_rows_from_worldpoints`` be

    psi_i(u) = a_v_i * v + a_w_i * w + rhs_i  >=  0
             with  rhs_i = alpha_h*h_i - |a_v_i|*av_hat - |a_w_i|*aw_hat

so, stacking,  A_s u >= b_s  with  A_s = [[a_v_i, a_w_i]] ,  b_s = -rhs_i.

The task-compatibility row is the single, symmetric NOT_REVERSE row.  It is
built from the REAL nominal command u_nav, NOT from goal geometry:

    u_nav.v >= 0  ->  A_t = [+1, 0],  b_t = 0      (v_sf >= 0)
    u_nav.v <  0  ->  A_t = [-1, 0],  b_t = 0      (v_sf <= 0)

Semantics: "the filter may decelerate or stop the robot, but must not reverse
the direction of travel that the nominal planner asked for".  In the frozen
experiment configuration Nav2/DWA runs with min_vel_x = 0, so u_nav.v >= 0
always and the row reduces to v_sf >= 0.

The row is ALWAYS FEASIBLE (v = 0 satisfies it), imposes NO lower speed bound
(it never forces a positive v) and does NOT couple v to w.  Alternative rows
were measured on the real state traces and rejected:
  * goal-heading row: suppressed mean |w| by 44% on healthy trajectories
    (mean |Dw| = 0.064 rad/s) -> fights the Nav2 local path.
  * goal-progress row: demands 0.22 m/s even where DWA deliberately slows
    down -> +37% overspeed on healthy trajectories (mean |Dv| = 0.047).
  * cos(alpha)*v >= 0: flips to v <= 0 for |alpha| > 90 deg, so it commands
    reverse whenever the goal happens to be behind the robot.
  * sign(alpha)*w >= 0: same |Dw| = 0.064 conflict as the heading row.

STRICT   : min_u   1/2 (u-u_nav)^T H (u-u_nav)
           s.t.    A_s u >= b_s ,  A_t u >= b_t ,  u in U_sf
           (A_t is omitted -> STRICT is the safety-only QP; see below)

SOFT     : min_{u,d} 1/2 (u-u_nav)^T H (u-u_nav) + (lambda_delta/2) * ||d||^2
           s.t.    A_s u >= b_s              <-- HARD, never relaxed
                   A_t u + d >= b_t
                   d >= 0
                   u in U_sf

MIN_RISK : TWO-STAGE LEXICOGRAPHIC (no weighted compromise)
     stage 1:  eta* = max_{u,eta} eta   s.t.  A_s u - b_s >= eta * 1 ,  u in U_sf
               exact LP; eta is FREE IN SIGN and is typically negative here
     stage 2:  min_u  1/2 (u-u_nav)^T H (u-u_nav)
               s.t.   A_s u - b_s >= eta* - tau ,  u in U_sf
               tau is a NUMERIC feasibility tolerance only (see MINRISK_TAU)
The safety residual is lexicographically strictly dominant: stage 2 may never
spend the stage-1 optimum to improve nominal tracking.

Stage order, with the exact entry conditions:
  STRICT   : hard robust-CBF safety rows + hard NOT_REVERSE row.  When no task
             row is supplied this is simply the safety-only QP (that path is
             now actually exercised; the previous code skipped it and jumped
             straight to MIN_RISK).  If the SLSQP objective solve fails but an
             exact LP proves the hard set is feasible, the solve is retried
             from the LP point, and if that also fails the LP point itself is
             used -- flagged, never silent.
  SOFT     : entered ONLY when STRICT is genuinely infeasible AND the robust
             safety rows are themselves feasible inside U_sf.  Safety rows stay
             HARD; only the task row receives a non-negative slack delta.
  MIN_RISK : entered ONLY when the full robust safety set is infeasible inside
             U_sf.  It can never be reached because of numerical noise alone.

Anomalies (exceptions, solver failures, LP fallbacks, warm-start retries) are
recorded in info["anomalies"] and are never hidden by bypassing the filter.
MIN_RISK never returns zero / nominal / previous command as a substitute.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    from scipy.optimize import minimize, Bounds, LinearConstraint, linprog
    _HAS_SCIPY = True
except Exception:  # pragma: no cover
    _HAS_SCIPY = False

# ── stage labels (published to /filter_status) ──────────────────────────
V2_STRICT = "V2_STRICT"
V2_SOFT = "V2_SOFT"
V2_MIN_RISK = "V2_MIN_RISK"
V2_DEGRADED = "V2_DEGRADED"
V2_NO_ROWS = "V2_NO_ROWS"      # no active barrier row: saturate nominal to U_sf

# ── solver tolerances (chosen to mirror the platform's SLSQP settings) ──
QP_MAXITER = 300
QP_FTOL = 1e-10
CONSTRAINT_TOL = 1e-6

# MIN_RISK stage 2 is a *numeric feasibility* band, never a performance knob.
# The lexicographic pair is
#     stage 1:  eta* = max_{u,eta} eta            (exact LP, HiGHS)
#     stage 2:  min_u (u-u_nav)^T H (u-u_nav)  s.t.  A_s u - b_s >= eta* - tau
# eta* is exact, so the only slack is the QP feasibility band.  Raising tau to
# buy better nominal tracking would silently break the lexicographic guarantee.
#
# Calibration (929 random infeasible-safety row sets): stage 2 always lands on
# the relaxed face, so the barrier residual actually spent equals tau:
#     tau = 0      -> loss 4.7e-15,  8/929 stage-2 numerical failures
#     tau = 1e-12  -> loss 2.1e-12,  0/929
#     tau = 1e-9   -> loss 1.0e-9,   1/929     <-- default
#     tau = 1e-6   -> loss 1.0e-6,   0/929
# tau = 1e-9 corresponds to 6.7e-10 m of barrier margin (3e-8 of the 0.02 m
# geometry margin), i.e. a purely numerical quantity.  A stage-2 solver failure
# is never fatal: the stage-1 LP point already satisfies stage 2 and is used
# instead, with the anomaly flagged.
MINRISK_TAU = 1e-9


# ─────────────────────────────────────────────────────────────────────
# helpers
# ─────────────────────────────────────────────────────────────────────
def to_upper(A_ge: np.ndarray, b_ge: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """A u >= b  ->  (-A) u <= (-b)."""
    return -np.asarray(A_ge, float), -np.asarray(b_ge, float)


def rows_to_As_bs(rows: Sequence) -> Tuple[np.ndarray, np.ndarray]:
    """Robust CBF rows -> (A_s, b_s) with the  A_s u >= b_s  convention.

    psi_i(u) = a_v_i v + a_w_i w + rhs_i >= 0   <=>   a_v_i v + a_w_i w >= -rhs_i
    """
    if len(rows) == 0:
        return np.zeros((0, 2)), np.zeros((0,))
    A = np.array([[r.a_v, r.a_w] for r in rows], dtype=float)
    b = np.array([-r.rhs for r in rows], dtype=float)
    return A, b


# ── task-compatibility row: NOT_REVERSE ────────────────────────────────
# The single task row used by Proposed V2.  It anchors on the REAL nominal
# command u_nav (which is what Nav2/DWA actually decided to do this cycle),
# not on goal geometry, so it cannot invent a task the planner did not ask for.
# The legacy goal-heading / goal-progress rows were removed: they were measured
# to fight the Nav2 local path (see the module docstring).
def not_reverse_row(u_nav_v: float) -> Tuple[np.ndarray, float]:
    """Protect the nominal direction of travel.

        u_nav_v >= 0 ->  A_t = [+1, 0], b_t = 0   <=>   v_sf >= 0
        u_nav_v <  0 ->  A_t = [-1, 0], b_t = 0   <=>   v_sf <= 0

    Always feasible (v = 0), no lower speed bound, no v/w coupling.
    """
    if float(u_nav_v) >= 0.0:
        return np.array([1.0, 0.0], float), 0.0
    return np.array([-1.0, 0.0], float), 0.0


# ── constraint verification (independent of the solver's own report) ────
def verify(u: np.ndarray, A_ge=None, b_ge=None) -> Dict:
    out = {"finite": bool(np.all(np.isfinite(u))),
           "hard_min_margin": None, "hard_max_violation": None}
    if A_ge is not None and len(A_ge) > 0:
        m = np.asarray(A_ge, float) @ np.asarray(u, float) - np.asarray(b_ge, float)
        out["hard_min_margin"] = float(np.min(m))
        out["hard_max_violation"] = float(max(0.0, -float(np.min(m))))
    return out


def _lp_feasible_point(A_ge, b_ge, lb, ub):
    """Exact LP feasibility test:  find u  s.t.  A_ge u >= b_ge ,  u in [lb, ub].

    Returns (u_or_None, feasible: bool, message).  Uses HiGHS with a zero
    objective, i.e. it answers the feasibility question exactly rather than
    relying on a nonlinear solver's failure message.
    """
    A_ge = np.asarray(A_ge, float).reshape(-1, 2)
    b_ge = np.asarray(b_ge, float).reshape(-1)
    if len(b_ge) == 0:
        return np.clip((np.asarray(lb, float) + np.asarray(ub, float)) / 2.0, lb, ub), True, "no_constraints"
    if not _HAS_SCIPY:
        return None, False, "no_scipy"
    res = linprog(np.zeros(2), A_ub=-A_ge, b_ub=-b_ge,
                  bounds=list(zip(np.asarray(lb, float), np.asarray(ub, float))),
                  method="highs")
    if res.status == 2:
        return None, False, "infeasible"
    if res.x is None:
        return None, False, f"lp_status_{res.status}"
    u = np.asarray(res.x, float)
    if not np.all(np.isfinite(u)):
        return None, False, f"lp_nonfinite_status_{res.status}"
    m = float(np.min(A_ge @ u - b_ge))
    if m < -CONSTRAINT_TOL:
        return None, False, f"lp_residual_{m:+.3e}"
    return u, True, "ok"


# ─────────────────────────────────────────────────────────────────────
# stage solvers
# ─────────────────────────────────────────────────────────────────────
def _slsqp(x0, obj, A_ub, b_ub, lb, ub):
    if not _HAS_SCIPY:
        return None, False, "no_scipy"
    cons = [LinearConstraint(A_ub, -np.inf * np.ones(len(b_ub)), b_ub)] \
        if len(b_ub) else []
    res = minimize(obj, x0=x0, method="SLSQP", bounds=Bounds(lb, ub),
                   constraints=cons,
                   options={"maxiter": QP_MAXITER, "ftol": QP_FTOL, "disp": False})
    if not res.success or not np.all(np.isfinite(res.x)):
        return None, False, f"slsqp_fail:{getattr(res,'message','')[:60]}"
    return np.asarray(res.x, float), True, "ok"


def solve_strict(u_nav, H, A_s, b_s, A_t, b_t, lb, ub, x0=None):
    """Level 1.  Returns (u, ok, info).

    ``A_t``/``b_t`` may be empty: STRICT then degenerates to the safety-only QP
    (this is the path taken when no task row is defined).
    """
    A_ge = np.vstack([A_s, A_t])
    b_ge = np.concatenate([b_s, b_t])
    A_ub, b_ub = to_upper(A_ge, b_ge)
    if not _HAS_SCIPY:
        return None, False, {"note": "no scipy"}
    Hm = np.asarray(H, float)

    def obj(x):
        d = x - u_nav
        return 0.5 * float(d @ Hm @ d)

    start = np.clip(u_nav, lb, ub) if x0 is None else np.clip(np.asarray(x0, float), lb, ub)
    x, ok, msg = _slsqp(start, obj, A_ub, b_ub, lb, ub)
    if not ok:
        return None, False, {"solver": msg,
                             "warm_start": "lp_point" if x0 is not None else "nominal",
                             "n_hard_rows": int(len(b_ge))}
    v = verify(x, A_ge, b_ge)
    if not v["finite"]:
        return None, False, {"solver": msg, "reason": "non_finite"}
    if v["hard_min_margin"] < -CONSTRAINT_TOL:
        return None, False, {"solver": msg, "reason": "residual_violation",
                             "hard_min_margin": v["hard_min_margin"]}
    return x, True, {"solver": msg, "objective": obj(x),
                     "n_hard_rows": int(len(b_ge)), **v}


def solve_soft(u_nav, H, A_s, b_s, A_t, b_t, lb, ub, lam):
    """Level 2.  Safety rows stay HARD; only the task row is relaxed by delta >= 0."""
    ns = len(b_s)
    # hard safety rows, delta column fixed to 0
    A_ub_s = np.hstack([-np.asarray(A_s, float), np.zeros((ns, 1))])
    b_ub_s = -np.asarray(b_s, float)
    # relaxed task row:  A_t u + delta >= b_t   <=>   -A_t u - delta <= -b_t
    A_ub_t = np.hstack([-np.asarray(A_t, float), -np.ones((1, 1))])
    b_ub_t = -np.asarray(b_t, float)
    A_ub = np.vstack([A_ub_s, A_ub_t])
    b_ub = np.concatenate([b_ub_s, b_ub_t])

    lb3 = np.concatenate([lb, [0.0]])
    ub3 = np.concatenate([ub, [1e6]])
    Hm = np.asarray(H, float)

    def obj(x):
        d = x[:2] - u_nav
        return 0.5 * float(d @ Hm @ d) + 0.5 * lam * float(x[2]) ** 2

    x0 = np.concatenate([np.clip(u_nav, lb, ub), [0.0]])
    x, ok, msg = _slsqp(x0, obj, A_ub, b_ub, lb3, ub3)
    if not ok:
        return None, False, {"solver": msg}
    u, d = x[:2], float(x[2])
    if d < -CONSTRAINT_TOL:
        return None, False, {"solver": msg, "reason": "delta_negative"}
    v = verify(u, A_s, b_s)                     # safety verified at u, not (u,d)
    if not v["finite"] or v["hard_min_margin"] < -CONSTRAINT_TOL:
        return None, False, {"solver": msg, "reason": "hard_safety_violated",
                             "hard_min_margin": v["hard_min_margin"]}
    return u, True, {"solver": msg, "objective": obj(x), "delta": d,
                     "task_slack_used": d > 1e-9, **v}


def solve_minrisk(u_nav, H, A_s, b_s, lb, ub, tau=MINRISK_TAU):
    """Level 3, solved as a STRICT two-stage lexicographic program.

    stage 1 (exact LP, HiGHS):  eta* = max_{u,eta} eta
                                s.t.  A_s u - b_s >= eta ,  u in U_sf
    stage 2 (SLSQP QP):         min_u 1/2 (u-u_nav)^T H (u-u_nav)
                                s.t.  A_s u - b_s >= eta* - tau ,  u in U_sf

    The stage-2 feasible set always contains the stage-1 LP point (its residual
    is >= eta* >= eta* - tau), so the lexicographic guarantee cannot be lost to
    a numerical failure -- the solve degrades to the LP point and is flagged.

    Returns (u, eta_achieved, ok, info); ``eta_achieved`` is the true worst
    robust residual of the returned control, recomputed independently.
    """
    t_start = time.perf_counter()
    info = {"tau": float(tau), "method": "lexicographic_2stage"}
    A_s = np.asarray(A_s, float)
    b_s = np.asarray(b_s, float)

    if len(b_s) == 0:
        u = np.clip(np.asarray(u_nav, float), lb, ub)
        info.update(eta_star=None, eta_achieved=None, note="no_safety_rows",
                    total_ms=(time.perf_counter() - t_start) * 1e3)
        return u, float("inf"), True, info
    if not _HAS_SCIPY:
        info.update(reason="no_scipy",
                    total_ms=(time.perf_counter() - t_start) * 1e3)
        return None, None, False, info

    # ---- stage 1: exact maximum of the worst robust residual ----
    t1 = time.perf_counter()
    u1, eta_star, msg1 = _minrisk_lp(A_s, b_s, lb, ub)
    info["stage1"] = {"solver": "linprog/highs", "eta_star": eta_star,
                      "message": msg1, "ms": (time.perf_counter() - t1) * 1e3}
    if u1 is None or eta_star is None or not np.isfinite(eta_star):
        info.update(reason="stage1_lp_failed",
                    total_ms=(time.perf_counter() - t_start) * 1e3)
        return None, None, False, info
    info["eta_star"] = float(eta_star)

    # ---- stage 2: H-weighted nominal projection on the eta*-optimal face ----
    b_req = b_s + (float(eta_star) - float(tau))
    A_ub, b_ub = to_upper(A_s, b_req)
    Hm = np.asarray(H, float)

    def obj(x):
        d = x - u_nav
        return 0.5 * float(d @ Hm @ d)

    t2 = time.perf_counter()
    x, ok, msg2 = _slsqp(np.clip(u1, lb, ub), obj, A_ub, b_ub, lb, ub)
    info["stage2"] = {"solver": msg2, "ms": (time.perf_counter() - t2) * 1e3}
    if not ok or not np.all(np.isfinite(x)):
        # Stage 1's LP point already satisfies stage 2 by construction, so the
        # lexicographic optimum is preserved; only the nominal projection is
        # sub-optimal.  Flagged, never silent.
        achieved = float(np.min(A_s @ u1 - b_s))
        info.setdefault("anomalies", []).append(
            "stage2_qp_failed__using_stage1_lp_point")
        info.update(eta_achieved=achieved,
                    eta_loss=float(eta_star) - achieved,
                    total_ms=(time.perf_counter() - t_start) * 1e3)
        return u1, achieved, True, info

    achieved = float(np.min(A_s @ x - b_s))
    info.update(eta_achieved=achieved, eta_loss=float(eta_star) - achieved,
                objective=obj(x),
                total_ms=(time.perf_counter() - t_start) * 1e3)
    return x, achieved, True, info


def _minrisk_lp(A_s, b_s, lb, ub):
    """Exact LP  max_{u,eta} eta  s.t.  A_s u - b_s >= eta ,  u in U_sf.

    This IS MIN_RISK stage 1 -- not merely a diagnostic reference.  It returns
    the lexicographic optimum ``eta*`` exactly, which stage 2 must then respect.
    """
    if not _HAS_SCIPY or len(b_s) == 0:
        return None, None, None
    ns = len(b_s)
    A = np.hstack([-np.asarray(A_s, float), np.ones((ns, 1))])
    b = -np.asarray(b_s, float)
    res = linprog([0.0, 0.0, -1.0], A_ub=A, b_ub=b,
                  bounds=list(zip(np.concatenate([lb, [-1e6]]),
                                  np.concatenate([ub, [1e6]]))),
                  method="highs")
    if not res.success:
        return None, None, res.message
    return np.asarray(res.x[:2], float), float(res.x[2]), "ok"


# ─────────────────────────────────────────────────────────────────────
# the three-tier cascade
# ─────────────────────────────────────────────────────────────────────
def three_tier(u_nav: np.ndarray, rows: Sequence, *,
               H: np.ndarray, lb: np.ndarray, ub: np.ndarray,
               task: Optional[Tuple[np.ndarray, float]] = None,
               lam: float = 10.0, tau: float = MINRISK_TAU
               ) -> Tuple[np.ndarray, str, Dict]:
    """Run STRICT -> SOFT -> MIN_RISK and return (u, stage, info).

    Entry conditions are enforced exactly:
      * STRICT   : hard safety rows (+ hard task row when one is supplied).
      * SOFT     : only when STRICT is genuinely infeasible AND the safety rows
                   on their own are still feasible inside U_sf.
      * MIN_RISK : only when the robust safety set is infeasible inside U_sf.
                   Solved lexicographically: the worst robust residual is
                   maximised first (exact LP), then the H-weighted nominal
                   deviation is minimised within ``tau`` of that optimum.
    """
    u_nav = np.asarray(u_nav, float)
    info: Dict = {"u_nav": u_nav.copy(), "lambda_delta": float(lam),
                  "minrisk_tau": float(tau), "n_safety_rows": int(len(rows)),
                  "task_row_defined": bool(task is not None),
                  "anomalies": []}

    # No active barrier row -> nothing to enforce; saturate nominal to U_sf.
    # This is NOT one of the three tiers; it is the empty-constraint guard.
    if len(rows) == 0:
        u = np.clip(u_nav, lb, ub)
        info.update(reason="no_active_rows", stage_reason="no_active_rows",
                    bounds_violation=_bounds_violation(u, lb, ub))
        return u, V2_NO_ROWS, info

    A_s, b_s = rows_to_As_bs(rows)
    info["safety_rows"] = {"A_s": A_s.tolist(), "b_s": b_s.tolist()}

    if task is None:
        A_t = np.zeros((0, 2), float)
        b_t = np.zeros((0,), float)
    else:
        A_t = np.asarray(task[0], float).reshape(1, 2)
        b_t = np.asarray([float(task[1])], float)
        info["task_row"] = {"A_t": A_t.tolist(), "b_t": b_t.tolist()}

    has_task = bool(len(b_t) > 0)
    if has_task:
        A_hard = np.vstack([A_s, A_t])
        b_hard = np.concatenate([b_s, b_t])
    else:
        A_hard, b_hard = np.asarray(A_s, float), np.asarray(b_s, float)

    def _task_margin(uu):
        """Margin of the task row at ``uu`` (None when no task row exists).

        Recorded so that an observed velocity modification can be ATTRIBUTED to
        the task row or to the safety rows instead of being asserted.
        """
        if not has_task:
            return None
        return float(np.min(np.asarray(A_t, float) @ np.asarray(uu, float) - b_t))

    # ---------- Level 1: STRICT ----------
    try:
        u, ok, si = solve_strict(u_nav, H, A_s, b_s, A_t, b_t, lb, ub)
    except Exception as exc:                                    # noqa: BLE001
        info["anomalies"].append(f"strict_exception:{exc!r}")
        u, ok, si = None, False, {"exception": repr(exc)}
    info["strict"] = si
    if ok:
        info["stage_reason"] = ("strict_feasible" if has_task
                                else "strict_feasible_safety_only")
        info["hard_min_margin"] = si.get("hard_min_margin")
        info["task_row_margin"] = _task_margin(u)
        info.update(bounds_violation=_bounds_violation(u, lb, ub))
        return u, V2_STRICT, info

    # STRICT did not return a usable point.  Separate "the solver failed" from
    # "the hard set is genuinely empty" with an exact LP feasibility test.
    u_lp, hard_feas, lp_msg = _lp_feasible_point(A_hard, b_hard, lb, ub)
    info["strict_lp_feasibility"] = {"feasible": bool(hard_feas), "message": lp_msg}
    if hard_feas and u_lp is not None:
        # The hard set IS feasible, so the failure was numerical.  Retry once
        # from the LP point; if that also fails, use the LP point itself -- a
        # genuine STRICT-feasible control -- explicitly flagged as an anomaly.
        info["anomalies"].append("strict_solver_failed_but_lp_feasible")
        u_r, ok_r, si_r = None, False, {}
        try:
            u_r, ok_r, si_r = solve_strict(u_nav, H, A_s, b_s, A_t, b_t, lb, ub,
                                           x0=u_lp)
        except Exception as exc:                                # noqa: BLE001
            info["anomalies"].append(f"strict_retry_exception:{exc!r}")
        if ok_r:
            info["strict"] = si_r
            info["stage_reason"] = "strict_feasible_after_lp_warm_start"
            info["hard_min_margin"] = si_r.get("hard_min_margin")
            info["task_row_margin"] = _task_margin(u_r)
            info.update(bounds_violation=_bounds_violation(u_r, lb, ub))
            return u_r, V2_STRICT, info
        info["stage_reason"] = "strict_lp_feasible_point_used"
        info["hard_min_margin"] = float(np.min(A_hard @ u_lp - b_hard))
        info["task_row_margin"] = _task_margin(u_lp)
        info.update(bounds_violation=_bounds_violation(u_lp, lb, ub))
        return u_lp, V2_STRICT, info
    info["stage_reason"] = "strict_infeasible"

    # ---------- Level 2: SOFT ----------
    if has_task:
        # SOFT keeps the safety rows HARD, so it can only succeed when the
        # safety rows alone are feasible inside U_sf.
        u_safe, safe_ok = None, True
        if len(b_s):
            u_safe, safe_ok, safe_msg = _lp_feasible_point(A_s, b_s, lb, ub)
            info["safety_only_feasible"] = bool(safe_ok)
            info["safety_only_message"] = safe_msg
        if not safe_ok:
            info["stage_reason"] = "strict_infeasible_and_safety_infeasible"
        else:
            try:
                u, ok, si = solve_soft(u_nav, H, A_s, b_s, A_t, b_t, lb, ub, lam)
            except Exception as exc:                            # noqa: BLE001
                info["anomalies"].append(f"soft_exception:{exc!r}")
                u, ok, si = None, False, {"exception": repr(exc)}
            info["soft"] = si
            if ok:
                info["stage_reason"] = "strict_infeasible_soft_feasible"
                info["delta"] = si.get("delta")
                info["hard_min_margin"] = si.get("hard_min_margin")
                info["task_row_margin"] = _task_margin(u)
                info.update(bounds_violation=_bounds_violation(u, lb, ub))
                return u, V2_SOFT, info
            # Numerical failure although the safety set is feasible: use an LP
            # feasible point and report the exact slack it needs.
            if u_safe is not None:
                info["anomalies"].append("soft_solver_failed_but_safety_feasible")
                info["stage_reason"] = "soft_solver_failed_lp_feasible_point"
                info["delta"] = float(max(0.0, float(np.min(b_t - A_t @ u_safe))))
                info["hard_min_margin"] = (float(np.min(A_s @ u_safe - b_s))
                                           if len(b_s) else None)
                info["task_row_margin"] = _task_margin(u_safe)
                info.update(bounds_violation=_bounds_violation(u_safe, lb, ub))
                return u_safe, V2_SOFT, info
    else:
        info["stage_reason"] = "strict_infeasible_no_task_row"

    # ---------- Level 3: MIN_RISK (two-stage lexicographic) ----------
    try:
        u, eta, ok, si = solve_minrisk(u_nav, H, A_s, b_s, lb, ub, tau)
    except Exception as exc:                                    # noqa: BLE001
        info["anomalies"].append(f"minrisk_exception:{exc!r}")
        u, eta, ok, si = None, None, False, {"exception": repr(exc)}
    info["minrisk"] = si
    for _a in list(si.get("anomalies") or []):
        info["anomalies"].append(_a)
    if not ok or u is None:
        # Solver failure --- NOT a mathematical infeasibility.  Fall back to the
        # exact LP max-min reference (still a genuine MIN_RISK solution, not a
        # stop/nominal/previous command).
        u_lp2, eta_lp, msg = _minrisk_lp(A_s, b_s, lb, ub)
        info["minrisk"]["lp_fallback"] = msg
        info["minrisk"]["eta_star"] = eta_lp
        if u_lp2 is None:
            u = np.clip(u_nav, lb, ub)
            info["anomalies"].append("minrisk_solver_and_lp_failed")
            info["stage_reason"] = "minrisk_solver_and_lp_failed__saturate_nominal"
            info.update(bounds_violation=_bounds_violation(u, lb, ub))
            return u, V2_DEGRADED, info
        u, eta = u_lp2, eta_lp
        info["stage_reason"] = "minrisk_lp_fallback"
    else:
        info["stage_reason"] = "safety_infeasible_minrisk"
    info["eta"] = float(eta)                     # achieved worst robust residual
    info["eta_star"] = (float(si["eta_star"]) if si.get("eta_star") is not None
                        else float(eta))          # stage-1 exact LP optimum
    info["hard_min_margin"] = float(eta)
    info["task_row_margin"] = _task_margin(u)
    info.update(bounds_violation=_bounds_violation(u, lb, ub))
    return u, V2_MIN_RISK, info


def _bounds_violation(u, lb, ub):
    return float(max(0.0, float(np.max(lb - u)), float(np.max(u - ub))))
