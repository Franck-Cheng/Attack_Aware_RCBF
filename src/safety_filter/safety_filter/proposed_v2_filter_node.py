#!/usr/bin/env python3
"""Proposed V2 safety-filter node — three-tier STRICT -> SOFT -> MIN_RISK.

Isolated addition.  It subclasses the unmodified platform node
(`cbf_filter_node.CBFSafetyFilterNode`) and overrides ONLY the final control
path so that perception, world-lock, barrier construction, scan handling and
QoS stay byte-identical to the formal experiments.

Run it with:

    ros2 run safety_filter proposed_v2_filter_node \
        --ros-args -p final_mode:=true -p lambda_delta:=10.0 -p minrisk_tau:=1e-9 ...

Topic interface is unchanged:
    subscribe  /cmd_vel_nom (remap <- /nav2_cmd_vel), /scan, /odom, /tf
    publish    /cmd_vel_safe , /filter_status , /filter_metrics

/filter_status values:  V2_STRICT | V2_SOFT | V2_MIN_RISK | V2_NO_ROWS | V2_DEGRADED

/filter_metrics layout (timestamp + 19 values in the recorder CSV):
    [0] stage_id  (0=NO_ROWS, 1=STRICT, 2=SOFT, 3=MIN_RISK, 4=DEGRADED)
    [1] n_safety_rows
    [2] hard_min_margin          (>=0 means all robust rows satisfied)
    [3] bounds_violation         (0 means u inside U_sf)
    [4] eta                      (MIN_RISK ACHIEVED worst robust residual)
    [5] delta                    (SOFT task slack; 0 otherwise)
    [6] alpha                    (heading error -- DIAGNOSTIC ONLY, not constrained)
    [7] r                        (distance to goal -- DIAGNOSTIC ONLY)
    [8] scan_age
    [9] task_row_defined   (0/1)
    [10] strict_ok         (0/1)
    [11] soft_ok           (0/1)
    [12] u_nav_v           (v of the nominal command feeding the NOT_REVERSE row)
    [13] task_sign         (+1 -> A_t=[+1,0]; -1 -> A_t=[-1,0])
    [14] safety_only_feasible (1/0/-1 unknown)
    [15] eta_star          (MIN_RISK stage-1 exact LP optimum; 0 otherwise)
    [16] task_row_margin   (A_t u - b_t at the returned u; attribution evidence)
    [17] stage1_ms         (MIN_RISK stage-1 LP solve time)
    [18] stage2_ms         (MIN_RISK stage-2 QP solve time)
The frozen recorder is untouched: it serialises whatever this array holds.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

from safety_filter.cbf_filter_node import CBFSafetyFilterNode
from safety_filter import proposed_v2 as v2
from safety_filter.final_robust import (scan_points_world_at_ts,
                                        barrier_rows_from_worldpoints)

STAGE_ID = {v2.V2_NO_ROWS: 0, v2.V2_STRICT: 1, v2.V2_SOFT: 2,
            v2.V2_MIN_RISK: 3, v2.V2_DEGRADED: 4}


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ProposedV2Node(CBFSafetyFilterNode):
    """Three-tier variant of the platform safety filter."""

    def __init__(self):
        super().__init__()

        # ── V2-only parameters (brand new; no historical counterpart) ──
        self.declare_parameter("lambda_delta", 10.0)      # SOFT task-slack weight
        self.declare_parameter("minrisk_tau", 1e-9)       # MIN_RISK stage-2 feasibility band
        self.declare_parameter("task_mode", "not_reverse")

        self.lambda_delta = float(self.get_parameter("lambda_delta").value)
        self.minrisk_tau = float(self.get_parameter("minrisk_tau").value)
        self.task_mode = str(self.get_parameter("task_mode").value)
        if self.task_mode != "not_reverse":
            self.get_logger().warn(
                f"[PROPOSED-V2] task_mode='{self.task_mode}' is not supported by the "
                f"V2 task-compatibility row; forcing 'not_reverse'.")

        self._seen_anomalies = set()
        self._anomaly_count = 0

        # Guarantee routing into the final path regardless of the launch default.
        self.final_mode = True

        self.get_logger().info(
            f"[PROPOSED-V2] three-tier filter enabled | "
            f"lambda_delta={self.lambda_delta} | tau={self.minrisk_tau} | "
            f"task_row=NOT_REVERSE(u_nav) | MIN_RISK=lexicographic_2stage")

    # ── task-compatibility row: NOT_REVERSE, anchored on the REAL u_nav ────
    def _task_row_for(self, state, u_nom):
        """Build the single NOT_REVERSE row from the real nominal command.

        The row is derived from u_nav.v, never from goal geometry, so it cannot
        invent a task the planner did not ask for.  alpha/r are returned for
        diagnostics only; they constrain nothing.
        """
        A_t, b_t = v2.not_reverse_row(float(u_nom[0]))
        dx = self._goal[0] - state[0]
        dy = self._goal[1] - state[1]
        r = float(math.hypot(dx, dy))
        alpha = wrap_to_pi(math.atan2(dy, dx) - float(state[2]))
        return (A_t, b_t), alpha, r

    # ── the one overridden method ────────────────────────────────────────
    def _final_control_loop(self, state, u_nom, now_sec, scan_age, scan_degraded):
        # (1) degradation policy -- identical to the platform node
        degraded = scan_degraded
        if (not self._scan_stamp_valid or len(self._latest_scan_ranges) == 0
                or not self._state_valid or not self._u_nom_valid):
            degraded = True
        if self.final_mode and not getattr(self, "_final_scan_valid", False):
            degraded = True
        if degraded:
            self._publish_v2(np.array([0.0, 0.0]), v2.V2_DEGRADED, scan_age,
                             {"n_safety_rows": 0, "hard_min_margin": float("nan"),
                              "bounds_violation": 0.0, "task_row_defined": 0,
                              "strict_ok": 0, "soft_ok": 0})
            return

        # (2) world-lock perception -- identical to the platform node
        margins = {"delta": self.barrier_builder.total_margin,
                   "point_radius": self.barrier_builder.d_meas + 0.01}
        q_W, _valid = scan_points_world_at_ts(
            self._latest_scan_ranges, self._latest_scan_angles, self._range_max,
            self._final_T_WL)

        # (3) barrier rows -- identical construction
        rows, min_h = barrier_rows_from_worldpoints(
            state, q_W, self.barrier_builder.body_disks,
            self.alpha_h, self.a_v_max, self.a_w_max, margins)

        # (4) three-tier solve
        H = np.diag([float(self.get_parameter("w_v").value),
                     float(self.get_parameter("w_w").value)])
        lb = np.array([self.filter_core.v_safe_min, -self.filter_core.w_safe_max], float)
        ub = np.array([self.filter_core.v_safe_max, self.filter_core.w_safe_max], float)

        u_nav = np.asarray(u_nom, float)
        task, alpha, r = self._task_row_for(state, u_nav)
        u_sf, stage, info = v2.three_tier(
            u_nav, rows, H=H, lb=lb, ub=ub,
            task=task, lam=self.lambda_delta, tau=self.minrisk_tau)

        info.setdefault("n_safety_rows", len(rows))
        info["task_row_defined"] = 1
        info["u_nav_v"] = float(u_nav[0])
        info["task_sign"] = float(task[0][0])
        info["alpha"] = alpha
        info["r"] = r
        info["min_h"] = float(min_h)
        self._publish_v2(u_sf, stage, scan_age, info)

    # ── publication with the V2 metrics layout ───────────────────────────
    def _publish_v2(self, u_sf, stage, scan_age, info):
        u_sf = np.clip(np.asarray(u_sf, float),
                       [self.filter_core.v_safe_min, -self.filter_core.w_safe_max],
                       [self.filter_core.v_safe_max, self.filter_core.w_safe_max])

        cmd = Twist()
        cmd.linear.x = float(u_sf[0])
        cmd.angular.z = float(u_sf[1])
        self._pub_cmd_safe.publish(cmd)

        st = String()
        st.data = stage
        self._pub_status.publish(st)

        def num(key, default=0.0):
            x = info.get(key, default)
            try:
                x = float(x)
            except Exception:
                return 0.0
            return x if math.isfinite(x) else 0.0

        strict_ok = 1.0 if info.get("strict") and "objective" in info.get("strict", {}) else 0.0
        soft_ok = 1.0 if info.get("soft") and "objective" in info.get("soft", {}) else 0.0
        safe_only = info.get("safety_only_feasible", None)
        safe_only_f = -1.0 if safe_only is None else (1.0 if safe_only else 0.0)
        _mr = info.get("minrisk") or {}
        _s1 = (_mr.get("stage1") or {}).get("ms", 0.0) or 0.0
        _s2 = (_mr.get("stage2") or {}).get("ms", 0.0) or 0.0

        m = Float32MultiArray()
        m.data = [
            float(STAGE_ID.get(stage, -1)),
            float(info.get("n_safety_rows", 0)),
            num("hard_min_margin"),
            num("bounds_violation"),
            num("eta"),
            num("delta"),
            num("alpha"),
            num("r"),
            float(scan_age) if math.isfinite(scan_age) else 0.0,
            float(info.get("task_row_defined", 0)),
            strict_ok,
            soft_ok,
            num("u_nav_v"),
            num("task_sign"),
            safe_only_f,
            num("eta_star"),
            num("task_row_margin"),
            float(_s1),
            float(_s2),
        ]
        self._pub_metrics.publish(m)

        anomalies = list(info.get("anomalies") or [])
        self._anomaly_count += len(anomalies)
        for a in anomalies:
            if a not in self._seen_anomalies:
                self._seen_anomalies.add(a)
                self.get_logger().warn(f"[V2-ANOMALY] {a}")

        if getattr(self, "_cnt", 0) % 20 == 0:
            self.get_logger().info(
                f"[V2] stage={stage:11s} rows={int(info.get('n_safety_rows',0)):3d} "
                f"margin={num('hard_min_margin'):+.4f} eta={num('eta'):+.4f} "
                f"eta*={num('eta_star'):+.4f} delta={num('delta'):.4f} "
                f"u_nav_v={num('u_nav_v'):+.3f} task_sign={num('task_sign'):+.0f} "
                f"u_sf=({u_sf[0]:+.3f},{u_sf[1]:+.3f}) "
                f"t1={_s1:.2f}ms t2={_s2:.2f}ms "
                f"anom_total={self._anomaly_count} "
                f"reason={info.get('stage_reason','-')}")
        self._cnt = getattr(self, "_cnt", 0) + 1


def main(args=None):
    rclpy.init(args=args)
    node = ProposedV2Node()
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ex.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
