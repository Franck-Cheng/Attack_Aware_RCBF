#!/usr/bin/env python3
"""
CBF 安全过滤器 ROS2 节点 v2.0 (safety_filter_node)

输入 topics:
  /odom          (nav_msgs/Odometry)  — 机器人状态
  /scan          (sensor_msgs/LaserScan) — LiDAR 点云
  /cmd_vel_nom   (geometry_msgs/Twist) — 名义控制输入

输出 topics:
  /cmd_vel_safe  (geometry_msgs/Twist) — 过滤后安全控制
  /filter_status (std_msgs/String)     — "strict" | "soft" | "min_risk"
  /filter_metrics (Float32MultiArray)  — η_H, η_S, num_constraints, min_clearance

v2.0 新增:
  - scan 时间戳跟踪与 age 检查
  - 膨胀参数可配置
  - 角度顺序聚类 / 线段 primitive 可切换
  - scan 过期时的安全降级策略
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray

from .barrier_builder import BarrierBuilder, CBFConstraintRow, InflationParams
from .safety_filter_core import SafetyFilterCore, FilterResult
from .geometry_utils import wrap_to_pi
from . import final_robust  # AUDIT16.1 FINAL 核心 (同一份代码供 unit test + runtime)

# AUDIT16.1: TF2 for measurement-time world-lock (§4)
try:
    from tf2_ros import Buffer, TransformListener
    from tf2_geometry_msgs import do_transform_point
    from geometry_msgs.msg import PointStamped
    _HAS_TF2 = True
except Exception:  # pragma: no cover
    _HAS_TF2 = False


class CBFSafetyFilterNode(Node):
    """攻击感知 CBF 安全过滤器 ROS2 节点。"""

    def __init__(self):
        super().__init__("safety_filter_node")

        # ─── 参数声明 ───────────────────────────────
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("alpha_h", 1.6)

        # ── FINAL MODE (AUDIT16.1): 论文最终唯一路径 ──
        self.declare_parameter("final_mode", False)

        # ── 安全过滤输出边界（直设，不通过 Pontryagin 差推导）──
        self.declare_parameter("v_safe_min", -0.60)
        self.declare_parameter("v_safe_max", 0.60)
        self.declare_parameter("w_safe_max", 2.20)

        # ── 攻击参数 —— 必须与 actuator_attack_node 一致 ──
        self.declare_parameter("a_v_max", 0.4)
        self.declare_parameter("a_w_max", 1.6)

        # ── 轮级变化率约束参数 ──
        self.declare_parameter("enable_rate_constraints", False)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_base", 0.160)
        self.declare_parameter("max_wheel_accel", 1.0)

        # ── QP 权重参数 ──
        self.declare_parameter("w_v", 2.0)
        self.declare_parameter("w_w", 0.8)
        self.declare_parameter("w_slack", 80.0)
        self.declare_parameter("v_nom_max", 0.55)
        self.declare_parameter("v_back", 0.22)

        # ── Barrier builder 参数 ──
        self.declare_parameter("active_h_threshold", 1.25)
        self.declare_parameter("cluster_dist", 0.20)
        self.declare_parameter("d_geom", 0.02)
        self.declare_parameter("d_meas", 0.015)
        self.declare_parameter("d_delay", 0.01)
        self.declare_parameter("d_hold", 0.01)
        self.declare_parameter("max_constraint_rows", 32)
        self.declare_parameter("min_object_radius", 0.05)

        # ── v2.0 新增参数 ──
        self.declare_parameter("use_ordered_clustering", True)
        self.declare_parameter("use_segment_primitive", True)
        self.declare_parameter("max_scan_age", 0.50)
        self.declare_parameter("eps_scan", 0.03)
        self.declare_parameter("eps_noise", 0.03)
        self.declare_parameter("eps_tf", 0.01)
        self.declare_parameter("eps_cover", 0.03)
        self.declare_parameter("max_inflation", 0.20)

        # ── 简化模型: 局部极小值点障碍物 ──
        self.declare_parameter("use_local_minima", False)
        self.declare_parameter("tau0", 0.04)
        self.declare_parameter("k_theta", 2.0)
        self.declare_parameter("k_sigma", 3.0)
        self.declare_parameter("sigma_r", 0.01)
        self.declare_parameter("nearest_obstacles_per_disk", 2)

        # ── RPSF 激活阈值 (min_h 低于此值才触发鲁棒预测安全过滤) ──
        # 0.60 过保守: 正常导航间隙 ~0.09m 时 RPSF 过早介入并与 Nav2 对抗
        self.declare_parameter("rpsf_activation_h", 0.25)

        # ── v2.2 时域预测安全检查 ──
        self.declare_parameter("horizon_check_enabled", True)
        self.declare_parameter("horizon_steps", 8)
        self.declare_parameter("horizon_dt", 0.05)
        self.declare_parameter("horizon_safety_margin", 0.02)
        self.declare_parameter("horizon_brake_gain", 0.8)

        # 读取参数
        self.alpha_h = self.get_parameter("alpha_h").value
        self.a_v_max = self.get_parameter("a_v_max").value
        self.a_w_max = self.get_parameter("a_w_max").value
        self.final_mode = bool(self.get_parameter("final_mode").value)

        # ── FINAL MODE 状态缓存 (AUDIT16.1) ──
        self._final_q_W = None            # world-locked scan points (measurement-time)
        self._final_scan_stamp = 0.0
        self._final_scan_frame = "base_scan"
        self._final_last_status = "OTHER"
        self._final_diag = {"feasibility_status": final_robust.FEASIBILITY_FEASIBLE,
                            "control_action": final_robust.CTRL_NOMINAL,
                            "safe_guarantee": True,
                            "safety_infeasible": False}
        self._final_T_WL = None           # T_WL (world->scan frame) at scan.header.stamp
        self._final_scan_valid = False

        # ── 时域预测参数 ──
        self.horizon_check_enabled = self.get_parameter("horizon_check_enabled").value
        self.horizon_steps = self.get_parameter("horizon_steps").value
        self.horizon_dt = self.get_parameter("horizon_dt").value
        self.horizon_safety_margin = self.get_parameter("horizon_safety_margin").value
        self.horizon_brake_gain = self.get_parameter("horizon_brake_gain").value

        # ─── 膨胀参数 ──────────────────────────────
        inflation_params = InflationParams(
            eps_scan=self.get_parameter("eps_scan").value,
            eps_noise=self.get_parameter("eps_noise").value,
            eps_tf=self.get_parameter("eps_tf").value,
            eps_cover=self.get_parameter("eps_cover").value,
            max_inflation=self.get_parameter("max_inflation").value,
        )

        # ─── 初始化 barrier builder ──────────────────
        self.barrier_builder = BarrierBuilder(
            alpha_h=self.alpha_h,
            a_v_max=self.a_v_max,
            a_w_max=self.a_w_max,
            v_safe_max=self.get_parameter("v_safe_max").value,
            w_safe_max=self.get_parameter("w_safe_max").value,
            active_h_threshold=self.get_parameter("active_h_threshold").value,
            cluster_dist=self.get_parameter("cluster_dist").value,
            d_geom=self.get_parameter("d_geom").value,
            d_meas=self.get_parameter("d_meas").value,
            d_delay=self.get_parameter("d_delay").value,
            d_hold=self.get_parameter("d_hold").value,
            min_object_radius=self.get_parameter("min_object_radius").value,
            max_constraint_rows=self.get_parameter("max_constraint_rows").value,
            use_ordered_clustering=self.get_parameter("use_ordered_clustering").value,
            use_segment_primitive=self.get_parameter("use_segment_primitive").value,
            max_scan_age=self.get_parameter("max_scan_age").value,
            inflation=inflation_params,
            # 简化模型参数
            use_local_minima=self.get_parameter("use_local_minima").value,
            tau0=self.get_parameter("tau0").value,
            k_theta=self.get_parameter("k_theta").value,
            k_sigma=self.get_parameter("k_sigma").value,
            sigma_r=self.get_parameter("sigma_r").value,
            nearest_obstacles_per_disk=self.get_parameter("nearest_obstacles_per_disk").value,
        )

        # ─── 初始化 safety filter core（直设 U_safe + 攻击边界）──
        self.filter_core = SafetyFilterCore(
            v_safe_min=self.get_parameter("v_safe_min").value,
            v_safe_max=self.get_parameter("v_safe_max").value,
            w_safe_max=self.get_parameter("w_safe_max").value,
            a_v_max=self.a_v_max,
            a_w_max=self.a_w_max,
            w_v=self.get_parameter("w_v").value,
            w_w=self.get_parameter("w_w").value,
            w_slack=self.get_parameter("w_slack").value,
            v_nom_max=self.get_parameter("v_nom_max").value,
            v_back=self.get_parameter("v_back").value,
            enable_rate_constraints=self.get_parameter("enable_rate_constraints").value,
            wheel_radius=self.get_parameter("wheel_radius").value,
            wheel_base=self.get_parameter("wheel_base").value,
            max_wheel_accel=self.get_parameter("max_wheel_accel").value,
        )

        # safe bounds（用于日志）
        safe_v_min = self.filter_core.v_safe_min
        safe_v_max = self.filter_core.v_safe_max
        safe_w_max = self.filter_core.w_safe_max

        # ─── 状态缓存 ────────────────────────────────
        self._state: np.ndarray = np.zeros(3)       # [x, y, θ]
        self._state_valid = False
        self._latest_scan_ranges: np.ndarray = np.zeros(0)
        self._latest_scan_angles: np.ndarray = np.zeros(0)
        self._range_max = 3.5
        self._angle_increment = 0.01745  # 默认 2π/360
        self._latest_u_nom: np.ndarray = np.zeros(2)
        self._u_nom_valid = False

        # scan age 跟踪
        self._last_scan_stamp_sec: float = 0.0
        self._scan_stamp_valid: bool = False

        # ─── QoS ─────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ─── Subscribers ─────────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )
        self._sub_scan = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, sensor_qos
        )
        self._sub_cmd_nom = self.create_subscription(
            Twist, "/cmd_vel_nom", self._cmd_nom_cb, 10
        )

        # AUDIT16.1 FINAL: TF2 for measurement-time world-lock (§4)
        self._tf_buffer = None
        self._tf_listener = None
        self._world_frame = "odom"  # sim 中 map==odom (static 0), odom 为权威
        if _HAS_TF2:
            try:
                # Buffer(node=self): 使用节点时钟 (use_sim_time=true 时 ROS 时间),
                # 否则默认系统时钟 → sim 时间戳 TF 全部过期 → lookup 必失败.
                self._tf_buffer = Buffer(node=self)
                # TransformListener 保持默认 (spin_thread=False), 由主 executor
                # (MultiThreadedExecutor, 见 main) 多线程消费 /tf —— 不能 spin_thread=True,
                # 否则它自建 executor 与主 spin 同管理一个节点 → 冲突/滞后.
                self._tf_listener = TransformListener(self._tf_buffer, self)
                self.get_logger().info("[FINAL] TF2 world-lock enabled (odom, node clock, MT executor)")
            except Exception as e:  # pragma: no cover
                self.get_logger().warn(f"[FINAL] TF2 init failed: {e}")

        # ─── Publishers ──────────────────────────────
        self._pub_cmd_safe = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self._pub_status = self.create_publisher(String, "/filter_status", 10)
        self._pub_metrics = self.create_publisher(
            Float32MultiArray, "/filter_metrics", 10
        )

        # ─── 定时器 (控制频率) ───────────────────────
        self.dt = 0.10  # 10 Hz control loop
        self._timer = self.create_timer(self.dt, self._control_loop)

        # ─── 目标信息 ───────────────────────────
        self._goal: np.ndarray = np.array([
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value,
        ])
        self._goal_valid = True

        self.get_logger().info("CBF Safety Filter node v2.1 started.")
        self.get_logger().info(
            f"  U_safe: v∈[{safe_v_min:.2f}, {safe_v_max:.2f}], ω∈[-{safe_w_max:.2f}, {safe_w_max:.2f}]"
        )
        self.get_logger().info(
            f"  Attack: a_v∈[-{self.a_v_max:.2f}, {self.a_v_max:.2f}], "
            f"a_w∈[-{self.a_w_max:.2f}, {self.a_w_max:.2f}]"
        )
        self.get_logger().info(
            f"  U_safe: v∈[{safe_v_min:.3f}, {safe_v_max:.3f}], ω∈[-{safe_w_max:.3f}, {safe_w_max:.3f}]"
        )
        self.get_logger().info(
            f"  Clustering: {'ordered' if self.barrier_builder.use_ordered_clustering else 'BFS'}"
        )
        self.get_logger().info(
            f"  Primitive: {'segment' if self.barrier_builder.use_segment_primitive else 'circle (Welzl)'}"
        )
        self.get_logger().info(
            f"  Inflation: scan={inflation_params.eps_scan:.2f} noise={inflation_params.eps_noise:.2f} "
            f"tf={inflation_params.eps_tf:.2f} cover={inflation_params.eps_cover:.2f} "
            f"max={inflation_params.max_inflation:.2f}"
        )
        self.get_logger().info(f"  Max scan age: {self.barrier_builder.max_scan_age:.2f}s")

    # ─── Callbacks ──────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._state[0] = msg.pose.pose.position.x
        self._state[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._state[2] = math.atan2(siny, cosy)
        self._state_valid = True
        # 缓存 odom 时间戳用于 world-lock (scan-time pose)
        self._odom_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._odom_state = self._state.copy()

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        self._latest_scan_ranges = np.array(msg.ranges, dtype=float)
        self._latest_scan_angles = (
            msg.angle_min + np.arange(n) * msg.angle_increment
        )
        self._range_max = float(msg.range_max)
        self._angle_increment = float(msg.angle_increment)

        # 记录 scan 时间戳
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._last_scan_stamp_sec = stamp_sec
        self._scan_stamp_valid = True

        # 通知 barrier_builder 新的 scan 时间戳
        self.barrier_builder.register_scan_stamp(stamp_sec)

        # AUDIT16.1 FINAL: measurement-time world-lock (scan.header.frame_id 权威)
        if self.final_mode:
            self._final_scan_frame = str(msg.header.frame_id)
            self._final_scan_stamp = stamp_sec
            twl = None
            if self._tf_buffer is not None:
                try:
                    # TF2 timestamp lookup: odom -> scan frame at scan.header.stamp
                    tf = self._tf_buffer.lookup_transform(
                        self._world_frame, self._final_scan_frame,
                        rclpy.time.Time(seconds=stamp_sec), timeout=rclpy.duration.Duration(seconds=0.1))
                    R = tf.transform.rotation
                    # 2D world-lock (map/odom 平面); 存 4x4 含 z=0
                    import math as _m
                    th = _m.atan2(2.0*(R.w*R.z + R.x*R.y),
                                  1.0 - 2.0*(R.y*R.y + R.z*R.z))
                    c_, s_ = _m.cos(th), _m.sin(th)
                    twl = np.array([[c_, -s_, 0.0, tf.transform.translation.x],
                                    [s_,  c_, 0.0, tf.transform.translation.y],
                                    [0.0, 0.0, 1.0, tf.transform.translation.z],
                                    [0.0, 0.0, 0.0, 1.0]])
                except Exception as e:
                    twl = None  # TF at measurement time 失败 → 不静默用当前 pose
                    if getattr(self, "_tf_fail_cnt", 0) % 100 == 0:
                        self.get_logger().warn(
                            f"[FINAL] TF lookup failed: {type(e).__name__}: {e} "
                            f"(world={self._world_frame} scan_frame={self._final_scan_frame} "
                            f"stamp={stamp_sec:.3f})")
                    self._tf_fail_cnt = getattr(self, "_tf_fail_cnt", 0) + 1
            if twl is None:
                # AUDIT16.1 收尾: 禁止 latest odom fallback.
                # TF2 不可用/lookup 失败 → 不伪造 measurement-time world-lock,
                # 统一 _final_scan_valid=False → DEGRADED_SENSOR.
                self.get_logger().warn(
                    "[FINAL] world-lock: TF2 unavailable/failed at scan time -> DEGRADED_SENSOR "
                    "(no odom fallback)")
            self._final_T_WL = twl
            self._final_scan_valid = twl is not None

    def _cmd_nom_cb(self, msg: Twist):
        self._latest_u_nom = np.array([msg.linear.x, msg.angular.z])
        self._u_nom_valid = True

    # ─── 时域预测安全检查 ──────────────────────────
    def _horizon_safety_check(self, state, u_proposed):
        """简单时域 rollout：预测 H 步后的最小 clearance。
        
        如果任何未来步 clearance < safety_margin，按比例制动。
        返回缩放后的安全命令。
        """
        if not self.horizon_check_enabled:
            return u_proposed

        ranges = self._latest_scan_ranges
        angles = self._latest_scan_angles
        if len(ranges) == 0:
            return u_proposed

        # 将 scan 转到世界坐标（只取有限点）
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < self._range_max - 0.1)
        if not np.any(valid):
            return u_proposed

        r = ranges[valid]
        a = angles[valid]
        pts_body = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)

        # 使用 SF 自身的 body disks (真实 footprint), 不用攻击器大偏移盘
        disks = self.barrier_builder.body_disks

        v = float(u_proposed[0])
        w = float(u_proposed[1])

        x, y, th = float(state[0]), float(state[1]), float(state[2])

        min_future_h = 1e6
        for step in range(1, self.horizon_steps + 1):
            # 前向模拟
            x += self.horizon_dt * v * math.cos(th)
            y += self.horizon_dt * v * math.sin(th)
            th += self.horizon_dt * w
            th = math.atan2(math.sin(th), math.cos(th))

            # 将 body disks 转到世界坐标
            c, s = math.cos(th), math.sin(th)
            R = np.array([[c, -s], [s, c]])
            pts_world = (R @ pts_body.T).T + np.array([x, y])

            # 计算每个 disk 到最近 LiDAR 点的 clearance
            for dx, dy, rd in disks:
                wx = x + c * dx - s * dy
                wy = y + s * dx + c * dy
                d = np.sqrt((pts_world[:, 0] - wx) ** 2 + (pts_world[:, 1] - wy) ** 2)
                # 简化：用 scan 点距离减去 disk 半径
                # 更精确：对每个 disk center 计算到每个 scan 点的距离
                disk_d = np.sqrt((pts_world[:, 0] - wx) ** 2 + (pts_world[:, 1] - wy) ** 2)
                h_disk = np.min(disk_d) - rd - self.horizon_safety_margin
                if h_disk < min_future_h:
                    min_future_h = h_disk

        # 如果未来 clearance 低于安全阈值，搜索安全方向
        if min_future_h < 0:
            # 策略 1: 在 [-w_safe_max, w_safe_max] 中搜索安全转角
            best_w = w
            best_h = min_future_h
            for w_candidate in np.linspace(-self.filter_core.w_safe_max,
                                           self.filter_core.w_safe_max, 11):
                # 重新 rollout
                x2, y2, th2 = float(state[0]), float(state[1]), float(state[2])
                fc_h = 1e6
                for step in range(self.horizon_steps):
                    x2 += self.horizon_dt * v * math.cos(th2)
                    y2 += self.horizon_dt * v * math.sin(th2)
                    th2 += self.horizon_dt * w_candidate
                    th2 = math.atan2(math.sin(th2), math.cos(th2))
                    c2, s2 = math.cos(th2), math.sin(th2)
                    pts_w2 = (np.array([[c2,-s2],[s2,c2]]) @ pts_body.T).T + np.array([x2, y2])
                    for dx, dy, rd in disks:
                        wx = x2 + c2*dx - s2*dy
                        wy = y2 + s2*dx + c2*dy
                        dd = np.sqrt((pts_w2[:,0]-wx)**2 + (pts_w2[:,1]-wy)**2)
                        hd = np.min(dd) - rd - self.horizon_safety_margin
                        if hd < fc_h: fc_h = hd
                if fc_h > best_h:
                    best_h = fc_h
                    best_w = w_candidate
            if best_h < 0:
                # 搜索失败 → 紧急制动
                return np.array([0.0, 0.0])
            return np.array([v, best_w])

        # 轻度预警
        if min_future_h < 0.05:
            scale = max(0.5, 1.0 - self.horizon_brake_gain * (0.05 - min_future_h) / 0.05)
            return np.array([v * scale, w * scale])

        return u_proposed

    # ─── 控制循环 ──────────────────────────────────
    def _control_loop(self):
        try:
            self._control_loop_impl()
        except Exception as e:
            cnt = self._control_loop.__dict__.get("_err_cnt", 0)
            self._control_loop.__dict__["_err_cnt"] = cnt + 1
            if cnt % 50 == 0:
                self.get_logger().error(
                    f"Control loop exception (#{cnt+1}): {e}")
            # Publish zero velocity on error to be safe
            cmd_msg = Twist()
            self._pub_cmd_safe.publish(cmd_msg)

    def _get_lidar_points_world(self, state: np.ndarray) -> np.ndarray:
        """将当前 LiDAR scan 转换到世界坐标系，返回 (N,2) 数组。"""
        ranges = self._latest_scan_ranges
        angles = self._latest_scan_angles
        if len(ranges) == 0:
            return np.zeros((0, 2))
        valid = (ranges > 0.01) & (ranges < self._range_max - 0.1)
        if not np.any(valid):
            return np.zeros((0, 2))
        r = ranges[valid]
        a = angles[valid]
        x_body = r * np.cos(a)
        y_body = r * np.sin(a)
        c = math.cos(state[2])
        s = math.sin(state[2])
        R = np.array([[c, -s], [s, c]])
        pts_body = np.column_stack([x_body, y_body])
        pts_world = (R @ pts_body.T).T + np.array([state[0], state[1]])
        return pts_world

    def _final_control_loop(self, state, u_nom, now_sec, scan_age, scan_degraded):
        """AUDIT16.1 FINAL: 论文最终路径 (world-lock + hard-safe QP + revalidate).

        语义:
          - DEGRADED_SENSOR (scan 过期/缺失/odom 缺失/TF 失败): u=(0,0), 无 robust guarantee
          - 否则: measurement-time world-lock 的障碍点 q_W + 当前 state barrier
          - final_filter: U1 + hard robust CBF rows (无 task rows)
          - publish 前 revalidation (psi>= -tol); 失败→SOLVER_INVALID/min-risk
          - 无 transparency clamp, 无 task hard rows, 无 v>=0 gate
        """
        # 1) DEGRADED_SENSOR policy (冻结)
        degraded = scan_degraded
        if (not self._scan_stamp_valid or len(self._latest_scan_ranges) == 0
                or not self._state_valid or not self._u_nom_valid):
            degraded = True
        if self.final_mode and not getattr(self, "_final_scan_valid", False):
            degraded = True  # scan-time odom pose 不可用 → world-lock 无法建立

        if degraded:
            u_sf = np.array([0.0, 0.0])
            status = final_robust.DEGRADED_SENSOR
            self._final_last_status = status
            info = final_robust._diagnostic_info(
                False, final_robust.CTRL_DEGRADED_STOP, False,
                float("nan"), float("nan"), 0, safety_infeasible=False)
            self._publish_final(u_sf, status, scan_age, 0, info)
            if getattr(self, "_cnt", 0) % 20 == 0:
                self.get_logger().warn(
                    f"[FINAL] DEGRADED_SENSOR: age={scan_age:.3f} "
                    f"scan_valid={self._scan_stamp_valid} odom_valid={self._state_valid}")
            return

        # 2) world-lock perception: q_W = T_WL(t_s) q_L (scan 时刻锁定, 缓存)
        margins = {"delta": self.barrier_builder.total_margin,
                   "point_radius": self.barrier_builder.d_meas + 0.01}
        q_W, valid = final_robust.scan_points_world_at_ts(
            self._latest_scan_ranges, self._latest_scan_angles, self._range_max,
            self._final_T_WL,   # T_WL (world->scan frame at scan.header.stamp)
        )
        # base_scan extrinsic: body 系 scan 已是 base_scan frame;
        # 此处 T_WL 由 odom_at_scan (旋转+平移) + 无需额外 extrinsic (URDF 已含,
        # 且 sim 中 odom==base_scan 原点仅差 x=-0.032; 由 odom_at_scan 近似).
        self._final_q_W = q_W

        # 3) barrier rows at CURRENT state (world-locked q_W)
        rows, min_h = final_robust.barrier_rows_from_worldpoints(
            state, q_W, self.barrier_builder.body_disks,
            self.alpha_h, self.a_v_max, self.a_w_max, margins)

        # 4) Final hard-safe QP (U1 + hard rows only)
        u_sf, status, info = final_robust.final_filter(
            self.filter_core, u_nom, rows,
            attack_bounds=(self.a_v_max, self.a_w_max),
            H=np.diag([self.get_parameter("w_v").value,
                       self.get_parameter("w_w").value]),
            tol=1e-6,
        )
        self._final_last_status = status

        # 5) publish + diagnostics (携带 info: feasibility_status/control_action/safe_guarantee)
        self._publish_final(u_sf, status, scan_age, len(rows), info)

    def _publish_final(self, u_sf, status, scan_age, nrows, info=None):
        """发布 /cmd_vel_safe + /filter_status + /filter_metrics (Final).

        info (可选) 携带无歧义诊断:
          feasibility_status: FEASIBLE | SAFETY_INFEASIBLE
          control_action:     NOMINAL | FILTERED | MIN_RISK | SOLVER_INVALID | DEGRADED_STOP
          safe_guarantee:     bool
        """
        u_sf = np.clip(u_sf, [self.filter_core.v_safe_min, -self.filter_core.w_safe_max],
                       [self.filter_core.v_safe_max, self.filter_core.w_safe_max])
        cmd_msg = Twist()
        cmd_msg.linear.x = float(u_sf[0])
        cmd_msg.angular.z = float(u_sf[1])
        self._pub_cmd_safe.publish(cmd_msg)

        status_msg = String()
        status_msg.data = status
        self._pub_status.publish(status_msg)

        # 诊断字段 (info 缺失时 fallback 到 status 推断)
        feasibility_status = final_robust.FEASIBILITY_FEASIBLE
        control_action = final_robust.CTRL_NOMINAL
        safe_guarantee = status in (final_robust.SAFE_NOMINAL, final_robust.SAFE_FILTERED)
        safety_infeasible = False
        if info is not None:
            feasibility_status = info.get("feasibility_status", feasibility_status)
            control_action = info.get("control_action", control_action)
            safe_guarantee = bool(info.get("safe_guarantee", safe_guarantee))
            safety_infeasible = bool(info.get("safety_infeasible", False))
        self._final_diag = {"feasibility_status": feasibility_status,
                            "control_action": control_action,
                            "safe_guarantee": safe_guarantee,
                            "safety_infeasible": safety_infeasible}

        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            0.0, 0.0, float(nrows), 0.0,
            1.0 if safe_guarantee else 0.0,
            0.0, float(scan_age), 0.0, 0.0,
            1.0 if status == final_robust.DEGRADED_SENSOR else 0.0,
        ]
        self._pub_metrics.publish(metrics_msg)
        if getattr(self, "_cnt", 0) % 20 == 0:
            self.get_logger().info(
                f"[FINAL] status={status:16s} feas={feasibility_status:16s} "
                f"act={control_action:15s} guarantee={int(safe_guarantee)} "
                f"u_sf=({u_sf[0]:.3f},{u_sf[1]:.3f}) rows={nrows} scan_age={scan_age:.3f}")

    def _control_loop_impl(self):
        if not self._state_valid or not self._u_nom_valid:
            return

        state = self._state.copy()
        u_nom = self._latest_u_nom.copy()

        # ── Scan age 检查 ──
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        scan_age = self.barrier_builder._get_scan_age(now_sec)
        scan_degraded = self.barrier_builder._is_scan_degraded(now_sec)

        # AUDIT16.1 FINAL MODE: 论文最终唯一路径
        if self.final_mode:
            self._final_control_loop(state, u_nom, now_sec, scan_age, scan_degraded)
            return

        # 1) 构造 CBF 约束行（使用同一时钟源）
        result = self.barrier_builder.build_rows_v2(
            state,
            self._latest_scan_ranges,
            self._latest_scan_angles,
            self._range_max,
            self._angle_increment,
            current_time_sec=now_sec,
        )

        rows = result.rows
        min_h = result.min_h

        # scan 严重过期 → 强制减速
        if result.scan_degraded:
            u_clamped = np.array([
                float(np.clip(u_nom[0], self.filter_core.v_safe_min,
                              min(self.filter_core.v_safe_max, 0.05))),
                float(np.clip(u_nom[1], -0.1, 0.1)),
            ])
            cmd_msg = Twist()
            cmd_msg.linear.x = float(u_clamped[0])
            cmd_msg.angular.z = float(u_clamped[1])
            self._pub_cmd_safe.publish(cmd_msg)

            status_msg = String()
            status_msg.data = "min_risk"
            self._pub_status.publish(status_msg)

            if self._control_loop.__dict__.get("_cnt", 0) % 20 == 0:
                self.get_logger().warn(
                    f"SCAN DEGRADED: age={scan_age:.3f}s > {self.barrier_builder.max_scan_age:.2f}s. "
                    f"Forcing safe slowdown."
                )
            self._control_loop.__dict__["_cnt"] = (
                self._control_loop.__dict__.get("_cnt", 0) + 1
            )
            return

        # 2) 计算目标跟踪误差（简化版）
        dx = self._goal[0] - state[0]
        dy = self._goal[1] - state[1]
        r = math.hypot(dx, dy)
        alpha = wrap_to_pi(math.atan2(dy, dx) - state[2])
        mode = "heading" if abs(alpha) > math.radians(28.0) else "progress"

        # 3) 安全过滤（strict → soft → min-risk QP）
        result_filter: FilterResult = self.filter_core.filter(
            u_nom, rows, alpha=alpha, r=r, mode=mode
        )

        # 3.5) 时域预测安全检查 —— 对未来 H 步做 rollout
        u_safe_horizon = self._horizon_safety_check(state, result_filter.u_safe)

        # 3.6) RPSF — 鲁棒预测安全检查（仅障碍物接近时激活）
        lidar_pts_world = self._get_lidar_points_world(state)
        rpsf_act_h = float(self.get_parameter("rpsf_activation_h").value)
        if len(lidar_pts_world) > 0 and result.min_h < rpsf_act_h:
            u_rpsf, rpsf_status, rpsf_info = self.filter_core.robust_predictive_filter(
                u_safe_horizon, u_nom, state,
                lidar_pts_world,
                self.barrier_builder.body_disks,
                horizon=self.horizon_steps,
                dt=self.horizon_dt,
                safety_margin=self.horizon_safety_margin,
                Nv=7, Nw=11,
            )
            u_safe_horizon = u_rpsf
            if rpsf_status == "rpsf_emergency":
                result_filter.status = "rpsf_emergency"

        # 3.9) 透明性钳制: SF 是过滤器 — 平时只减速+转向, 永不加速、永不倒车。
        # 后方障碍物的 CBF 行(ℓ_v<0)会形成下界约束强迫 v 增大(最高到 v_safe_max),
        # 导致 SF 把机器人推得比 Nav2 名义速度还快 → 冲进前方障碍/墙壁。
        # 例外: 危急时 (min_h < 鲁棒CBF倒车阈值 a_vmax/α) 临时解锁倒车,
        # 因为攻击的前向推力 u_act = u_sf + a_v 在 u_sf≥0 时至少 +a_vmax,
        # 只有倒车 (u_sf < -a_vmax) 才能保证净运动远离障碍物。
        v_nom_val = float(u_nom[0])
        h_reverse_thresh = self.a_v_max / max(self.alpha_h, 1e-6) + 0.01
        if result.min_h < h_reverse_thresh and result.min_h > -1e9:
            # 危急: 允许倒车 (脱困), 但上限仍不超过名义前进速度
            u_safe_horizon[0] = float(np.clip(
                u_safe_horizon[0], self.filter_core.v_safe_min,
                max(0.0, v_nom_val)))
        else:
            # 平时: 完全透明钳制
            v_lo = min(0.0, v_nom_val)
            v_hi = max(0.0, v_nom_val)
            u_safe_horizon[0] = float(np.clip(u_safe_horizon[0], v_lo, v_hi))

        # 4) 发布 /cmd_vel_safe
        cmd_msg = Twist()
        cmd_msg.linear.x = float(u_safe_horizon[0])
        cmd_msg.angular.z = float(u_safe_horizon[1])
        self._pub_cmd_safe.publish(cmd_msg)

        # 5) 发布 /filter_status
        status_msg = String()
        status_msg.data = result_filter.status
        self._pub_status.publish(status_msg)

        # 6) 发布 /filter_metrics（含 scan age 诊断信息）
        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            float(result_filter.eta_h),
            float(result_filter.eta_s),
            float(result_filter.num_constraints),
            float(result_filter.min_clearance) if not math.isinf(result_filter.min_clearance) else 999.0,
            1.0 if result_filter.qp_feasible else 0.0,
            float(result_filter.soft_slack),
            float(scan_age),                       # [6] scan age
            float(result.num_raw_points),          # [7] raw LiDAR points
            float(result.num_clusters),            # [8] num clusters
            1.0 if result.scan_degraded else 0.0,  # [9] degraded flag
        ]
        self._pub_metrics.publish(metrics_msg)

        # 日志（降频）
        if self._control_loop.__dict__.get("_cnt", 0) % 20 == 0:
            self.get_logger().info(
                f"status={result_filter.status:8s} | "
                f"η_H={result_filter.eta_h: .3f} | "
                f"rows={result_filter.num_constraints:2d} | "
                f"min_h={result_filter.min_clearance: .3f} | "
                f"u_safe=({result_filter.u_safe[0]: .3f},{result_filter.u_safe[1]: .3f}) | "
                f"u_nom=({u_nom[0]: .3f},{u_nom[1]: .3f}) | "
                f"scan_age={scan_age:.3f}s | "
                f"clusters={result.num_clusters}"
            )
        self._control_loop.__dict__["_cnt"] = (
            self._control_loop.__dict__.get("_cnt", 0) + 1
        )


def main(args=None):
    rclpy.init(args=args)
    node = CBFSafetyFilterNode()
    # MultiThreadedExecutor: /tf + /scan + /odom + timer 回调并行消费.
    # 单线程下 scan 回调内 lookup_transform 会阻塞 executor 处理 /tf,
    # 导致 TF buffer 数据滞后 → ExtrapolationException.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
