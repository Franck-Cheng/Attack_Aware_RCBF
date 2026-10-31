#!/usr/bin/env python3
"""
执行器注入攻击节点。

攻击模式:
  - zero:                   无攻击 (a=0)
  - constant:               恒定偏置 a = ρ·[a_v_max, a_w_max]
  - random:                 随机符号 a = ρ·[±a_v_max, ±a_w_max]
  - min_h:                  定向攻击最危险CBF行（单行符号攻击）
  - front_crash:            恒定正向攻击
  - rollout_min_h:          多步rollout枚举min_h（使用CBF行评分）
  - rollout_min_clearance:  多步rollout几何最小clearance攻击 ★推荐

rollout_min_clearance 核心思路:
  枚举候选攻击 → 标定后运动模型rollout H步 →
  计算未来H步中body footprint到障碍物的最小clearance →
  选择让未来最小clearance最小的攻击

  使用经阶跃响应标定的有效角速度模型:
    v_eff = v_cmd                              (跟踪比≈1.0)
    w_eff = clip(κ_ω·w_cmd, ±ω_eff_max)        (κ_ω≈0.55, ω_eff_max≈0.83)

  攻击footprint使用大偏移前后盘（无中心盘）以避免中心盘对角速度不敏感。
  支持条件触发: 仅在clearance < trigger_clearance时攻击。
"""
import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# 复用 safety_filter 的 barrier_builder
from safety_filter.barrier_builder import BarrierBuilder
from safety_filter.simple_perception import (
    extract_local_minima, danger_points_world_xy,
)


class ActuatorAttackNode(Node):
    def __init__(self):
        super().__init__('actuator_attack_node')

        self.declare_parameter('rho', 1.0)
        self.declare_parameter('a_v_max', 0.05)
        self.declare_parameter('a_w_max', 0.30)
        self.declare_parameter('attack_mode', 'constant')
        # zero / constant / random / min_h / front_crash / rollout_min_h / rollout_min_clearance

        self.rho = self.get_parameter('rho').value
        self.a_v_max = self.get_parameter('a_v_max').value
        self.a_w_max = self.get_parameter('a_w_max').value
        self.attack_mode = self.get_parameter('attack_mode').value

        # ── rollout 通用参数 ──
        self.declare_parameter('rollout_horizon', 24)
        self.declare_parameter('rollout_dt', 0.05)
        self.declare_parameter('rollout_margin', 0.03)
        # 角速度物理层跟踪比（阶跃响应标定：w_odom/w_cmd ≈ 0.554）
        self.declare_parameter('omega_eff_gain', 0.55)
        # 角速度有效饱和值（阶跃响应标定）
        self.declare_parameter('omega_eff_limit', 0.83)
        # 攻击触发 clearance 阈值（仅在 min_clearance < 此值时攻击）
        self.declare_parameter('trigger_clearance', 0.45)
        # 候选攻击网格粒度
        self.declare_parameter('candidate_levels', 5)  # 5→5×5=25, 7→7×7=49

        self.rollout_horizon = 24
        self.rollout_dt = 0.05
        self.rollout_margin = 0.03
        self.omega_eff_gain = 0.55
        self.omega_eff_limit = 0.83
        self.trigger_clearance = 0.45
        self.candidate_levels = 5

        # ── 攻击专用 footprint（无中心盘，大偏移使ω可充分作用）──
        # 默认安全过滤器使用: [(0.05,0,0.10),(0,0,0.11),(-0.05,0,0.10)]
        # 攻击器不使用中心盘，改用大偏移前后盘以暴露偏航碰撞趋势
        self.attack_body_disks = [
            (0.14, 0.0, 0.12),
            (-0.14, 0.0, 0.12),
        ]

        # ── Subscribers ──────────────────────────────
        self.sub = self.create_subscription(
            Twist, '/cmd_vel_safe', self.cmd_safe_callback, 10
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 读取 rollout 参数
        self.rollout_horizon = self.get_parameter('rollout_horizon').value
        self.rollout_dt = self.get_parameter('rollout_dt').value
        self.rollout_margin = self.get_parameter('rollout_margin').value
        self.omega_eff_gain = self.get_parameter('omega_eff_gain').value
        self.omega_eff_limit = self.get_parameter('omega_eff_limit').value
        self.trigger_clearance = self.get_parameter('trigger_clearance').value
        self.candidate_levels = self.get_parameter('candidate_levels').value

        # min_h 攻击需要感知信息
        self._sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, sensor_qos
        )
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10
        )

        # ── Publishers ───────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_attack = self.create_publisher(Twist, '/attack_signal', 10)

        # ── 状态缓存 ─────────────────────────────────
        self._state = np.zeros(3)
        self._state_valid = False
        self._latest_ranges = np.zeros(0)
        self._latest_angles = np.zeros(0)
        self._range_max = 3.5

        # ── min_h 攻击用的 BarrierBuilder ────────────
        self._barrier_builder = BarrierBuilder(
            alpha_h=1.6,
            a_v_max=self.a_v_max,
            a_w_max=self.a_w_max,
            use_ordered_clustering=True,
            max_constraint_rows=32,
        )

        self._angle_increment = 0.01745  # 默认 2π/360
        self._scan_stamp_valid = False

        self.get_logger().info('Actuator attack node v3.0 started.')
        self.get_logger().info(f'  mode={self.attack_mode}, rho={self.rho}')
        self.get_logger().info(f'  a_v_max={self.a_v_max:.3f}, a_w_max={self.a_w_max:.3f}')
        self.get_logger().info(f'  attack_disks={self.attack_body_disks}')
        self.get_logger().info(f'  rollout: H={self.rollout_horizon}, dt={self.rollout_dt}')
        self.get_logger().info(f'  ω_eff: gain={self.omega_eff_gain}, limit={self.omega_eff_limit}')
        self.get_logger().info(f'  trigger_clearance={self.trigger_clearance}')
        self.get_logger().info('  Subscribe: /cmd_vel_safe, /odom, /scan')
        self.get_logger().info('  Publish:   /cmd_vel, /attack_signal')

    def _odom_cb(self, msg: Odometry):
        self._state[0] = msg.pose.pose.position.x
        self._state[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._state[2] = math.atan2(siny, cosy)
        self._state_valid = True

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        self._latest_ranges = np.array(msg.ranges, dtype=float)
        self._latest_angles = (
            msg.angle_min + np.arange(n) * msg.angle_increment
        )
        self._range_max = float(msg.range_max)
        self._angle_increment = float(msg.angle_increment)

        # 记录 scan 时间戳
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._barrier_builder.register_scan_stamp(stamp_sec)
        self._scan_stamp_valid = True

    # ── min_h attack logic ────────────────────────
    def _get_min_h_attack(self) -> tuple:
        """定向攻击最危险CBF行: a = -ρ·a_max·sign(ℓ_critical)

        使用与 safety_filter 相同的 barrier_builder（角度顺序聚类 + Welzl 精确圆），
        确保攻击瞄准的约束行与安全过滤器处理的一致。
        """
        if not self._state_valid or len(self._latest_ranges) == 0:
            return (
                self.rho * self.a_v_max * random.choice([-1.0, 1.0]),
                self.rho * self.a_w_max * random.choice([-1.0, 1.0]),
            )

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        rows, _, _ = self._barrier_builder.build_rows(
            self._state,
            self._latest_ranges,
            self._latest_angles,
            self._range_max,
            self._angle_increment,
            current_time_sec=now_sec,
        )

        if len(rows) == 0:
            return (
                self.rho * self.a_v_max * random.choice([-1.0, 1.0]),
                self.rho * self.a_w_max * random.choice([-1.0, 1.0]),
            )

        critical = min(rows, key=lambda r: r.h_value)

        a_v = 0.0
        a_w = 0.0

        if abs(critical.a_v) > 1e-12:
            a_v = -self.rho * self.a_v_max * math.copysign(1.0, critical.a_v)

        if abs(critical.a_w) > 1e-12:
            a_w = -self.rho * self.a_w_max * math.copysign(1.0, critical.a_w)

        # 诊断：打印最危险的5个CBF行
        top5 = sorted(rows, key=lambda r: r.h_value)[:5]
        self.get_logger().debug(
            f'min_h top5: ' + ' | '.join(
                f'h={r.h_value:.3f} a_v={r.a_v:.3f} a_w={r.a_w:.3f}'
                for r in top5
            )
        )
        has_w = any(abs(r.a_w) > 0.01 for r in rows)
        if not has_w:
            self.get_logger().debug(
                f'All {len(rows)} CBF rows have a_w≈0 (obstacles directly ahead, '
                f'center disk g_w=[0,0]). ω-attack needs side obstacles.'
            )

        return a_v, a_w

    # ── front_crash: 恒定正向攻击 ─────────────────
    def _get_front_crash_attack(self) -> tuple:
        return self.rho * self.a_v_max, 0.0

    # ── rollout_min_h: 多步几何最坏攻击 ──────────
    def _get_scan_points_world(self):
        """简化感知模型: 360束分段连续数据 → 每段局部极小值 = 障碍物代表点。

        把每个障碍物的局部极小值位置当作碰撞检测位置(危险点)，
        取代原"全部有效 LiDAR 点"模型。
        """
        if not self._state_valid or len(self._latest_ranges) == 0:
            return np.zeros((0, 2))
        minima = extract_local_minima(
            self._latest_ranges, self._latest_angles, self._range_max,
            max_points=self.candidate_levels * self.candidate_levels,
        )
        return danger_points_world_xy(minima, self._state)

    def _disk_centers_for_state(self, state):
        x, y, th = float(state[0]), float(state[1]), float(state[2])
        c, s = math.cos(th), math.sin(th)
        centers = []
        for dx, dy, rr in self.attack_body_disks:
            wx = x + c*dx - s*dy; wy = y + s*dx + c*dy
            centers.append((wx, wy, rr))
        return centers

    def _min_clearance(self, state, pts_world):
        if pts_world.shape[0] == 0: return 1e6
        min_h = 1e6
        for cx, cy, rr in self._disk_centers_for_state(state):
            d = np.sqrt((pts_world[:,0]-cx)**2 + (pts_world[:,1]-cy)**2)
            h = float(np.min(d - rr - self.rollout_margin))
            if h < min_h: min_h = h
        return min_h

    def _rollout_score(self, u_v, u_w, a_v, a_w, pts_world):
        """多步 rollout，使用有效角速度模型（阶跃响应标定）。

        v_eff = v_cmd                              (跟踪比≈1.0)
        w_eff = clip(κ_ω·w_cmd, ±ω_eff_limit)      (κ_ω≈0.55, limit≈0.83)
        """
        state = self._state.copy().astype(float)
        worst_h = 1e6
        v_cmd = u_v + a_v
        w_cmd = u_w + a_w
        v_eff = v_cmd  # 线速度通道无损
        w_eff = max(-self.omega_eff_limit, min(self.omega_eff_limit,
                                                self.omega_eff_gain * w_cmd))
        for _ in range(self.rollout_horizon):
            th = float(state[2])
            state[0] += self.rollout_dt * v_eff * math.cos(th)
            state[1] += self.rollout_dt * v_eff * math.sin(th)
            state[2] += self.rollout_dt * w_eff
            state[2] = math.atan2(math.sin(state[2]), math.cos(state[2]))
            h = self._min_clearance(state, pts_world)
            if h < worst_h:
                worst_h = h
        return worst_h

    # ── rollout 候选攻击生成器 ──────────────────
    def _candidate_grid(self):
        """生成候选攻击网格。n=5 → 25候选, n=7 → 49候选。"""
        n = self.candidate_levels
        if n == 5:
            levels = [-1.0, -0.5, 0.0, 0.5, 1.0]
        elif n == 7:
            levels = [-1.0, -0.67, -0.33, 0.0, 0.33, 0.67, 1.0]
        else:
            levels = np.linspace(-1.0, 1.0, n).tolist()
        return levels

    def _get_rollout_min_h_attack(self, msg) -> tuple:
        """rollout_min_h: 枚举候选攻击，用 _rollout_score 评分。"""
        if not self._state_valid or len(self._latest_ranges) == 0:
            return (self.rho * self.a_v_max * random.choice([-1, 1]),
                    self.rho * self.a_w_max * random.choice([-1, 1]))
        pts = self._get_scan_points_world()
        if pts.shape[0] == 0:
            return (0.0, 0.0)
        u_v, u_w = float(msg.linear.x), float(msg.angular.z)
        levels = self._candidate_grid()
        best_a, best_s = (0.0, 0.0), 1e9
        for sv in levels:
            for sw in levels:
                av_c = self.rho * self.a_v_max * sv
                aw_c = self.rho * self.a_w_max * sw
                s = self._rollout_score(u_v, u_w, av_c, aw_c, pts)
                if s < best_s:
                    best_s = s
                    best_a = (av_c, aw_c)
        self.get_logger().info(
            f'rollout_min_h: best_h={best_s:.3f} '
            f'u_nom=({u_v:.3f},{u_w:.3f}) a=({best_a[0]:.3f},{best_a[1]:.3f})')
        return best_a

    # ── rollout_min_clearance: 触发式几何最小clearance攻击 ──
    def _get_rollout_min_clearance_attack(self, msg) -> tuple:
        """rollout_min_clearance — 几何最坏攻击。

        枚举候选攻击 → 标定运动模型rollout H步 →
        计算未来H步中attack_body_disks到LiDAR点的最小clearance →
        选择让未来最小clearance最小的攻击。

        支持条件触发: 仅当当前min_clearance < trigger_clearance时攻击。
        """
        pts_world = self._get_scan_points_world()

        if pts_world.shape[0] == 0:
            return (0.0, 0.0)

        u_v = float(msg.linear.x)
        u_w = float(msg.angular.z)

        # ── 条件触发：仅在危险区域攻击 ──
        current_h = self._min_clearance(self._state, pts_world)
        if current_h > self.trigger_clearance:
            self.get_logger().debug(
                f'rollout_min_clearance: SKIP (current_h={current_h:.3f} > '
                f'trigger={self.trigger_clearance:.3f})')
            return (0.0, 0.0)

        levels = self._candidate_grid()
        best_a = (0.0, 0.0)
        best_score = 1e9

        for sv in levels:
            for sw in levels:
                a_v = self.rho * self.a_v_max * sv
                a_w = self.rho * self.a_w_max * sw

                score = self._rollout_score(u_v, u_w, a_v, a_w, pts_world)

                if score < best_score:
                    best_score = score
                    best_a = (a_v, a_w)

        self.get_logger().info(
            f'rollout_min_clearance: '
            f'cur_h={current_h:.3f} pred_h={best_score:.3f} '
            f'u_nom=({u_v:.3f},{u_w:.3f}) '
            f'a=({best_a[0]:.3f},{best_a[1]:.3f}) '
            f'{"COLLISION_PREDICTED" if best_score < 0 else "safe"}')

        return best_a

    def get_attack(self) -> tuple:
        if self.attack_mode == 'zero':
            return 0.0, 0.0

        if self.attack_mode == 'constant':
            return self.rho * self.a_v_max, self.rho * self.a_w_max

        if self.attack_mode == 'random':
            a_v = self.rho * self.a_v_max * random.choice([-1.0, 1.0])
            a_w = self.rho * self.a_w_max * random.choice([-1.0, 1.0])
            return a_v, a_w

        if self.attack_mode == 'min_h':
            return self._get_min_h_attack()

        if self.attack_mode == 'front_crash':
            return self._get_front_crash_attack()

        return 0.0, 0.0

    def cmd_safe_callback(self, msg):
        try:
            if self.attack_mode == 'rollout_min_h':
                a_v, a_w = self._get_rollout_min_h_attack(msg)
            elif self.attack_mode == 'rollout_min_clearance':
                a_v, a_w = self._get_rollout_min_clearance_attack(msg)
            else:
                a_v, a_w = self.get_attack()

            cmd_act = Twist()
            cmd_act.linear.x = msg.linear.x + a_v
            cmd_act.angular.z = msg.angular.z + a_w

            attack_msg = Twist()
            attack_msg.linear.x = a_v
            attack_msg.angular.z = a_w

            self.pub_cmd.publish(cmd_act)
            self.pub_attack.publish(attack_msg)

            cnt = getattr(self, '_cb_cnt', 0) + 1
            self._cb_cnt = cnt
            if cnt <= 5 or cnt % 100 == 0:
                self.get_logger().info(
                    f'[#{cnt}] u_nom=({msg.linear.x:.3f}, {msg.angular.z:.3f}), '
                    f'a=({a_v:.3f}, {a_w:.3f}), '
                    f'u_act=({cmd_act.linear.x:.3f}, {cmd_act.angular.z:.3f})'
                )
        except Exception as e:
            err_cnt = getattr(self, '_err_cnt', 0) + 1
            self._err_cnt = err_cnt
            if err_cnt <= 5:
                self.get_logger().error(f'Callback error (#{err_cnt}): {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorAttackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
