#!/usr/bin/env python3
"""CBF 约束构造器 v2.0 — 从 LiDAR 点云到 finite-row attack-aware hard-safe 约束。

重构要点（与 Python 2D 仿真口径对齐）:
  1. 采用基于 LiDAR 角度顺序的跳变聚类（替换无序 BFS）
  2. 采用 Welzl 精确最小包围圆（替换质心近似）
  3. 补齐多层感知膨胀项（扫描间隙 + 噪声 + TF + 覆盖 + 时间延迟）
  4. 可选线段/胶囊体 primitive 模式
  5. 支持 scan age 安全降级
  6. 多 body disk footprint（ω 可进入 CBF row）
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import math
import numpy as np

from .geometry_utils import (
    laser_to_world,
    laser_to_world_with_meta,
    ordered_lidar_clusters,
    cluster_points,
    minimum_enclosing_circle,
    min_enclosing_circle_approx,
    body_disk_velocity_coeffs,
    body_disk_world_centers,
    closest_point_on_segment,
    segment_distance,
)
from .simple_perception import extract_local_minima, danger_points_world_xy


@dataclass
class LocalObstacle:
    """圆形局部障碍物 primitive。"""
    center: np.ndarray
    radius: float


@dataclass
class LocalSegmentObstacle:
    """线段/胶囊体局部障碍物 primitive。

    表示 LiDAR 可见的一段障碍物表面片段 [a, b] 经 inflation 膨胀后的胶囊体。
    """
    a: np.ndarray        # 线段端点1 [2]
    b: np.ndarray        # 线段端点2 [2]
    inflation: float     # 膨胀半径
    range_val: float = 0.0  # 该线段的近似距离（用于调试）


@dataclass
class CBFConstraintRow:
    """一条 hard-safe CBF 约束：a_v * v + a_w * w >= rhs"""
    a_v: float
    a_w: float
    rhs: float
    h_value: float       # 当前安全函数值
    source_idx: int       # 来源障碍物/disk索引
    disk_idx: int = -1            # 来源 body disk 索引
    primitive_type: str = ""      # "circle" | "segment" | "degraded"
    primitive_idx: int = -1       # 来源 primitive 索引


@dataclass
class InflationParams:
    """多层感知膨胀参数。"""
    eps_scan: float = 0.03     # 角分辨率间隙膨胀 [m]
    eps_noise: float = 0.03    # 测距噪声膨胀 3σ [m]
    eps_tf: float = 0.01       # TF/odom 误差膨胀 [m]
    eps_cover: float = 0.03    # 工程覆盖裕度 [m]
    max_inflation: float = 0.20  # 膨胀上界 [m]


@dataclass
class BuildRowsResult:
    """build_rows() 的统一返回结构。"""
    rows: List[CBFConstraintRow] = field(default_factory=list)
    local_objects_circle: List[LocalObstacle] = field(default_factory=list)
    local_objects_segment: List[LocalSegmentObstacle] = field(default_factory=list)
    min_h: float = float("inf")
    scan_age: float = 0.0
    scan_degraded: bool = False
    num_raw_points: int = 0
    num_clusters: int = 0


class BarrierBuilder:
    """从 /odom + /scan 构造有限行攻击感知 CBF 约束。

    参数（与 Python 仿真一致）:
        body_disks: [(cx, cy, radius), ...] 机器人本体覆盖圆盘
        alpha_h: CBF 类-K 函数系数
        a_v_max, a_w_max: 攻击上界
        v_safe_max, w_safe_max: 𝒰_safe 上界（用于 future horizon buffer）
        dt_control: 控制周期
        active_h_threshold: 只激活 h < threshold 的约束
        cluster_dist: LiDAR 点聚类距离（用于 BFS 模式）
        min_object_radius: 最小局部障碍物半径
        d_geom: 几何安全裕度 (固定)
        d_delay: 延迟裕度
        d_hold: ZOH hold 裕度
        d_meas: 测量误差裕度
        max_constraint_rows: 最大保留 CBF 行数
        use_ordered_clustering: 是否使用角度顺序聚类
        use_segment_primitive: 是否使用线段 primitive（否则用 Welzl 圆）
        max_scan_age: 最大允许 scan 年龄 [s]
    """

    DEFAULT_BODY_DISKS = [
        (0.08, 0.0, 0.105),    # 前盘 — 匹配 TB3 真实 footprint
        (0.0, 0.0, 0.105),     # 中心盘
        (-0.08, 0.0, 0.105),   # 后盘
    ]

    def __init__(
        self,
        body_disks: Optional[List[Tuple[float, float, float]]] = None,
        alpha_h: float = 1.6,
        a_v_max: float = 0.225,
        a_w_max: float = 0.375,
        v_safe_max: float = 0.775,
        w_safe_max: float = 1.225,
        dt_control: float = 0.10,
        active_h_threshold: float = 0.50,
        cluster_dist: float = 0.20,
        min_object_radius: float = 0.05,
        d_geom: float = 0.02,
        d_delay: float = 0.01,
        d_hold: float = 0.01,
        d_meas: float = 0.015,
        future_horizon_scale: float = 3.0,
        max_constraint_rows: int = 32,
        # ── 新增参数 ──
        use_ordered_clustering: bool = True,
        use_segment_primitive: bool = True,
        max_scan_age: float = 0.50,
        inflation: Optional[InflationParams] = None,
        # ── 简化模型: 局部极小值点障碍物 ──
        use_local_minima: bool = False,
        tau0: float = 0.04,
        k_theta: float = 2.0,
        k_sigma: float = 3.0,
        sigma_r: float = 0.01,
        # 每个 disk 只考虑距离自己最近的 K 个点障碍物
        nearest_obstacles_per_disk: int = 2,
        # ── E10: generic norm-bounded disturbance model ──
        disturbance_model: str = "box",   # "box" (attack-aware) | "l2_norm" (generic)
        s_v: float = 0.22,                # normalized DWA v scale [m/s]
        s_w: float = 1.0,                 # normalized DWA w scale [rad/s]
        rho_G: float = 0.848528137423857, # generic L2 norm bound (0.6*sqrt(2))
    ):
        self.body_disks = body_disks or self.DEFAULT_BODY_DISKS
        self.alpha_h = alpha_h
        self.a_v_max = a_v_max
        self.a_w_max = a_w_max
        self.v_safe_max = v_safe_max
        self.w_safe_max = w_safe_max
        self.dt_control = dt_control
        self.active_h_threshold = active_h_threshold
        self.cluster_dist = cluster_dist
        self.min_object_radius = min_object_radius
        self.d_geom = d_geom
        self.d_delay = d_delay
        self.d_hold = d_hold
        self.d_meas = d_meas
        self.future_horizon_scale = future_horizon_scale
        self.max_constraint_rows = max_constraint_rows

        # 新增
        self.use_ordered_clustering = use_ordered_clustering
        self.use_segment_primitive = use_segment_primitive
        # 简化模型
        self.use_local_minima = use_local_minima
        self.tau0 = tau0
        self.k_theta = k_theta
        self.k_sigma = k_sigma
        self.sigma_r = sigma_r
        self.nearest_obstacles_per_disk = nearest_obstacles_per_disk
        self.max_scan_age = max_scan_age
        self.inflation = inflation or InflationParams()

        # E10 generic disturbance model
        self.disturbance_model = disturbance_model
        self.s_v = float(s_v)
        self.s_w = float(s_w)
        self.rho_G = float(rho_G)

        # 内部状态
        self._last_scan_stamp: Optional[float] = None

    @property
    def total_margin(self) -> float:
        """CBF 安全裕度（不含感知膨胀项——感知膨胀已在障碍物 radius 中处理）。"""
        return self.d_geom + self.d_delay + self.d_hold

    def _robust_penalty(self, coeff_v: float, coeff_w: float) -> float:
        """robust disturbance support penalty for a row.

        disturbance_model="box" (frozen attack-aware, E0-E9):
            Delta = |coeff_v| * a_v_max + |coeff_w| * a_w_max
            (attack-specific channel-wise box support)

        disturbance_model="l2_norm" (generic norm-bounded, E10):
            normalized sensitivity q = [s_v*coeff_v, s_w*coeff_w]
            Delta = rho_G * ||q||_2
                  = rho_G * sqrt((s_v*coeff_v)^2 + (s_w*coeff_w)^2)
            (isotropic L2 ball support covering the actual normalized box)
        """
        if self.disturbance_model == "l2_norm":
            qv = self.s_v * coeff_v
            qw = self.s_w * coeff_w
            return self.rho_G * math.sqrt(qv * qv + qw * qw)
        # default "box": attack-aware channel-wise box support (frozen behavior)
        return abs(coeff_v) * self.a_v_max + abs(coeff_w) * self.a_w_max

    # ─── 感知膨胀计算 ─────────────────────────────

    def _compute_obstacle_inflation(
        self,
        range_val: float,
        angle_increment: float,
        scan_age: float,
        disk_offset_norm: float = 0.0,
    ) -> float:
        """计算单一障碍物 primitive 的总膨胀半径。

        膨胀项组成:
            ε_scan  : LiDAR 角分辨率间隙 = 0.5·r·Δθ
            ε_noise : 测距噪声 3σ
            ε_tf    : TF/odom 误差
            ε_cover : 工程覆盖裕度
            ε_time  : 时间延迟膨胀 = v_disk_max·(scan_age + dt_control)

        Returns:
            总膨胀半径，不超过 max_inflation
        """
        p = self.inflation
        eps_scan = 0.5 * range_val * angle_increment
        eps_noise = p.eps_noise
        eps_tf = p.eps_tf
        eps_cover = p.eps_cover

        v_disk_max = self.v_safe_max + self.a_v_max + (self.w_safe_max + self.a_w_max) * disk_offset_norm
        eps_time = v_disk_max * (scan_age + self.dt_control)

        eps = eps_scan + eps_noise + eps_tf + eps_cover + eps_time
        return min(eps, p.max_inflation)

    # ─── Scan age 管理 ─────────────────────────────

    def register_scan_stamp(self, stamp_sec: float) -> None:
        """记录最近一次 scan 的时间戳（ROS sim time 秒）。"""
        self._last_scan_stamp = float(stamp_sec)

    def _get_scan_age(self, current_time_sec: Optional[float] = None) -> float:
        """获取当前 scan 年龄 [s]。

        Args:
            current_time_sec: 当前 ROS 时间（sim time 或 wall time），秒。
                             必须与 register_scan_stamp 使用同一时钟源。
                             若为 None，返回 0.0（不触发降级）。
        """
        if self._last_scan_stamp is None:
            return 0.0
        if current_time_sec is None:
            return 0.0
        return max(0.0, float(current_time_sec) - self._last_scan_stamp)

    def _is_scan_degraded(self, current_time_sec: Optional[float] = None) -> bool:
        """判断 scan 是否过期（需要降级处理）。"""
        return self._get_scan_age(current_time_sec) > self.max_scan_age

    # ─── 主入口 ───────────────────────────────────

    def build_rows(
        self,
        state: np.ndarray,
        ranges: np.ndarray,
        angles: np.ndarray,
        range_max: float,
        angle_increment: Optional[float] = None,
        current_time_sec: Optional[float] = None,
    ) -> Tuple[List[CBFConstraintRow], List[LocalObstacle], float]:
        """从状态 + LiDAR 构造当前 CBF 约束行。

        返回 (rows, local_objects, min_h) —— 保持原有接口兼容。
        local_objects 现在合并 circle 和 segment primitives。
        """
        if angle_increment is None:
            angle_increment = 2.0 * math.pi / float(max(len(ranges), 1))

        result = self.build_rows_v2(state, ranges, angles, range_max,
                                     angle_increment, current_time_sec)
        # 合并所有障碍物 primitive 用于调试/可视化
        all_objects = list(result.local_objects_circle)
        return result.rows, all_objects, result.min_h

    def build_rows_v2(
        self,
        state: np.ndarray,
        ranges: np.ndarray,
        angles: np.ndarray,
        range_max: float,
        angle_increment: float,
        current_time_sec: Optional[float] = None,
    ) -> BuildRowsResult:
        """完整版 CBF 约束构造（推荐使用）。

        Returns:
            BuildRowsResult 包含所有中间结果和诊断信息。
        """
        scan_age = self._get_scan_age(current_time_sec)
        scan_degraded = self._is_scan_degraded(current_time_sec)

        # ── Step 0: scan 严重过期 → 安全降级 ──
        if scan_degraded:
            rows: List[CBFConstraintRow] = []
            safe_v = max(0.0, self.v_safe_max - 0.3 * scan_age)
            safe_w = max(0.0, self.w_safe_max - 0.5 * scan_age)
            # 双向速度约束：|v| ≤ safe_v, |ω| ≤ safe_w
            #   v ≥ -safe_v  →   a_v= 1.0, rhs=-safe_v
            #   v ≤  safe_v  →   a_v=-1.0, rhs=-safe_v  (即 -v ≥ -safe_v)
            rows.append(CBFConstraintRow(
                a_v=1.0, a_w=0.0, rhs=-safe_v,
                h_value=-1.0, source_idx=-1, disk_idx=-1,
                primitive_type="degraded", primitive_idx=-1,
            ))
            rows.append(CBFConstraintRow(
                a_v=-1.0, a_w=0.0, rhs=-safe_v,
                h_value=-1.0, source_idx=-2, disk_idx=-1,
                primitive_type="degraded", primitive_idx=-1,
            ))
            rows.append(CBFConstraintRow(
                a_v=0.0, a_w=1.0, rhs=-safe_w,
                h_value=-1.0, source_idx=-3, disk_idx=-1,
                primitive_type="degraded", primitive_idx=-1,
            ))
            rows.append(CBFConstraintRow(
                a_v=0.0, a_w=-1.0, rhs=-safe_w,
                h_value=-1.0, source_idx=-4, disk_idx=-1,
                primitive_type="degraded", primitive_idx=-1,
            ))
            return BuildRowsResult(
                rows=rows, min_h=-1.0,
                scan_age=scan_age, scan_degraded=True,
            )

        # ── 简化模型入口: 局部极小值点障碍物 ──
        if self.use_local_minima:
            return self.build_rows_local_minima(
                state, ranges, angles, range_max,
                angle_increment, current_time_sec,
            )

        # ── Step 1: LiDAR 有效点提取 + 坐标变换 ──
        valid = (ranges > 0.01) & (ranges < range_max - 1e-3)
        if not np.any(valid):
            return BuildRowsResult(scan_age=scan_age)

        r_valid = ranges[valid]
        a_valid = angles[valid]

        # body-frame 坐标
        x_body = r_valid * np.cos(a_valid)
        y_body = r_valid * np.sin(a_valid)
        pts_body = np.stack([x_body, y_body], axis=1)

        # world-frame 坐标
        px, py, theta = float(state[0]), float(state[1]), float(state[2])
        R_mat = np.array([[math.cos(theta), -math.sin(theta)],
                          [math.sin(theta), math.cos(theta)]], dtype=float)
        pts_world = (R_mat @ pts_body.T).T + np.array([px, py])

        num_raw_points = len(pts_world)

        # ── Step 2: 聚类 ──
        if self.use_ordered_clustering:
            clusters_body = ordered_lidar_clusters(
                ranges, angles, range_max, angle_increment,
                min_cluster_size=3, merge_first_last=True,
            )
            # 转换 cluster 到 world frame
            clusters_world = []
            for clu_body in clusters_body:
                clu_world = (R_mat @ clu_body.T).T + np.array([px, py])
                clusters_world.append(clu_world)
        else:
            # 回退到无序 BFS
            clusters_world = cluster_points(pts_world, self.cluster_dist, min_cluster_size=3)

        num_clusters = len(clusters_world)

        # ── Step 3: 构造局部障碍物 primitive ──
        local_objects_circle: List[LocalObstacle] = []
        local_objects_segment: List[LocalSegmentObstacle] = []

        # 最大 body disk offset（用于感知膨胀中的 angular motion 贡献）
        disk_offset_max = max(math.hypot(cx, cy) for cx, cy, _ in self.body_disks)

        for clu_w, clu_body in zip(
            clusters_world,
            clusters_body if self.use_ordered_clustering else [None] * len(clusters_world)
        ):
            # 计算 cluster 的代表性 range（用于膨胀计算）
            if clu_body is not None:
                clu_ranges = np.sqrt(np.sum(clu_body ** 2, axis=1))
                rep_range = float(np.median(clu_ranges))
            else:
                clu_center = np.mean(clu_w, axis=0)
                rep_range = float(np.linalg.norm(clu_center - np.array([px, py])))

            # 障碍物膨胀（使用最大 disk offset 保守估计 angular motion 贡献）
            eps_obs = self._compute_obstacle_inflation(
                rep_range, angle_increment, scan_age,
                disk_offset_norm=disk_offset_max,
            )

            if self.use_segment_primitive and len(clu_w) >= 2:
                # 线段/胶囊体模式：对 cluster 内相邻点生成线段 primitive
                stride = max(1, len(clu_w) // 12)  # 动态步长控制 primitive 数量
                for k in range(0, len(clu_w) - stride, max(1, stride // 2)):
                    seg_a = clu_w[k]
                    seg_b = clu_w[min(k + stride, len(clu_w) - 1)]
                    seg_len = float(np.linalg.norm(seg_b - seg_a))
                    if seg_len > 1e-4:
                        local_objects_segment.append(LocalSegmentObstacle(
                            a=seg_a.copy(), b=seg_b.copy(),
                            inflation=eps_obs,
                            range_val=rep_range,
                        ))
            else:
                # 圆形模式：Welzl 精确最小包围圆
                center, exact_radius = minimum_enclosing_circle(clu_w)
                inflated_radius = max(
                    self.min_object_radius + self.d_meas,
                    exact_radius + eps_obs + self.d_meas,
                )
                local_objects_circle.append(LocalObstacle(
                    center=center, radius=inflated_radius,
                ))

        # ── Step 4: 每个 body disk × 每个 obstacle primitive → ALL CBF rows ──
        #  先生成所有 rows，再用分组保留做 pruning（而非简单阈值截断）
        disk_centers = body_disk_world_centers(state, self.body_disks)
        delta = self.total_margin

        v_act_max = self.v_safe_max + self.a_v_max
        w_act_max = self.w_safe_max + self.a_w_max
        future_horizon = self.future_horizon_scale * (self.dt_control + 0.03)

        all_rows: List[CBFConstraintRow] = []
        all_h_values: List[float] = []
        rows_by_disk = {j: [] for j in range(len(self.body_disks))}
        rows_by_obstacle: dict = {}

        # ── 处理圆形 obstacles ──
        for obj_idx, obj in enumerate(local_objects_circle):
            obs_key = f"circle_{obj_idx}"
            rows_by_obstacle[obs_key] = []
            o = obj.center
            for disk_idx, ((_, _, rho_j), p_j) in enumerate(
                zip(self.body_disks, disk_centers)
            ):
                d_vec = p_j - o
                dist = float(np.linalg.norm(d_vec))
                if dist < 1e-9:
                    unit = np.array([1.0, 0.0], dtype=float)
                else:
                    unit = d_vec / dist

                h_ij = dist - (obj.radius + rho_j + delta)
                all_h_values.append(h_ij)

                g_v, g_w = body_disk_velocity_coeffs(state, self.body_disks, disk_idx)
                coeff_v = float(np.dot(unit, g_v))
                coeff_w = float(np.dot(unit, g_w))

                # future buffer for scoring
                future_buffer = future_horizon * (
                    abs(coeff_v) * v_act_max + abs(coeff_w) * w_act_max
                )
                future_buffer = max(future_buffer, 0.02)

                rhs = self.alpha_h * h_ij - self._robust_penalty(coeff_v, coeff_w)

                row = CBFConstraintRow(
                    a_v=coeff_v, a_w=coeff_w, rhs=rhs, h_value=h_ij,
                    source_idx=obj_idx * len(self.body_disks) + disk_idx,
                    disk_idx=disk_idx, primitive_type="circle",
                    primitive_idx=obj_idx,
                )
                all_rows.append(row)
                rows_by_disk[disk_idx].append(row)
                rows_by_obstacle[obs_key].append(row)

        # ── 处理线段 obstacles ──
        seg_offset = len(local_objects_circle) * len(self.body_disks)
        for seg_idx, seg in enumerate(local_objects_segment):
            obs_key = f"seg_{seg_idx}"
            rows_by_obstacle[obs_key] = []
            for disk_idx, ((_, _, rho_j), p_j) in enumerate(
                zip(self.body_disks, disk_centers)
            ):
                c_obs = closest_point_on_segment(p_j, seg.a, seg.b)
                d_vec = p_j - c_obs
                dist = float(np.linalg.norm(d_vec))

                safe_radius = rho_j + seg.inflation + delta
                h_ij = dist - safe_radius
                all_h_values.append(h_ij)

                g_v, g_w = body_disk_velocity_coeffs(state, self.body_disks, disk_idx)

                if dist < 1e-9:
                    coeff_v = 1.0
                    coeff_w = 0.0
                else:
                    unit = d_vec / dist
                    coeff_v = float(np.dot(unit, g_v))
                    coeff_w = float(np.dot(unit, g_w))

                future_buffer = future_horizon * (
                    abs(coeff_v) * v_act_max + abs(coeff_w) * w_act_max
                )
                future_buffer = max(future_buffer, 0.02)

                rhs = self.alpha_h * h_ij - self._robust_penalty(coeff_v, coeff_w)

                row = CBFConstraintRow(
                    a_v=coeff_v, a_w=coeff_w, rhs=rhs, h_value=h_ij,
                    source_idx=seg_offset + seg_idx * len(self.body_disks) + disk_idx,
                    disk_idx=disk_idx, primitive_type="segment",
                    primitive_idx=seg_idx,
                )
                all_rows.append(row)
                rows_by_disk[disk_idx].append(row)
                rows_by_obstacle[obs_key].append(row)

        # ── Step 5: 分组保留 pruning（替代简单阈值截断）──
        KEEP_PER_DISK = 3
        KEEP_PER_OBSTACLE = 2
        h_threshold = 1.25  # 原始 active_h_threshold

        keep_ids = set()  # 使用 id() 因为 CBFConstraintRow 不可哈希

        # (a) 每个 body disk 保留最近的 K rows
        for disk_idx, disk_rows in rows_by_disk.items():
            disk_rows_sorted = sorted(disk_rows, key=lambda r: r.h_value)
            for r in disk_rows_sorted[:KEEP_PER_DISK]:
                keep_ids.add(id(r))

        # (b) 每个 obstacle 保留最近的 K rows
        for obs_key, obs_rows in rows_by_obstacle.items():
            obs_rows_sorted = sorted(obs_rows, key=lambda r: r.h_value)
            for r in obs_rows_sorted[:KEEP_PER_OBSTACLE]:
                keep_ids.add(id(r))

        # (c) 保留所有 h < h_threshold 的行
        for r in all_rows:
            if r.h_value < h_threshold:
                keep_ids.add(id(r))

        # (d) 排序 + 截断到 max_constraint_rows
        rows = sorted(
            [r for r in all_rows if id(r) in keep_ids],
            key=lambda r: r.h_value
        )[:self.max_constraint_rows]

        min_h = float(min(all_h_values)) if all_h_values else float("inf")

        return BuildRowsResult(
            rows=rows,
            local_objects_circle=local_objects_circle,
            local_objects_segment=local_objects_segment,
            min_h=min_h,
            scan_age=scan_age,
            scan_degraded=scan_degraded,
            num_raw_points=num_raw_points,
            num_clusters=num_clusters,
        )

    # ═══════════════════════════════════════════════════════════
    # 简化模型: 分段连续数据 → 局部极小值 → 点障碍物 → CBF 行
    # ═══════════════════════════════════════════════════════════

    def build_rows_local_minima(
        self, state, ranges, angles, range_max, angle_increment,
        current_time_sec=None,
    ) -> BuildRowsResult:
        """简化模型主入口。

        360束距离数据分段连续: 每个障碍物对应一段连续数据,
        每一段存在一个局部极小值 = 该障碍物离机器人最近的点。
        把极小值位置当作该障碍物的碰撞检测代表点(点障碍物),
        每个点障碍物 × body disk → 一条 CBF 行。
        """
        scan_age = self._get_scan_age(current_time_sec)
        scan_degraded = self._is_scan_degraded(current_time_sec)
        valid_mask = (ranges > 0.01) & (ranges < range_max - 1e-3)
        num_raw_points = int(np.count_nonzero(valid_mask))

        # 局部极小值提取 (simple_perception 唯一规范实现)
        minima = extract_local_minima(
            ranges, angles, range_max,
            tau0=self.tau0, k_theta=self.k_theta,
            k_sigma=self.k_sigma, sigma_r=self.sigma_r,
            max_points=self.max_constraint_rows,
        )
        if num_raw_points == 0 or len(minima) == 0:
            return BuildRowsResult(
                rows=[], min_h=float("inf"), scan_age=scan_age,
                scan_degraded=scan_degraded, num_raw_points=num_raw_points,
                num_clusters=len(minima))

        px, py, theta = float(state[0]), float(state[1]), float(state[2])
        R_mat = np.array([[math.cos(theta), -math.sin(theta)],
                          [math.sin(theta), math.cos(theta)]], dtype=float)

        # 每个极小值 → 一个点障碍物。
        # 点模型膨胀: 极小值点已经是该障碍物的真实最近点(亚束插值),
        # 膨胀只需覆盖测距噪声 + 束宽足迹, 不需要为面障碍设计的
        # eps_scan/eps_tf/eps_cover/eps_time 堆栈(那会双重计入并造成
        # 0.24m 级缓冲 → SF 在正常导航间隙 0.24m 时持续干预)。
        point_radius = self.d_meas + 0.01
        point_obstacles = []
        for mp in minima:
            r_i = mp.range
            a_i = mp.theta
            body = np.array([r_i * math.cos(a_i), r_i * math.sin(a_i)])
            world = R_mat @ body + np.array([px, py])
            point_obstacles.append(LocalObstacle(center=world, radius=point_radius))

        disk_centers = body_disk_world_centers(state, self.body_disks)
        delta = self.total_margin
        K = max(1, int(self.nearest_obstacles_per_disk))
        all_rows = []
        all_h_values = []
        # 每个 disk 只考虑距离自己最近的 K 个点障碍物
        for disk_idx, ((_, _, rho_j), p_j) in enumerate(
                zip(self.body_disks, disk_centers)):
            # 计算该 disk 到每个点障碍物的距离, 排序取最近 K 个
            candidates = []
            for obj_idx, obj in enumerate(point_obstacles):
                d_vec = p_j - obj.center
                dist = float(np.linalg.norm(d_vec))
                candidates.append((dist, obj_idx, obj, d_vec))
            candidates.sort(key=lambda c: c[0])

            for dist, obj_idx, obj, d_vec in candidates[:K]:
                if dist < 1e-9:
                    unit = np.array([1.0, 0.0], dtype=float)
                else:
                    unit = d_vec / dist
                h_ij = dist - (obj.radius + rho_j + delta)
                all_h_values.append(h_ij)
                g_v, g_w = body_disk_velocity_coeffs(state, self.body_disks, disk_idx)
                coeff_v = float(np.dot(unit, g_v))
                coeff_w = float(np.dot(unit, g_w))
                rhs = self.alpha_h * h_ij - self._robust_penalty(coeff_v, coeff_w)
                all_rows.append(CBFConstraintRow(
                    a_v=coeff_v, a_w=coeff_w, rhs=rhs, h_value=h_ij,
                    source_idx=obj_idx * len(self.body_disks) + disk_idx,
                    disk_idx=disk_idx, primitive_type="local_min",
                    primitive_idx=obj_idx))
        rows = sorted(all_rows, key=lambda r: r.h_value)[:self.max_constraint_rows]
        min_h = float(min(all_h_values)) if all_h_values else float("inf")
        return BuildRowsResult(
            rows=rows, local_objects_circle=point_obstacles,
            local_objects_segment=[], min_h=min_h, scan_age=scan_age,
            scan_degraded=scan_degraded, num_raw_points=num_raw_points,
            num_clusters=len(minima))
