#!/usr/bin/env python3
"""
简化感知模型 — LiDAR 分段连续数据的局部极小值提取。

核心思想:
  360° LiDAR 距离数据是"分段连续"的:
    - 同一障碍物表面 → 相邻射线距离连续平缓变化 → 构成一段
    - 不同障碍物之间 → 距离跳变 → 分段
    - 每一段(每个障碍物)的极小值 = 该障碍物离机器人最近的点

  算法两步:
    Step 1: 沿角度顺序扫描, 相邻射线欧氏距离跳变处切段
    Step 2: 每段内取 argmin (段内全局最小, 天然就是该段极小值),
            并用抛物线插值细化到亚束精度

  把这些极小值位置当作障碍物代表点(危险点),
  用于碰撞检测 / CBF 约束 / 攻击评分, 取代原聚类+primitive 模型。
"""

from dataclasses import dataclass
from typing import List, Tuple
import math
import numpy as np


@dataclass
class DangerPoint:
    """一个障碍物段的局部极小值代表点。"""
    idx: int            # 原始 scan 索引 (argmin)
    theta: float        # 插值细化后的射线角度 [rad] (body 系)
    range: float        # 插值细化后的距离 [m]
    seg_start: int      # 所属段的起始索引
    seg_end: int        # 所属段的结束索引 (含)


def _segment_by_jump(
    r: np.ndarray,
    a: np.ndarray,
    angle_increment: float,
    *,
    tau0: float = 0.04,
    k_theta: float = 2.0,
    k_sigma: float = 3.0,
    sigma_r: float = 0.01,
    min_segment_size: int = 3,
) -> List[Tuple[int, int]]:
    """Step 1: 跳变检测分段 (循环数组, 处理跨 0° 的障碍物)。

    相邻两束射线的 body 系欧氏距离超过自适应阈值 → 切段。
    阈值: τ_m = τ₀ + k_θ·max(r_m, r_{m-1})·Δθ + k_σ·σ_r
    """
    n = len(r)
    idx_seq = [i for i in range(n) if np.isfinite(r[i])]
    if len(idx_seq) < min_segment_size:
        return []

    segments: List[List[int]] = []
    current: List[int] = [idx_seq[0]]
    prev_i = idx_seq[0]
    for i in idx_seq[1:]:
        d = math.hypot(
            r[i] * math.cos(a[i]) - r[prev_i] * math.cos(a[prev_i]),
            r[i] * math.sin(a[i]) - r[prev_i] * math.sin(a[prev_i]),
        )
        r_max_pair = max(r[i], r[prev_i])
        threshold = tau0 + k_theta * r_max_pair * angle_increment + k_sigma * sigma_r
        if d > threshold:
            segments.append(current)
            current = [i]
        else:
            current.append(i)
        prev_i = i
    segments.append(current)

    # 跨 0° 合并: 首段起点与末段终点距离足够近 → 同一障碍物
    if len(segments) >= 2:
        first, last = segments[0], segments[-1]
        i0, i1 = first[0], last[-1]
        d = math.hypot(
            r[i0] * math.cos(a[i0]) - r[i1] * math.cos(a[i1]),
            r[i0] * math.sin(a[i0]) - r[i1] * math.sin(a[i1]),
        )
        r_max_pair = max(r[i0], r[i1])
        threshold = tau0 + k_theta * r_max_pair * angle_increment + k_sigma * sigma_r
        if d <= threshold:
            segments = segments[1:-1] + [last + first]

    # 注意: 保留 (first_idx, last_idx) 顺序语义。
    # start <= end → 正常段 [start, end]
    # start > end  → 跨 0° 环绕段 [start, n-1] + [0, end]
    out: List[Tuple[int, int]] = []
    for seg in segments:
        if len(seg) >= min_segment_size:
            out.append((seg[0], seg[-1]))
    return out


def _parabolic_refine(r: np.ndarray, a: np.ndarray, i: int) -> Tuple[float, float]:
    """Step 2b: 对 argmin 索引 i 做三点抛物线插值, 细化到亚束精度。

    拟合 (θ_{i-1}, r_{i-1}), (θ_i, r_i), (θ_{i+1}, r_{i+1}) 三点抛物线,
    顶点即该障碍物精确的最近点。退化时退回原索引。
    """
    n = len(r)
    im, ip = (i - 1) % n, (i + 1) % n
    r0, r1, r2 = float(r[im]), float(r[i]), float(r[ip])
    t0, t1, t2 = float(a[im]), float(a[i]), float(a[ip])

    # 角度环绕展开到 t1 附近
    def _unwrap(t: float, ref: float) -> float:
        while t - ref > math.pi:
            t -= 2.0 * math.pi
        while t - ref < -math.pi:
            t += 2.0 * math.pi
        return t

    t0 = _unwrap(t0, t1)
    t2 = _unwrap(t2, t1)

    # 三点拟合抛物线, 顶点角度
    num = (r2 - r0) * (t1 - t0) ** 2 + (r0 - r1) * (t2 - t0) ** 2
    den = (r2 - r1) * (t1 - t0) + (r0 - r1) * (t2 - t0)
    if abs(den) < 1e-12:
        return float(a[i]), float(r[i])

    t_star = t1 - 0.5 * num / den
    if not (min(t0, t2) <= t_star <= max(t0, t2)):
        return float(a[i]), float(r[i])

    # 顶点处距离: 直接解线性方程组重算抛物线值
    A = np.array([
        [t0 * t0, t0, 1.0],
        [t1 * t1, t1, 1.0],
        [t2 * t2, t2, 1.0],
    ], dtype=float)
    b = np.array([r0, r1, r2], dtype=float)
    try:
        c2, c1, c0 = np.linalg.solve(A, b)
        r_star = float(c2 * t_star * t_star + c1 * t_star + c0)
    except np.linalg.LinAlgError:
        r_star = r1

    # 安全钳制: 细化值不得比原 argmin 更远
    if r_star > r1:
        return float(a[i]), float(r[i])

    theta_out = (t_star + math.pi) % (2.0 * math.pi) - math.pi
    return theta_out, max(r_star, 0.0)


def extract_local_minima(
    ranges: np.ndarray,
    angles: np.ndarray,
    range_max: float,
    *,
    tau0: float = 0.04,
    k_theta: float = 2.0,
    k_sigma: float = 3.0,
    sigma_r: float = 0.01,
    min_segment_size: int = 3,
    max_points: int = 40,
) -> List[DangerPoint]:
    """从 360° LiDAR 原始距离数据提取局部极小值代表点。

    分段连续模型: 先按跳变切段(每段=一个障碍物),
    再取每段内 argmin = 该障碍物离机器人最近的点,
    抛物线插值细化到亚束精度。

    Args:
        ranges: [N] 原始距离数据
        angles: [N] 对应角度 (rad)
        range_max: 量程，超出视为无效 (inf)
        tau0: 切段基础跳变阈值 [m]
        k_theta: 角分辨率间隙系数
        k_sigma: 噪声 sigma 系数
        sigma_r: 测距噪声标准差 [m]
        min_segment_size: 最小段长
        max_points: 最多返回的代表点数

    Returns:
        按距离升序排列的 DangerPoint 列表
    """
    n = len(ranges)
    if n < 3:
        return []

    r = np.asarray(ranges, dtype=float)
    # 无效点(太近/超量程/inf) → inf
    r = np.where((r > 0.05) & (r < range_max - 1e-3) & np.isfinite(r), r, np.inf)
    a = np.asarray(angles, dtype=float)
    angle_increment = float(np.mean(np.diff(a))) if n > 1 else 0.01745

    # ── Step 1: 跳变分段 ──
    segments = _segment_by_jump(
        r, a, angle_increment,
        tau0=tau0, k_theta=k_theta, k_sigma=k_sigma, sigma_r=sigma_r,
        min_segment_size=min_segment_size,
    )
    if not segments:
        return []

    # ── Step 2: 每段 argmin + 抛物线细化 ──
    points: List[DangerPoint] = []
    for seg_start, seg_end in segments:
        if seg_end >= seg_start:
            seg_idx = list(range(seg_start, seg_end + 1))
        else:
            seg_idx = list(range(seg_start, n)) + list(range(0, seg_end + 1))
        if not seg_idx:
            continue
        seg_r = r[seg_idx]
        finite_mask = np.isfinite(seg_r)
        if not np.any(finite_mask):
            continue
        i_local = int(np.argmin(np.where(finite_mask, seg_r, np.inf)))
        i = seg_idx[i_local]
        theta_ref, range_ref = _parabolic_refine(r, a, i)
        points.append(DangerPoint(
            idx=i, theta=float(theta_ref), range=float(range_ref),
            seg_start=seg_start, seg_end=seg_end,
        ))

    # 距离升序, 截断
    points.sort(key=lambda p: p.range)
    return points[:max_points]


def danger_points_body_xy(points: List[DangerPoint]) -> np.ndarray:
    """DangerPoint 列表 → body 系坐标 [M, 2]。"""
    if not points:
        return np.zeros((0, 2), dtype=float)
    return np.array([
        [p.range * math.cos(p.theta), p.range * math.sin(p.theta)]
        for p in points
    ], dtype=float)


def danger_points_world_xy(
    points: List[DangerPoint],
    state: np.ndarray,
) -> np.ndarray:
    """DangerPoint 列表 → 世界系坐标 [M, 2]。"""
    body = danger_points_body_xy(points)
    if body.shape[0] == 0:
        return body
    x, y, theta = float(state[0]), float(state[1]), float(state[2])
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]], dtype=float)
    return (R @ body.T).T + np.array([x, y])
