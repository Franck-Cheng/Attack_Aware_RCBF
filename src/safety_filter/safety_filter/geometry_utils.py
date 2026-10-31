#!/usr/bin/env python3
"""几何工具函数 — 坐标变换、聚类、最小包围圆、线段工具。

v2.0: 新增基于 LiDAR 角度顺序的跳变聚类、Welzl 精确最小包围圆、
      线段/胶囊体障碍物 primitive 支持。
"""

import math
import numpy as np
from typing import List, Tuple, Optional


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rotmat(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


# ============================================================
# 坐标变换
# ============================================================

def laser_to_world(
    ranges: np.ndarray,
    angles: np.ndarray,
    state: np.ndarray,
    range_max: float,
) -> np.ndarray:
    """将 LiDAR scan 点转为世界坐标系。

    Args:
        ranges: [N] range values
        angles: [N] corresponding angles in LiDAR frame
        state: [x, y, theta] robot pose in world frame
        range_max: max valid range

    Returns:
        [M, 2] valid world-frame points
    """
    valid = (ranges > 0.01) & (ranges < range_max)
    r = ranges[valid]
    a = angles[valid]

    x_body = r * np.cos(a)
    y_body = r * np.sin(a)

    px, py, theta = float(state[0]), float(state[1]), float(state[2])
    R = rotmat(theta)

    pts_body = np.stack([x_body, y_body], axis=1)  # [M, 2]
    pts_world = (R @ pts_body.T).T + np.array([px, py])

    return pts_world


def laser_to_world_with_meta(
    ranges: np.ndarray,
    angles: np.ndarray,
    state: np.ndarray,
    range_max: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """将 LiDAR scan 点转为世界坐标系，同时返回 body-frame 坐标和 range 值。

    Returns:
        pts_world: [M, 2] world-frame points
        pts_body:  [M, 2] body-frame points
        r_valid:   [M]    range values
    """
    valid = (ranges > 0.01) & (ranges < range_max)
    r = ranges[valid]
    a = angles[valid]

    x_body = r * np.cos(a)
    y_body = r * np.sin(a)

    px, py, theta = float(state[0]), float(state[1]), float(state[2])
    R = rotmat(theta)

    pts_body = np.stack([x_body, y_body], axis=1)
    pts_world = (R @ pts_body.T).T + np.array([px, py])

    return pts_world, pts_body, r


# ============================================================
# 聚类算法
# ============================================================

def cluster_points(
    pts: np.ndarray,
    cluster_dist: float,
    min_cluster_size: int = 3,
) -> List[np.ndarray]:
    """（保留）简单无序 BFS 欧氏距离聚类。"""
    if len(pts) == 0:
        return []

    clusters = []
    visited = np.zeros(len(pts), dtype=bool)

    for i in range(len(pts)):
        if visited[i]:
            continue
        cluster = [pts[i]]
        visited[i] = True
        queue = [i]
        while queue:
            cur = queue.pop(0)
            for j in range(len(pts)):
                if visited[j]:
                    continue
                if np.linalg.norm(pts[cur] - pts[j]) <= cluster_dist:
                    visited[j] = True
                    cluster.append(pts[j])
                    queue.append(j)
        if len(cluster) >= min_cluster_size:
            clusters.append(np.array(cluster))

    return clusters


def ordered_lidar_clusters(
    ranges: np.ndarray,
    angles: np.ndarray,
    range_max: float,
    angle_increment: float,
    *,
    tau0: float = 0.04,
    k_theta: float = 2.0,
    k_sigma: float = 3.0,
    sigma_r: float = 0.01,
    min_cluster_size: int = 3,
    merge_first_last: bool = True,
) -> List[np.ndarray]:
    """基于 LiDAR 角度顺序的跳变聚类。

    利用 LiDAR 扫描点按角度递增排列的性质，通过相邻射线间的
    欧氏距离跳变来分割不同障碍物表面。

    跳变阈值（距离自适应）:
        τ_m = τ₀ + k_θ·max(r_m, r_{m-1})·Δθ + k_σ·σ_r

    其中 Δθ 为 LiDAR 角分辨率（约 0.01745 rad for 360 samples）。

    Args:
        ranges: [N] raw range values
        angles: [N] corresponding angles
        range_max: max valid range
        angle_increment: LiDAR angular resolution
        tau0: 基础跳变阈值 [m]
        k_theta: 角度间距系数
        k_sigma: 噪声 sigma 系数
        sigma_r: 测距噪声标准差 [m]
        min_cluster_size: 最小 cluster 点数
        merge_first_last: 是否合并首尾 cluster（跨越0°的障碍物）

    Returns:
        List of [N_k, 2] body-frame point arrays per cluster.
        Caller is responsible for transforming to world-frame if needed.
    """
    # 过滤有效点
    valid = (ranges > 0.01) & (ranges < range_max - 1e-3)
    if not np.any(valid):
        return []

    r_valid = ranges[valid]
    a_valid = angles[valid]

    # 计算 body-frame 坐标
    x_body = r_valid * np.cos(a_valid)
    y_body = r_valid * np.sin(a_valid)

    n = len(r_valid)
    if n < min_cluster_size:
        return []

    # 按顺序扫描，通过跳变分割
    clusters_indices = []
    current_start = 0

    for i in range(1, n):
        # 相邻两点的 body-frame 欧氏距离
        dx = x_body[i] - x_body[i - 1]
        dy = y_body[i] - y_body[i - 1]
        gap = math.hypot(dx, dy)

        # 自适应跳变阈值
        r_max_pair = max(r_valid[i], r_valid[i - 1])
        threshold = tau0 + k_theta * r_max_pair * angle_increment + k_sigma * sigma_r

        if gap > threshold:
            # 切断
            if i - current_start >= min_cluster_size:
                clusters_indices.append((current_start, i))
            current_start = i

    # 最后一个 cluster
    if n - current_start >= min_cluster_size:
        clusters_indices.append((current_start, n))

    # 首尾合并：检查第一个和最后一个 cluster 是否应合并
    if merge_first_last and len(clusters_indices) >= 2:
        first_start, first_end = clusters_indices[0]
        last_start, last_end = clusters_indices[-1]

        # 计算首尾两点的 body-frame 距离
        dx_fl = x_body[first_start] - x_body[last_end - 1]
        dy_fl = y_body[first_start] - y_body[last_end - 1]
        gap_fl = math.hypot(dx_fl, dy_fl)

        r_max_fl = max(r_valid[first_start], r_valid[last_end - 1])
        threshold_fl = tau0 + k_theta * r_max_fl * angle_increment + k_sigma * sigma_r

        if gap_fl <= threshold_fl:
            # 合并首尾
            merged_start = last_start
            merged_end = first_end
            clusters_indices = clusters_indices[1:-1]
            clusters_indices.insert(0, (merged_start, merged_end))

    # 输出 world-frame 点
    clusters = []
    for start, end in clusters_indices:
        pts_body_cluster = np.stack([x_body[start:end], y_body[start:end]], axis=1)
        clusters.append(pts_body_cluster)

    return clusters


# ============================================================
# 最小包围圆
# ============================================================

def min_enclosing_circle_approx(pts: np.ndarray) -> Tuple[np.ndarray, float]:
    """（保留）近似最小包围圆（质心+最远点法）。"""
    if len(pts) == 1:
        return pts[0].copy(), 0.05
    center = np.mean(pts, axis=0)
    radius = max(np.linalg.norm(p - center) for p in pts)
    return center, max(radius, 0.05)


# ─── Welzl 精确最小包围圆 ─────────────────────────

def _is_in_circle(p: np.ndarray, center: np.ndarray, radius: float, eps: float = 1e-9) -> bool:
    return float(np.linalg.norm(p - center)) <= radius + eps


def _circle_from_two_points(a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, float]:
    center = 0.5 * (a + b)
    radius = 0.5 * float(np.linalg.norm(a - b))
    return center, radius


def _circle_from_three_points(a: np.ndarray, b: np.ndarray, c: np.ndarray):
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    cx, cy = float(c[0]), float(c[1])

    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None

    ux = (
        (ax * ax + ay * ay) * (by - cy)
        + (bx * bx + by * by) * (cy - ay)
        + (cx * cx + cy * cy) * (ay - by)
    ) / d
    uy = (
        (ax * ax + ay * ay) * (cx - bx)
        + (bx * bx + by * by) * (ax - cx)
        + (cx * cx + cy * cy) * (bx - ax)
    ) / d

    center = np.array([ux, uy], dtype=float)
    radius = float(np.linalg.norm(a - center))
    return center, radius


def _smallest_circle_from_boundary(boundary_pts: List[np.ndarray]) -> Tuple[np.ndarray, float]:
    if len(boundary_pts) == 0:
        return np.zeros(2, dtype=float), 0.0
    if len(boundary_pts) == 1:
        return boundary_pts[0].copy(), 0.0
    if len(boundary_pts) == 2:
        return _circle_from_two_points(boundary_pts[0], boundary_pts[1])

    a, b, c = boundary_pts[0], boundary_pts[1], boundary_pts[2]
    circ = _circle_from_three_points(a, b, c)
    if circ is not None:
        center, radius = circ
        if all(_is_in_circle(p, center, radius) for p in [a, b, c]):
            return center, radius

    candidates = []
    for p, q in [(a, b), (a, c), (b, c)]:
        center, radius = _circle_from_two_points(p, q)
        if all(_is_in_circle(x, center, radius) for x in [a, b, c]):
            candidates.append((center, radius))

    if len(candidates) == 0:
        pts_arr = np.array([a, b, c], dtype=float)
        center = np.mean(pts_arr, axis=0)
        radius = float(np.max(np.linalg.norm(pts_arr - center[None, :], axis=1)))
        return center, radius

    best = min(candidates, key=lambda cr: cr[1])
    return best[0], best[1]


def minimum_enclosing_circle(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """Welzl 随机增量算法计算二维点集的精确最小包围圆。

    返回 (center, radius)，保证所有输入点都在圆内。

    与 Python 2D 仿真 (attack_safe_nav/geometry.py) 完全一致。
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return np.zeros(2, dtype=float), 0.0
    if len(pts) == 1:
        return pts[0].copy(), 0.0

    pts_list = [p.copy() for p in pts]
    rng = np.random.default_rng(0)
    rng.shuffle(pts_list)

    center = None
    radius = 0.0

    for i, p in enumerate(pts_list):
        if center is not None and _is_in_circle(p, center, radius):
            continue

        center, radius = p.copy(), 0.0
        for j in range(i):
            q = pts_list[j]
            if _is_in_circle(q, center, radius):
                continue

            center, radius = _circle_from_two_points(p, q)
            for k in range(j):
                r = pts_list[k]
                if _is_in_circle(r, center, radius):
                    continue

                center, radius = _smallest_circle_from_boundary([p, q, r])

    return center, float(radius)


# ============================================================
# 线段/胶囊体工具
# ============================================================

def closest_point_on_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """计算点 p 到线段 [a, b] 的最近点投影。

    Args:
        p: 查询点 [2]
        a, b: 线段端点 [2]

    Returns:
        线段上的最近点 [2]
    """
    ab = b - a
    denom = float(np.dot(ab, ab))

    if denom < 1e-12:
        return a.copy()

    t = float(np.dot(p - a, ab) / denom)
    t = max(0.0, min(1.0, t))

    return a + t * ab


def segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """点 p 到线段 [a, b] 的最短距离。"""
    c = closest_point_on_segment(p, a, b)
    return float(np.linalg.norm(p - c))


# ============================================================
# 机器人运动学系数
# ============================================================

def body_disk_velocity_coeffs(
    state: np.ndarray,
    body_disks: List[Tuple[float, float, float]],
    disk_idx: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """计算第 disk_idx 个 body disk 的速度系数 g_v, g_w。

    g_v: 线速度方向向量 (在世界坐标系中对 disk 中心速度的贡献)
    g_w: 角速度贡献向量
    """
    theta = float(state[2])
    cx, cy, _ = body_disks[disk_idx]
    c = np.array([cx, cy], dtype=float)
    R = rotmat(theta)

    g_v = np.array([math.cos(theta), math.sin(theta)], dtype=float)
    Jc = np.array([-cy, cx], dtype=float)
    g_w = R @ Jc
    return g_v, g_w


def body_disk_world_centers(
    state: np.ndarray,
    body_disks: List[Tuple[float, float, float]],
) -> np.ndarray:
    """计算所有 body disk 的世界坐标中心。"""
    p = np.array(state[:2], dtype=float)
    theta = float(state[2])
    R = rotmat(theta)
    centers = []
    for cx, cy, _ in body_disks:
        centers.append(p + R @ np.array([cx, cy], dtype=float))
    return np.array(centers)
