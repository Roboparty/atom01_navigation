#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD to PGM Converter (2.5D Navigation Support)
将3D激光点云(.pcd)转换为支持2.5D导航的占用栅格地图(.pgm)及配套yaml文件，
支持上下台阶和有一定坡度的斜坡导航。

原理:
    1. 构建高程图：记录每个栅格的地面高度
    2. 计算梯度：分析相邻栅格间的高度变化率
    3. 基于梯度判断可通行性：
       - 平地/缓坡（梯度小）: 可通行
       - 台阶/斜坡（梯度中等，且高度差在机器人能力范围内）: 可通行
       - 垂直障碍物（梯度大或高度差超限）: 不可通行

用法:
    python3 pcd2pgm.py --input /path/to/map.pcd --output /path/to/output_dir --resolution 0.05

参数:
    --input (-i): 输入PCD文件路径
    --output (-o): 输出目录（默认与输入文件同目录）
    --resolution (-r): 栅格分辨率(米/像素)，默认0.05
    --ground_height: 地面参考高度(米)，默认0.0
    --ground_percentile: 地面估计使用的分位数(0-100)，默认90
    --robot_height: 机器人高度(米)，用于判断头顶障碍，默认1.5
    --clearance_margin: 净空安全冗余(米)，默认0.05
    --max_step_height: 机器人能跨越的最大台阶高度(米)，默认0.15
    --max_slope_angle: 机器人能爬的最大坡度角度(度)，默认30
    --smooth_kernel: 高程图平滑核大小(奇数)，默认3
    --occupied_thresh: 占用阈值，默认0.65
    --free_thresh: 空闲阈值，默认0.196
    --map_name: 地图名称，默认为输入文件名
    --inflate: 障碍物膨胀像素数，默认0（不膨胀）
    --z_min: 点云最小高度过滤(米)，低于此高度的点会被过滤
    --z_max: 点云最大高度过滤(米)，高于此高度的点会被过滤（用于去除天花板）
"""

import argparse
import os
import sys
import numpy as np
from collections import defaultdict

try:
    import open3d as o3d
except ImportError:
    print("Error: open3d not installed. Install via: pip3 install open3d")
    sys.exit(1)

try:
    from PIL import Image
except ImportError:
    print("Error: Pillow not installed. Install via: pip3 install Pillow")
    sys.exit(1)

try:
    from scipy.ndimage import binary_dilation, uniform_filter
except ImportError:
    print("Warning: scipy not installed. Some features may not work. Install via: pip3 install scipy")
    binary_dilation = None
    uniform_filter = None


def load_pcd(pcd_path: str, z_min: float = None, z_max: float = None) -> np.ndarray:
    """加载PCD文件并返回点云坐标数组 (N, 3)，可选高度过滤"""
    if not os.path.exists(pcd_path):
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")
    
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    
    if points.shape[0] == 0:
        raise ValueError("PCD file contains no points")
    
    original_count = points.shape[0]
    print(f"Loaded {original_count} points from {pcd_path}")
    
    # 打印点云高度范围，帮助用户了解数据
    z_values = points[:, 2]
    print(f"Point cloud Z range: [{z_values.min():.3f}, {z_values.max():.3f}] m")
    
    # 高度过滤（去除地面以下和天花板）
    if z_min is not None or z_max is not None:
        mask = np.ones(points.shape[0], dtype=bool)
        if z_min is not None:
            mask &= (points[:, 2] >= z_min)
            print(f"Filtering points below z_min={z_min}m")
        if z_max is not None:
            mask &= (points[:, 2] <= z_max)
            print(f"Filtering points above z_max={z_max}m (removing ceiling)")
        
        points = points[mask]
        filtered_count = original_count - points.shape[0]
        print(f"Filtered {filtered_count} points ({100*filtered_count/original_count:.1f}%), remaining: {points.shape[0]}")
        
        if points.shape[0] == 0:
            raise ValueError("All points were filtered out! Check z_min and z_max parameters.")
    
    return points


def save_filtered_pcd(points: np.ndarray, output_path: str):
    """保存过滤后的点云为PCD文件（用于调试）"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(output_path, pcd)
    print(f"Saved filtered point cloud to: {output_path} ({points.shape[0]} points)")


def save_elevation_pcd(elevation_map: np.ndarray, origin_x: float, origin_y: float, 
                       resolution: float, output_path: str):
    """
    将高程图转换为点云并保存为PCD文件（用于调试）
    每个有效栅格生成一个点，高度为该栅格的高程值
    """
    height, width = elevation_map.shape
    valid_mask = ~np.isnan(elevation_map)
    
    # 获取有效栅格的索引
    valid_indices = np.where(valid_mask)
    py_arr = valid_indices[0]
    px_arr = valid_indices[1]
    
    # 计算世界坐标
    x_coords = origin_x + (px_arr + 0.5) * resolution
    y_coords = origin_y + (py_arr + 0.5) * resolution
    z_coords = elevation_map[valid_mask]
    
    # 组合成点云
    points = np.column_stack([x_coords, y_coords, z_coords])
    
    # 创建Open3D点云并保存
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 根据高度添加颜色（低=蓝色，高=红色）
    z_min, z_max = z_coords.min(), z_coords.max()
    if z_max - z_min > 0.001:
        normalized = (z_coords - z_min) / (z_max - z_min)
    else:
        normalized = np.zeros_like(z_coords)
    
    # 颜色映射：蓝(低) -> 绿(中) -> 红(高)
    colors = np.zeros((len(normalized), 3))
    colors[:, 0] = normalized  # R: 高度越高越红
    colors[:, 1] = 1 - np.abs(normalized - 0.5) * 2  # G: 中间高度最绿
    colors[:, 2] = 1 - normalized  # B: 高度越低越蓝
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.io.write_point_cloud(output_path, pcd)
    print(f"Saved elevation map as point cloud to: {output_path}")
    print(f"  - Points: {len(points)}")
    print(f"  - Z range: [{z_min:.3f}, {z_max:.3f}] m")


def save_traversability_pcd(traversability: np.ndarray, elevation_map: np.ndarray,
                            origin_x: float, origin_y: float, resolution: float,
                            output_path: str):
    """
    将可通行性地图转换为带颜色的点云并保存为PCD文件（用于调试）
    - 绿色：可通行 (free)
    - 红色：障碍物 (occupied)  
    - 灰色：未知 (unknown)
    """
    height, width = traversability.shape
    valid_mask = ~np.isnan(elevation_map)
    
    # 获取有效栅格的索引
    valid_indices = np.where(valid_mask)
    py_arr = valid_indices[0]
    px_arr = valid_indices[1]
    
    # 计算世界坐标
    x_coords = origin_x + (px_arr + 0.5) * resolution
    y_coords = origin_y + (py_arr + 0.5) * resolution
    z_coords = elevation_map[valid_mask]
    
    # 获取可通行性值
    trav_values = traversability[valid_mask]
    
    # 组合成点云
    points = np.column_stack([x_coords, y_coords, z_coords])
    
    # 创建Open3D点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 根据可通行性添加颜色
    colors = np.zeros((len(trav_values), 3))
    colors[trav_values == 0] = [0, 1, 0]      # 绿色：可通行
    colors[trav_values == 1] = [1, 0, 0]      # 红色：障碍物
    colors[trav_values == 2] = [0.5, 0.5, 0.5]  # 灰色：未知
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.io.write_point_cloud(output_path, pcd)
    
    free_count = np.sum(trav_values == 0)
    occupied_count = np.sum(trav_values == 1)
    unknown_count = np.sum(trav_values == 2)
    print(f"Saved traversability as point cloud to: {output_path}")
    print(f"  - Green (free): {free_count}")
    print(f"  - Red (occupied): {occupied_count}")
    print(f"  - Gray (unknown): {unknown_count}")


def build_elevation_map(points: np.ndarray, resolution: float,
                       robot_height: float, clearance_margin: float = 0.05,
                       ground_percentile: float = 10.0, smooth_kernel: int = 3) -> tuple:
    """
    构建高程图，记录每个栅格的地面高度和上方障碍物信息
    
    返回: (elevation_map, obstacle_map, origin_x, origin_y, width, height)
    - elevation_map: 每个栅格的地面高度（取最低点作为地面）
    - obstacle_map: 每个栅格在机器人高度范围内是否有障碍物
    """
    if points.shape[0] == 0:
        raise ValueError("No points to convert")
    
    # 计算边界
    x_min, y_min = points[:, 0].min(), points[:, 1].min()
    x_max, y_max = points[:, 0].max(), points[:, 1].max()
    
    # 添加边距
    margin = resolution * 10
    x_min -= margin
    y_min -= margin
    x_max += margin
    y_max += margin
    
    # 计算栅格尺寸
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    
    print(f"Grid size: {width} x {height}, origin: ({x_min:.3f}, {y_min:.3f})")
    
    # 初始化高程图（NaN表示未知区域）
    elevation_map = np.full((height, width), np.nan, dtype=np.float32)
    # 记录每个栅格的最低“上方”点，用于检测垂直净空（避免天花板影响地面可行性）
    min_above_ground_map = np.full((height, width), np.nan, dtype=np.float32)
    # 记录每个栅格的点数，用于后续分析
    point_count_map = np.zeros((height, width), dtype=np.int32)
    
    # 使用向量化操作加速
    px_arr = ((points[:, 0] - x_min) / resolution).astype(np.int32)
    py_arr = ((points[:, 1] - y_min) / resolution).astype(np.int32)
    
    # 过滤有效索引
    valid_mask = (px_arr >= 0) & (px_arr < width) & (py_arr >= 0) & (py_arr < height)
    px_valid = px_arr[valid_mask]
    py_valid = py_arr[valid_mask]
    z_valid = points[valid_mask, 2]
    
    # 按栅格分组处理点云
    cell_points = defaultdict(list)
    for px, py, z in zip(px_valid, py_valid, z_valid):
        cell_points[(py, px)].append(z)
    
    for (py, px), z_list in cell_points.items():
        z_arr = np.array(z_list)
        # 地面高度：使用可配置分位数（避免噪声/离群点）
        ground_z = np.percentile(z_arr, ground_percentile)
        elevation_map[py, px] = ground_z
        # 统计点数
        point_count_map[py, px] = len(z_list)
        # 计算上方最小高度（净空判定），过滤掉紧贴地面的点
        ground_eps = max(0.02, 0.5 * resolution)
        above_mask = z_arr > (ground_z + ground_eps)
        if np.any(above_mask):
            min_above_ground_map[py, px] = np.min(z_arr[above_mask])
    # 平滑高程图（仅对有效区域），降低单帧噪声，接近 elevation_mapping 的栅格滤波
    if uniform_filter is not None and smooth_kernel > 1:
        if smooth_kernel % 2 == 0:
            smooth_kernel += 1
        val_mask = ~np.isnan(elevation_map)
        filled = np.where(val_mask, elevation_map, 0.0)
        sum_filtered = uniform_filter(filled, size=smooth_kernel)
        cnt_filtered = uniform_filter(val_mask.astype(np.float32), size=smooth_kernel)
        smoothed = sum_filtered / np.maximum(cnt_filtered, 1e-6)
        elevation_map = np.where(cnt_filtered > 0, smoothed, np.nan)
    
    # 检测头顶障碍物（在机器人净空范围内的障碍），类似 elevation_mapping 的净空判断
    obstacle_map = np.zeros((height, width), dtype=np.uint8)
    valid_elevation = ~np.isnan(elevation_map)
    
    # 净空高度：上方最低点与地面的差值；若不存在上方点，视为无限净空
    clearance_map = min_above_ground_map - elevation_map
    clearance_map[np.isnan(min_above_ground_map)] = np.inf
    
    required_clearance = robot_height + clearance_margin
    # 只有当净空不足时才标记为障碍（悬空障碍物）
    # 注意：这里不应该影响斜坡和台阶的判断
    obstacle_map[valid_elevation & (clearance_map < required_clearance)] = 1
    
    overhead_count = np.sum(obstacle_map == 1)
    print(f"Built elevation map: valid cells = {np.sum(valid_elevation)}, overhead obstacles = {overhead_count}")
    
    # 打印高程统计，帮助调试
    valid_elevations = elevation_map[valid_elevation]
    if len(valid_elevations) > 0:
        print(f"Elevation stats: min={valid_elevations.min():.3f}m, max={valid_elevations.max():.3f}m, "
              f"mean={valid_elevations.mean():.3f}m, std={valid_elevations.std():.3f}m")
    
    return elevation_map, obstacle_map, point_count_map, x_min, y_min, width, height


def compute_traversability(elevation_map: np.ndarray, obstacle_map: np.ndarray, 
                           point_count_map: np.ndarray, resolution: float,
                           max_step_height: float, max_slope_angle: float,
                           min_point_density: int = 3) -> np.ndarray:
    """
    基于高程图计算可通行性地图（支持2.5D导航：台阶和斜坡）
    
    原理：
    1. 对于每个栅格，计算与所有相邻栅格的高度差
    2. 判断该高度差是否在"可通行范围"内：
       - 台阶：高度差 <= max_step_height（允许离散跳变）
       - 斜坡：坡度角 <= max_slope_angle（连续坡度）
    3. 只有当高度差超过所有允许值时，才标记为障碍
    
    关键区别：
    - 可通行的台阶/斜坡：高度有变化，但在机器人能力范围内
    - 不可通行的障碍物：高度变化超过机器人能力
    
    返回: traversability_map (0=free, 1=occupied, 2=unknown)
    """
    height, width = elevation_map.shape
    
    # 初始化为未知
    traversability = np.full((height, width), 2, dtype=np.uint8)
    
    # 有效区域（有点云数据的地方）
    valid_mask = ~np.isnan(elevation_map)
    
    # 点密度过低的区域视为未知（可能是噪声或边缘）
    low_density_mask = point_count_map < min_point_density
    
    # 计算最大允许的坡度对应的高度差（基于水平距离）
    max_slope_tan = np.tan(np.radians(max_slope_angle))
    
    print(f"Traversability params: max_step={max_step_height:.3f}m, max_slope={max_slope_angle}° (tan={max_slope_tan:.3f})")
    
    # 使用NaN填充边界
    padded = np.pad(elevation_map, 1, mode='constant', constant_values=np.nan)
    
    # 计算与8个邻居的高度差，判断是否可通行
    # 一个栅格可通行的条件：到达所有有效邻居的路径都可通行
    neighbor_traversable_count = np.zeros((height, width), dtype=np.int32)
    neighbor_valid_count = np.zeros((height, width), dtype=np.int32)
    max_height_diff_map = np.zeros((height, width), dtype=np.float32)
    
    for dy in [-1, 0, 1]:
        for dx in [-1, 0, 1]:
            if dy == 0 and dx == 0:
                continue
            
            # 计算到邻居的水平距离
            if dy != 0 and dx != 0:
                horizontal_dist = resolution * np.sqrt(2)
            else:
                horizontal_dist = resolution
            
            # 对于当前距离，基于坡度允许的最大高度差
            max_slope_height_for_dist = horizontal_dist * max_slope_tan
            # 综合台阶高度和坡度，取较大值作为允许的最大高度差
            max_allowed_height_diff = max(max_step_height, max_slope_height_for_dist)
            
            neighbor = padded[1+dy:height+1+dy, 1+dx:width+1+dx]
            
            # 计算高度差（绝对值）
            height_diff = np.abs(elevation_map - neighbor)
            
            # 邻居有效的掩码
            neighbor_valid = ~np.isnan(neighbor)
            neighbor_valid_count += neighbor_valid.astype(np.int32)
            
            # 判断到该邻居是否可通行
            can_traverse_to_neighbor = neighbor_valid & (height_diff <= max_allowed_height_diff)
            neighbor_traversable_count += can_traverse_to_neighbor.astype(np.int32)
            
            # 记录最大高度差（用于调试和障碍物判断）
            height_diff_filled = np.where(neighbor_valid, height_diff, 0)
            max_height_diff_map = np.maximum(max_height_diff_map, height_diff_filled)
    
    # 计算可通行比例：能通行的邻居数 / 有效邻居数
    with np.errstate(divide='ignore', invalid='ignore'):
        traversable_ratio = neighbor_traversable_count / np.maximum(neighbor_valid_count, 1)
    
    # 判断可通行性的条件
    # 1. 有足够的点密度
    # 2. 至少有一些有效邻居
    # 3. 大部分邻居都可通行（比例 > 阈值）
    # 4. 没有头顶障碍
    
    has_enough_density = ~low_density_mask
    has_valid_neighbors = neighbor_valid_count >= 2  # 至少2个有效邻居
    mostly_traversable = traversable_ratio >= 0.5   # 至少50%的邻居可通行
    no_overhead_obstacle = (obstacle_map == 0)
    
    # 可通行区域
    traversable = valid_mask & has_enough_density & has_valid_neighbors & mostly_traversable & no_overhead_obstacle
    
    # 障碍物区域：有效但不可通行
    # 1. 有头顶障碍
    # 2. 或者几乎没有邻居可通行（说明是墙壁边缘或高台）
    is_obstacle = valid_mask & (
        (obstacle_map == 1) |  # 有头顶障碍
        (has_valid_neighbors & (traversable_ratio < 0.3) & (max_height_diff_map > max_step_height))  # 边缘/墙壁
    )
    
    traversability[traversable] = 0  # free
    traversability[is_obstacle] = 1   # occupied
    # 其余保持为2 (unknown)
    
    # 后处理：填充被可通行区域包围的小障碍区域（可能是噪声）
    if uniform_filter is not None:
        # 1. 移除孤立的可通行点
        traversable_float = (traversability == 0).astype(np.float32)
        local_free_ratio = uniform_filter(traversable_float, size=3)
        isolated_free = (traversability == 0) & (local_free_ratio < 0.25)
        traversability[isolated_free] = 2
        
        # 2. 填充被可通行区域包围的孤立障碍（可能是噪声）
        obstacle_float = (traversability == 1).astype(np.float32)
        local_obs_ratio = uniform_filter(obstacle_float, size=5)
        isolated_obstacle = (traversability == 1) & (local_obs_ratio < 0.15)
        # 只有当周围大部分是可通行时才填充
        isolated_obstacle &= (local_free_ratio > 0.6)
        traversability[isolated_obstacle] = 0
    
    free_count = np.sum(traversability == 0)
    occupied_count = np.sum(traversability == 1)
    unknown_count = np.sum(traversability == 2)
    print(f"Traversability: free={free_count}, occupied={occupied_count}, unknown={unknown_count}")
    
    # 打印高度差统计信息（帮助调试）
    valid_height_diff = max_height_diff_map[valid_mask]
    if len(valid_height_diff) > 0:
        print(f"Height diff stats: min={valid_height_diff.min():.3f}, max={valid_height_diff.max():.3f}, "
              f"mean={valid_height_diff.mean():.3f}, median={np.median(valid_height_diff):.3f}")
    
    print("\n[DEBUG] Traversability condition analysis:")
    print(f"  - Cells with enough density: {np.sum(has_enough_density)}")
    print(f"  - Cells with valid neighbors: {np.sum(has_valid_neighbors)}")
    print(f"  - Cells with mostly traversable neighbors: {np.sum(mostly_traversable)}")
    print(f"  - Cells with no overhead obstacle: {np.sum(no_overhead_obstacle)}")

    # 详细统计未知点的原因
    unknown_mask = (traversability == 2)
    unknown_no_density = unknown_mask & low_density_mask
    unknown_no_neighbors = unknown_mask & ~has_valid_neighbors
    unknown_no_traversable = unknown_mask & has_valid_neighbors & ~mostly_traversable
    unknown_has_obstacle = unknown_mask & (obstacle_map == 1)

    print(f"\nUnknown cells breakdown:")
    print(f"  - Low density: {np.sum(unknown_no_density)}")
    print(f"  - No neighbors: {np.sum(unknown_no_neighbors)}")
    print(f"  - No mostly traversable: {np.sum(unknown_no_traversable)}")
    print(f"  - Has overhead obstacle: {np.sum(unknown_has_obstacle)}")
    
    return traversability


def fill_unknown_regions(traversability: np.ndarray, elevation_map: np.ndarray) -> np.ndarray:
    """
    填充未知区域：
    - 被可通行区域包围的小未知区域 -> 可通行
    - 其他未知区域保持未知（Nav2会将其视为可探索区域）
    """
    from scipy.ndimage import binary_fill_holes, label
    
    height, width = traversability.shape
    result = traversability.copy()
    
    # 找到所有未知区域
    unknown_mask = (traversability == 2)
    
    # 标记连通的未知区域
    labeled, num_features = label(unknown_mask)
    
    # 对于小的未知区域（被可通行区域包围），填充为可通行
    for i in range(1, num_features + 1):
        region = (labeled == i)
        region_size = np.sum(region)
        
        # 小区域（面积小于阈值）
        if region_size < 100:  # 可调节阈值
            # 检查周围是否主要是可通行区域
            dilated = binary_dilation(region, iterations=2)
            boundary = dilated & ~region
            if np.sum(boundary) > 0:
                boundary_free_ratio = np.sum(traversability[boundary] == 0) / np.sum(boundary)
                if boundary_free_ratio > 0.7:
                    result[region] = 0  # 填充为可通行
    
    return result


def inflate_obstacles(traversability: np.ndarray, inflate_pixels: int) -> np.ndarray:
    """膨胀障碍物，保持未知区域不变"""
    if inflate_pixels <= 0:
        return traversability
    
    if binary_dilation is None:
        print("Warning: scipy not installed, skipping inflation")
        return traversability
    
    result = traversability.copy()
    
    # 只膨胀障碍物区域（值为1）
    obstacle_mask = (traversability == 1)
    structure = np.ones((2 * inflate_pixels + 1, 2 * inflate_pixels + 1))
    dilated_obstacle = binary_dilation(obstacle_mask, structure=structure)
    
    # 将膨胀后的区域标记为障碍（但不覆盖原有障碍）
    # 注意：只膨胀到可通行区域，不膨胀到未知区域
    result[dilated_obstacle & (traversability == 0)] = 1
    
    print(f"Inflated obstacles by {inflate_pixels} pixels")
    return result


def downsample_traversability(traversability: np.ndarray, elevation_map: np.ndarray,
                               src_resolution: float, dst_resolution: float,
                               src_origin_x: float, src_origin_y: float) -> tuple:
    """
    将高分辨率的可通行性地图降采样到低分辨率
    
    降采样策略：
    - 如果块内有障碍物(1) -> 障碍物
    - 如果块内全是未知(2) -> 未知
    - 如果块内有可通行(0)且无障碍物 -> 可通行
    
    返回: (downsampled_traversability, downsampled_elevation, new_origin_x, new_origin_y, new_width, new_height)
    """
    if dst_resolution <= src_resolution:
        # 不需要降采样
        return traversability, elevation_map, src_origin_x, src_origin_y, traversability.shape[1], traversability.shape[0]
    
    scale = int(np.round(dst_resolution / src_resolution))
    src_height, src_width = traversability.shape
    
    # 计算新尺寸
    dst_height = int(np.ceil(src_height / scale))
    dst_width = int(np.ceil(src_width / scale))
    
    print(f"Downsampling: {src_width}x{src_height} (res={src_resolution}m) -> {dst_width}x{dst_height} (res={dst_resolution}m), scale={scale}")
    
    # 初始化降采样结果
    dst_traversability = np.full((dst_height, dst_width), 2, dtype=np.uint8)  # 默认未知
    dst_elevation = np.full((dst_height, dst_width), np.nan, dtype=np.float32)
    
    for dy in range(dst_height):
        for dx in range(dst_width):
            # 源图像中的块范围
            sy_start = dy * scale
            sy_end = min((dy + 1) * scale, src_height)
            sx_start = dx * scale
            sx_end = min((dx + 1) * scale, src_width)
            
            # 获取块内的值
            block_trav = traversability[sy_start:sy_end, sx_start:sx_end]
            block_elev = elevation_map[sy_start:sy_end, sx_start:sx_end]
            
            # 降采样策略
            has_obstacle = np.any(block_trav == 1)
            has_free = np.any(block_trav == 0)
            
            if has_obstacle:
                dst_traversability[dy, dx] = 1  # 有障碍物就是障碍物
            elif has_free:
                dst_traversability[dy, dx] = 0  # 有可通行就是可通行
            # 否则保持未知(2)
            
            # 高程取有效值的平均
            valid_elev = block_elev[~np.isnan(block_elev)]
            if len(valid_elev) > 0:
                dst_elevation[dy, dx] = np.mean(valid_elev)
    
    # 新的origin保持不变（左下角）
    new_origin_x = src_origin_x
    new_origin_y = src_origin_y
    
    free_count = np.sum(dst_traversability == 0)
    occupied_count = np.sum(dst_traversability == 1)
    unknown_count = np.sum(dst_traversability == 2)
    print(f"After downsampling: free={free_count}, occupied={occupied_count}, unknown={unknown_count}")
    
    return dst_traversability, dst_elevation, new_origin_x, new_origin_y, dst_width, dst_height


def traversability_to_pgm(traversability: np.ndarray, output_path: str):
    """
    将可通行性地图转换为PGM图像
    ROS/Nav2约定: 
        - 0 (黑色): occupied 障碍物
        - 254 (白色): free 可通行
        - 205 (灰色): unknown 未知
    """
    # 翻转Y轴（图像坐标系与地图坐标系Y方向相反）
    grid_flipped = np.flipud(traversability)
    
    # 转换为PGM灰度值
    # 0 (free) -> 254 (白色)
    # 1 (occupied) -> 0 (黑色)
    # 2 (unknown) -> 205 (灰色)
    pgm = np.zeros_like(grid_flipped, dtype=np.uint8)
    pgm[grid_flipped == 0] = 254  # free -> white
    pgm[grid_flipped == 1] = 0    # occupied -> black
    pgm[grid_flipped == 2] = 205  # unknown -> gray
    
    # 保存为PGM
    img = Image.fromarray(pgm, mode='L')
    img.save(output_path)
    print(f"Saved PGM map to: {output_path}")


def save_yaml(yaml_path: str, pgm_filename: str, resolution: float,
              origin_x: float, origin_y: float,
              occupied_thresh: float, free_thresh: float):
    """生成Nav2 map_server兼容的yaml文件"""
    yaml_content = f"""image: {pgm_filename}
mode: trinary
resolution: {resolution}
origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]
negate: 0
occupied_thresh: {occupied_thresh}
free_thresh: {free_thresh}
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"Saved YAML config to: {yaml_path}")


def save_elevation_map(elevation_map: np.ndarray, output_path: str):
    """保存高程图为可视化图像（用于调试）"""
    valid_mask = ~np.isnan(elevation_map)
    if not np.any(valid_mask):
        return
    
    # 归一化到0-255
    z_min = np.nanmin(elevation_map)
    z_max = np.nanmax(elevation_map)
    if z_max - z_min < 0.001:
        return
    
    normalized = (elevation_map - z_min) / (z_max - z_min) * 255
    normalized[~valid_mask] = 128  # 未知区域设为中间灰度
    
    # 翻转Y轴
    normalized = np.flipud(normalized)
    
    img = Image.fromarray(normalized.astype(np.uint8), mode='L')
    img.save(output_path)
    print(f"Saved elevation map visualization to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Convert PCD point cloud to 2.5D traversability map (PGM)")
    parser.add_argument('-i', '--input', type=str, default="/home/zyq/atom01/atom01_navigation/src/robots_localization_ros2/PCD/garden.pcd", help='Input PCD file path')
    parser.add_argument('-o', '--output', type=str, default="/home/zyq/atom01/atom01_navigation/src/nav2_localization_adapter/map/", help='Output directory (default: same as input)')
    parser.add_argument('-r', '--resolution', type=float, default=0.05, help='PGM output resolution in meters/pixel (default: 0.05)')
    parser.add_argument('--elevation_resolution', type=float, default=0.05, help='High resolution for elevation map computation (default: 0.02, should be <= resolution)')
    parser.add_argument('--ground_percentile', type=float, default=100.0, help='Ground estimation percentile (default: 90, lower=more robust to noise)')
    parser.add_argument('--robot_height', type=float, default=1.5, help='Robot height in meters for overhead obstacle detection (default: 1.5)')
    parser.add_argument('--clearance_margin', type=float, default=0.2, help='Safety margin added to robot height for clearance (default: 0.05)')
    parser.add_argument('--max_step_height', type=float, default=0.035, help='Max step height robot can traverse in meters (default: 0.15)')
    parser.add_argument('--max_slope_angle', type=float, default=25.0, help='Max slope angle robot can traverse in degrees (default: 30)')
    parser.add_argument('--smooth_kernel', type=int, default=3, help='Smoothing kernel size for elevation map (default: 3, odd number)')
    parser.add_argument('--min_point_density', type=int, default=1, help='Min points per cell to consider valid (default: 1, lowered for sparse data)')
    parser.add_argument('--occupied_thresh', type=float, default=0.65, help='Occupied threshold for YAML (default: 0.65)')
    parser.add_argument('--free_thresh', type=float, default=0.196, help='Free threshold for YAML (default: 0.196)')
    parser.add_argument('--map_name', type=str, default="garden", help='Map name (default: input filename)')
    parser.add_argument('--inflate', type=int, default=0, help='Obstacle inflation in pixels (default: 0)')
    parser.add_argument('--z_min', type=float, default=-0.1, help='Min Z height filter in meters (filter ground noise)')
    parser.add_argument('--z_max', type=float, default=1.9, help='Max Z height filter in meters (IMPORTANT: use this to remove ceiling!)')
    parser.add_argument('--save_elevation', type=bool, default=True, help='Save elevation map visualization for debugging')
    parser.add_argument('--fill_unknown', type=bool, default=True, help='Fill small unknown regions surrounded by free space')
    parser.add_argument('--save_debug_pcd', type=bool, default=True, help='Save intermediate PCD files for debugging (filtered, elevation, traversability)')
    
    args = parser.parse_args()
    
    # 处理路径
    input_path = os.path.abspath(args.input)
    if args.output is None:
        output_dir = os.path.dirname(input_path)
    else:
        output_dir = os.path.abspath(args.output)
        os.makedirs(output_dir, exist_ok=True)
    
    if args.map_name is None:
        map_name = os.path.splitext(os.path.basename(input_path))[0]
    else:
        map_name = args.map_name
    
    pgm_filename = f"{map_name}.pgm"
    yaml_filename = f"{map_name}.yaml"
    pgm_path = os.path.join(output_dir, pgm_filename)
    yaml_path = os.path.join(output_dir, yaml_filename)
    
    print("=" * 70)
    print("PCD to PGM Converter (2.5D Navigation Support)")
    print("=" * 70)
    print(f"Input PCD:        {input_path}")
    print(f"Output dir:       {output_dir}")
    print(f"Map name:         {map_name}")
    print(f"PGM Resolution:   {args.resolution} m/pixel")
    print(f"Elev Resolution:  {args.elevation_resolution} m/pixel (high-res for computation)")
    print(f"Ground percentile:{args.ground_percentile}")
    print(f"Robot height:     {args.robot_height} m")
    print(f"Clearance margin: {args.clearance_margin} m")
    print(f"Max step height:  {args.max_step_height} m")
    print(f"Max slope angle:  {args.max_slope_angle}°")
    print(f"Smooth kernel:    {args.smooth_kernel}")
    print(f"Min point density:{args.min_point_density}")
    print(f"Inflate:          {args.inflate} pixels")
    print(f"Z filter:         min={args.z_min}, max={args.z_max}")
    print(f"Save debug PCD:   {args.save_debug_pcd}")
    print("=" * 70)
    
    # 1. 加载点云（带高度过滤）
    points = load_pcd(input_path, z_min=args.z_min, z_max=args.z_max)
    
    # 可选：保存过滤后的点云
    if args.save_debug_pcd:
        filtered_pcd_path = os.path.join(output_dir, f"{map_name}_filtered.pcd")
        save_filtered_pcd(points, filtered_pcd_path)
    
    # 2. 构建高程图（使用高分辨率）
    elev_resolution = min(args.elevation_resolution, args.resolution)  # 确保不大于输出分辨率
    print(f"\n[Step 1/6] Building elevation map at high resolution ({elev_resolution}m)...")
    elevation_map, obstacle_map, point_count_map, origin_x, origin_y, width, height = \
        build_elevation_map(points, elev_resolution,
                           args.robot_height, args.clearance_margin,
                           args.ground_percentile, args.smooth_kernel)
    
    # 可选：保存高程图可视化（高分辨率）
    if args.save_elevation:
        elevation_vis_path = os.path.join(output_dir, f"{map_name}_elevation_highres.png")
        save_elevation_map(elevation_map, elevation_vis_path)
    
    # 可选：保存高程图点云（高分辨率）
    if args.save_debug_pcd:
        elevation_pcd_path = os.path.join(output_dir, f"{map_name}_elevation_highres.pcd")
        save_elevation_pcd(elevation_map, origin_x, origin_y, elev_resolution, elevation_pcd_path)
    
    # 3. 计算可通行性（在高分辨率上）
    print(f"\n[Step 2/6] Computing traversability at high resolution ({elev_resolution}m)...")
    traversability = compute_traversability(
        elevation_map, obstacle_map, point_count_map,
        elev_resolution, args.max_step_height, args.max_slope_angle,
        args.min_point_density
    )
    
    # 可选：保存高分辨率可通行性点云
    if args.save_debug_pcd:
        trav_pcd_path = os.path.join(output_dir, f"{map_name}_traversability_highres.pcd")
        save_traversability_pcd(traversability, elevation_map, origin_x, origin_y, 
                                elev_resolution, trav_pcd_path)
    
    # 4. 降采样到PGM分辨率
    print(f"\n[Step 3/6] Downsampling to PGM resolution ({args.resolution}m)...")
    traversability, elevation_map, origin_x, origin_y, width, height = downsample_traversability(
        traversability, elevation_map, elev_resolution, args.resolution, origin_x, origin_y
    )
    
    # 5. 填充小的未知区域（可选，在降采样后）
    if args.fill_unknown and binary_dilation is not None:
        print("\n[Step 4/6] Filling small unknown regions...")
        traversability = fill_unknown_regions(traversability, elevation_map)
    else:
        print("\n[Step 4/6] Skipping unknown region filling...")
    
    # 6. 障碍物膨胀（可选）
    if args.inflate > 0:
        print(f"\n[Step 5/6] Inflating obstacles by {args.inflate} pixels...")
        traversability = inflate_obstacles(traversability, args.inflate)
    else:
        print("\n[Step 5/6] Skipping obstacle inflation...")
    
    # 可选：保存降采样后的可通行性点云
    if args.save_debug_pcd:
        trav_pcd_path = os.path.join(output_dir, f"{map_name}_traversability.pcd")
        save_traversability_pcd(traversability, elevation_map, origin_x, origin_y, 
                                args.resolution, trav_pcd_path)
    
    # 7. 保存PGM和YAML
    print("\n[Step 6/6] Saving map files...")
    traversability_to_pgm(traversability, pgm_path)
    save_yaml(yaml_path, pgm_filename, args.resolution,
              origin_x, origin_y,
              args.occupied_thresh, args.free_thresh)
    
    # 统计信息
    total_cells = width * height
    free_cells = np.sum(traversability == 0)
    occupied_cells = np.sum(traversability == 1)
    unknown_cells = np.sum(traversability == 2)
    
    print("\n" + "=" * 70)
    print("Conversion completed successfully!")
    print("=" * 70)
    print(f"Map files:")
    print(f"  - {pgm_path}")
    print(f"  - {yaml_path}")
    print(f"\nMap statistics:")
    print(f"  - Total cells:    {total_cells}")
    print(f"  - Free (white):   {free_cells} ({100*free_cells/total_cells:.1f}%)")
    print(f"  - Occupied (black):{occupied_cells} ({100*occupied_cells/total_cells:.1f}%)")
    print(f"  - Unknown (gray): {unknown_cells} ({100*unknown_cells/total_cells:.1f}%)")
    print("=" * 70)
    print("\nNote: This map supports 2.5D navigation (stairs and slopes).")
    print(f"  - Stairs up to {args.max_step_height}m are traversable")
    print(f"  - Slopes up to {args.max_slope_angle}° are traversable")
    print("=" * 70)


if __name__ == '__main__':
    main()
