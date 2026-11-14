"""Pure-geometry helpers for the autonomous search stage.

No ROS imports here on purpose, so the logic can be unit-tested on a laptop.

The search waypoints are generated at run time from the occupancy grid that
was built on site during Step 1.  Nothing about the sandbox is hard-coded.
"""

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

Point = Tuple[float, float]
Waypoint = Tuple[float, float, float]  # x, y, yaw


def free_space_distance(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    occupied_threshold: int = 65,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (distance_to_obstacle_m, blocked_mask) for a flat grid.

    Unknown cells (-1) are treated as blocked, so the robot never plans a
    search point outside the area that was actually mapped.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    blocked = (grid >= occupied_threshold) | (grid < 0)
    free_u8 = (~blocked).astype(np.uint8)
    dist = cv2.distanceTransform(free_u8, cv2.DIST_L2, 5) * resolution
    return dist, blocked


def plan_search_waypoints(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_xy: Point,
    start_xy: Point,
    spacing: float = 0.9,
    clearance: float = 0.22,
    min_separation_factor: float = 0.6,
    occupied_threshold: int = 65,
) -> List[Point]:
    """Pick well-spread, wall-safe search points in the area around start.

    1. Distance transform -> keep cells at least ``clearance`` from walls.
    2. Keep only the connected region that contains the start pose.
    3. In every ``spacing`` x ``spacing`` block choose the most open cell.
    4. Drop points that are too close to an already chosen one.
    """
    dist, _ = free_space_distance(
        data, width, height, resolution, occupied_threshold
    )
    safe = dist >= clearance
    if not safe.any():
        return []

    _, labels = cv2.connectedComponents(safe.astype(np.uint8), connectivity=8)

    ox, oy = origin_xy
    sx = int((start_xy[0] - ox) / resolution)
    sy = int((start_xy[1] - oy) / resolution)

    label = 0
    if 0 <= sx < width and 0 <= sy < height:
        label = int(labels[sy, sx])
    if label == 0:
        ys, xs = np.nonzero(safe)
        nearest = int(np.argmin((xs - sx) ** 2 + (ys - sy) ** 2))
        label = int(labels[ys[nearest], xs[nearest]])

    region = labels == label
    stride = max(1, int(round(spacing / resolution)))
    min_sep = spacing * min_separation_factor

    candidates: List[Tuple[float, Point]] = []
    for r0 in range(0, height, stride):
        for c0 in range(0, width, stride):
            reg = region[r0:r0 + stride, c0:c0 + stride]
            if not reg.any():
                continue
            block = np.where(
                reg, dist[r0:r0 + stride, c0:c0 + stride], -1.0
            )
            idx = np.unravel_index(int(np.argmax(block)), block.shape)
            row, col = r0 + int(idx[0]), c0 + int(idx[1])
            candidates.append((
                float(block[idx]),
                (ox + (col + 0.5) * resolution,
                 oy + (row + 0.5) * resolution),
            ))

    # Most open points first, so thinning keeps the safest ones.
    candidates.sort(key=lambda item: -item[0])
    chosen: List[Point] = []
    for _, point in candidates:
        if all(math.dist(point, other) >= min_sep for other in chosen):
            chosen.append(point)
    return chosen


def _path_length(route: Sequence[Point], start: Point) -> float:
    total, current = 0.0, start
    for point in route:
        total += math.dist(current, point)
        current = point
    return total


def order_waypoints(points: Sequence[Point], start: Point) -> List[Point]:
    """Nearest-neighbour tour from ``start`` followed by 2-opt clean-up."""
    remaining = list(points)
    route: List[Point] = []
    current = start
    while remaining:
        nxt = min(remaining, key=lambda p: math.dist(p, current))
        remaining.remove(nxt)
        route.append(nxt)
        current = nxt

    improved, rounds = True, 0
    while improved and rounds < 50 and len(route) > 3:
        improved = False
        rounds += 1
        for i in range(len(route) - 1):
            for j in range(i + 1, len(route)):
                candidate = route[:i] + route[i:j + 1][::-1] + route[j + 1:]
                if (_path_length(candidate, start)
                        < _path_length(route, start) - 1e-6):
                    route = candidate
                    improved = True
    return route


def add_headings(route: Sequence[Point], start: Point) -> List[Waypoint]:
    """Face each goal along its direction of travel (no extra turn-in-place)."""
    result: List[Waypoint] = []
    previous = start
    last_yaw = 0.0
    for point in route:
        if math.dist(previous, point) > 1e-3:
            last_yaw = math.atan2(point[1] - previous[1],
                                  point[0] - previous[0])
        result.append((point[0], point[1], last_yaw))
        previous = point
    return result


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation about Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def estimate_position(
    samples: Sequence[Point], outlier_m: float = 0.15
) -> Optional[Tuple[float, float, int]]:
    """Robust (x, y) estimate: median, drop outliers, then mean of inliers."""
    if not samples:
        return None
    xs = np.array([s[0] for s in samples], dtype=float)
    ys = np.array([s[1] for s in samples], dtype=float)
    med = np.array([np.median(xs), np.median(ys)])
    dist = np.hypot(xs - med[0], ys - med[1])
    keep = dist <= outlier_m
    if not keep.any():
        return float(med[0]), float(med[1]), 1
    return float(xs[keep].mean()), float(ys[keep].mean()), int(keep.sum())
