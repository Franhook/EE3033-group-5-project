import math

import numpy as np

from balance_mission.search_planner import (
    add_headings,
    estimate_position,
    free_space_distance,
    order_waypoints,
    plan_search_waypoints,
)

RES = 0.05


def make_sandbox():
    """Fenced area with one box obstacle and an unmapped border."""
    h, w = 60, 80
    grid = np.full((h, w), -1, dtype=np.int16)  # unknown everywhere
    grid[5:55, 5:75] = 0                        # mapped free space
    grid[5, 5:75] = 100                         # fence
    grid[54, 5:75] = 100
    grid[5:55, 5] = 100
    grid[5:55, 74] = 100
    grid[25:35, 35:45] = 100                    # box in the middle
    return grid, w, h


def test_waypoints_are_safe_and_inside_mapped_area():
    grid, w, h = make_sandbox()
    origin = (-1.0, -1.0)
    start = (origin[0] + 10 * RES, origin[1] + 10 * RES)
    pts = plan_search_waypoints(
        grid.flatten().tolist(), w, h, RES, origin, start,
        spacing=0.8, clearance=0.22,
    )
    assert len(pts) >= 4

    dist, _ = free_space_distance(grid.flatten().tolist(), w, h, RES)
    for x, y in pts:
        col = int((x - origin[0]) / RES)
        row = int((y - origin[1]) / RES)
        assert dist[row, col] >= 0.22 - 1e-6
        assert grid[row, col] == 0


def test_region_connected_to_start_only():
    grid, w, h = make_sandbox()
    # A second enclosed free pocket that the robot cannot reach.
    grid[40:50, 60:70] = 100
    grid[42:48, 62:68] = 0
    origin = (0.0, 0.0)
    start = (10 * RES, 10 * RES)
    pts = plan_search_waypoints(
        grid.flatten().tolist(), w, h, RES, origin, start,
        spacing=0.5, clearance=0.1,
    )
    for x, y in pts:
        col, row = int(x / RES), int(y / RES)
        assert not (40 <= row < 50 and 60 <= col < 70)


def test_order_and_headings():
    start = (0.0, 0.0)
    pts = [(2.0, 0.0), (1.0, 0.0), (3.0, 0.0), (1.0, 1.0)]
    route = order_waypoints(pts, start)
    assert set(route) == set(pts)
    assert route[0] in [(1.0, 0.0), (1.0, 1.0)]

    wps = add_headings([(1.0, 0.0), (1.0, 1.0)], start)
    assert math.isclose(wps[0][2], 0.0, abs_tol=1e-6)
    assert math.isclose(wps[1][2], math.pi / 2, abs_tol=1e-6)


def test_estimate_position_rejects_outlier():
    samples = [(1.00, 2.00), (1.01, 2.01), (0.99, 1.99), (1.02, 2.0),
               (1.6, 2.7)]  # last one is a bad detection
    x, y, n = estimate_position(samples, outlier_m=0.15)
    assert n == 4
    assert abs(x - 1.005) < 0.02 and abs(y - 2.0) < 0.02


def test_estimate_position_empty():
    assert estimate_position([]) is None
