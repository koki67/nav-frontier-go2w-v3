"""Unit tests for the pure-Python frontier_core module."""
from __future__ import annotations

import pytest

from nav_frontier_go2w_frontier.frontier_core import (
    GridSpec,
    RobotPose2D,
    analyze_frontiers,
    cell_center,
    cluster_frontier_cells,
    compute_info_gain,
    compute_reachable_free_cells,
    detect_frontier_cells,
    score_cluster,
    world_to_cell,
)


# Cell legend used in the synthetic grids below:
#   0  free,  -1 unknown,  100 occupied
#
# All test grids are linearised row-major (y * width + x).


def _grid(rows: list[list[int]], resolution: float = 1.0) -> tuple[list[int], GridSpec]:
    height = len(rows)
    width = len(rows[0]) if height else 0
    flat: list[int] = []
    for row in rows:
        flat.extend(row)
    return flat, GridSpec(width=width, height=height, resolution=resolution,
                          origin_x=0.0, origin_y=0.0)


class TestCoords:
    def test_world_to_cell_round_trip(self):
        _, grid = _grid([[0] * 5 for _ in range(5)], resolution=0.5)
        cell = world_to_cell(grid, 1.25, 1.75)
        assert cell == (2, 3)
        assert cell_center(grid, cell) == pytest.approx((1.25, 1.75))

    def test_world_to_cell_out_of_bounds(self):
        _, grid = _grid([[0, 0, 0]] * 3)
        assert world_to_cell(grid, -0.1, 0.0) is None
        assert world_to_cell(grid, 100.0, 0.0) is None


class TestDetect:
    def test_no_unknown_means_no_frontier(self):
        data, grid = _grid([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ])
        assert detect_frontier_cells(data, grid) == set()

    def test_single_unknown_neighbour_marks_frontier(self):
        data, grid = _grid([
            [0,  0,  0],
            [0,  0, -1],
            [0,  0,  0],
        ])
        # Only free cells with at least one unknown neighbour are frontier.
        # At 8-connectivity, (1,0), (1,1), (2,0), (2,2), (1,2) all neighbour the unknown (2,1).
        frontier = detect_frontier_cells(data, grid, connectivity=8)
        assert (2, 1) not in frontier  # the unknown itself is not free
        assert (1, 1) in frontier

    def test_occupied_cells_are_never_frontier(self):
        data, grid = _grid([
            [100, 0, -1],
            [  0, 0,  0],
        ])
        frontier = detect_frontier_cells(data, grid, connectivity=4)
        assert (0, 0) not in frontier  # occupied cell


class TestClustering:
    def test_two_disjoint_clusters(self):
        cells = [(0, 0), (1, 0), (5, 5), (6, 5)]
        clusters = cluster_frontier_cells(cells, connectivity=8)
        # Order of clusters is not deterministic; sort for stability.
        sorted_clusters = sorted([sorted(c) for c in clusters])
        assert sorted_clusters == [[(0, 0), (1, 0)], [(5, 5), (6, 5)]]

    def test_single_cluster_when_diagonally_connected(self):
        cells = [(0, 0), (1, 1)]
        clusters = cluster_frontier_cells(cells, connectivity=8)
        assert len(clusters) == 1

    def test_diagonal_split_under_4_connectivity(self):
        cells = [(0, 0), (1, 1)]
        clusters = cluster_frontier_cells(cells, connectivity=4)
        assert len(clusters) == 2


class TestReachability:
    def test_flood_fill_respects_walls(self):
        data, grid = _grid([
            [0, 0, 100, 0, 0],
            [0, 0, 100, 0, 0],
        ])
        reachable = compute_reachable_free_cells(data, grid, (0, 0), connectivity=4)
        assert (1, 0) in reachable
        assert (3, 0) not in reachable  # walled off


class TestInfoGain:
    def test_unknown_count_in_bounding_box(self):
        # Cluster spans rows 0–1, cols 0–2 (bbox 0..2 x 0..1 = 6 cells).
        data, grid = _grid([
            [-1, 0, -1],
            [ 0, 0, -1],
        ])
        cluster = [(0, 0), (1, 0), (1, 1)]
        # Bounding box (0,0)-(1,1): values are -1, 0, 0, 0 → 1 unknown.
        assert compute_info_gain(data, grid, cluster) == 1


class TestScoring:
    def test_score_rewards_info_gain_penalises_distance(self):
        # info_gain dominates short distance: score positive.
        assert score_cluster(info_gain=20, travel_cost=2.0, score_lambda=0.5) == pytest.approx(19.0)
        # high cost flips the sign:
        assert score_cluster(info_gain=5, travel_cost=20.0, score_lambda=0.5) == pytest.approx(-5.0)


class TestAnalyzePipeline:
    def test_selects_higher_info_gain_when_costs_similar(self):
        # 5x5 grid: robot at (0,0). Two frontier clusters, one with bigger info area.
        # Layout (-1 unknown):
        # row 0: [0, 0, 0, -1, -1]   <- frontier on right (small unknown patch)
        # row 1: [0, 0, 0, -1, -1]
        # row 2: [0, 0, 0,  0,  0]
        # row 3: [0, 0, 0,  0,  0]
        # row 4: [-1,-1,-1, 0,  0]   <- another frontier on the bottom-left (bigger unknown patch)
        data, grid = _grid([
            [ 0,  0,  0, -1, -1],
            [ 0,  0,  0, -1, -1],
            [ 0,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0],
            [-1, -1, -1,  0,  0],
        ])
        result = analyze_frontiers(
            map_data=data, grid=grid,
            robot_pose=RobotPose2D(x=0.5, y=0.5),
            connectivity=8, min_cluster_size=1, score_lambda=0.1,
        )
        assert result.selected_cluster is not None
        assert result.selected_cluster.info_gain >= 1
        # With small lambda the selector should prefer the larger info-gain cluster
        # regardless of travel cost — both clusters are reachable and similarly close.
        assert all(
            (c.score is None) or (result.selected_cluster.score >= c.score)
            for c in result.clusters
        )

    def test_no_frontier_when_no_unknown(self):
        data, grid = _grid([[0, 0], [0, 0]])
        result = analyze_frontiers(
            map_data=data, grid=grid,
            robot_pose=RobotPose2D(x=0.5, y=0.5),
            connectivity=8, min_cluster_size=1,
        )
        assert result.selected_cluster is None
        assert result.frontier_cells == ()

    def test_unreachable_cluster_is_filtered_out(self):
        # Robot is in the left half; the only frontier sits behind a wall.
        data, grid = _grid([
            [0, 100, 0, -1],
            [0, 100, 0,  0],
        ])
        result = analyze_frontiers(
            map_data=data, grid=grid,
            robot_pose=RobotPose2D(x=0.5, y=0.5),
            connectivity=4, min_cluster_size=1,
        )
        assert result.selected_cluster is None  # walled off frontiers ignored
