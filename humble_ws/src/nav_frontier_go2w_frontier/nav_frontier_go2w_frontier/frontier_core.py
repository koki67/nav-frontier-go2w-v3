"""Pure frontier-detection / clustering / scoring functions. No ROS imports.

The pipeline:
    1. detect_frontier_cells — free cells with at least one unknown neighbour.
    2. cluster_frontier_cells — BFS connected components.
    3. compute_reachable_free_cells — flood fill from a robot seed cell.
    4. score_clusters — score = info_gain - lambda * travel_cost
       where  info_gain   = unknown cells inside cluster bounding box
              travel_cost = euclidean distance robot → cluster goal cell (m)
    5. select_best_cluster — argmax score, restricted to reachable clusters.

`analyze_frontiers` is the top-level convenience that runs all steps.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence


Cell = tuple[int, int]


@dataclass(frozen=True)
class GridSpec:
    """OccupancyGrid metadata needed for frontier calculations."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0
    occupied_threshold: int = 65


@dataclass(frozen=True)
class RobotPose2D:
    """Robot pose projected onto the 2D map plane."""

    x: float
    y: float


@dataclass(frozen=True)
class FrontierCluster:
    cells: tuple[Cell, ...]
    centroid_x: float
    centroid_y: float
    reachable_cells: tuple[Cell, ...]
    goal_cell: Optional[Cell]
    goal_x: Optional[float]
    goal_y: Optional[float]
    travel_cost: Optional[float]      # meters, robot → goal_cell
    info_gain: int                    # # unknown cells in cluster bounding box
    score: Optional[float]            # info_gain - lambda * travel_cost


@dataclass(frozen=True)
class FrontierSearchResult:
    frontier_cells: tuple[Cell, ...]
    clusters: tuple[FrontierCluster, ...] = field(default_factory=tuple)
    selected_cluster: Optional[FrontierCluster] = None
    robot_seed_cell: Optional[Cell] = None


def _neighbor_offsets(connectivity: int) -> tuple[Cell, ...]:
    if connectivity == 4:
        return ((1, 0), (-1, 0), (0, 1), (0, -1))
    if connectivity == 8:
        return ((1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1))
    raise ValueError("connectivity must be 4 or 8")


def _validate(map_data: Sequence[int], grid: GridSpec, connectivity: int) -> None:
    if grid.width <= 0 or grid.height <= 0:
        raise ValueError("grid dimensions must be positive")
    if grid.resolution <= 0.0:
        raise ValueError("grid resolution must be positive")
    if len(map_data) != grid.width * grid.height:
        raise ValueError("map_data length does not match grid dimensions")
    _neighbor_offsets(connectivity)


def _idx(grid: GridSpec, cell: Cell) -> int:
    return cell[1] * grid.width + cell[0]


def _in_bounds(grid: GridSpec, cell: Cell) -> bool:
    return 0 <= cell[0] < grid.width and 0 <= cell[1] < grid.height


def _value_at(map_data: Sequence[int], grid: GridSpec, cell: Cell) -> int:
    return int(map_data[_idx(grid, cell)])


def _is_free(value: int) -> bool:
    return value == 0


def _is_unknown(value: int) -> bool:
    return value == -1


def _is_traversable(value: int) -> bool:
    return _is_free(value)


def world_to_cell(grid: GridSpec, x: float, y: float) -> Optional[Cell]:
    """Convert world coordinates into a grid cell, honoring origin pose."""
    dx = x - grid.origin_x
    dy = y - grid.origin_y
    cy = math.cos(grid.origin_yaw)
    sy = math.sin(grid.origin_yaw)
    local_x = cy * dx + sy * dy
    local_y = -sy * dx + cy * dy
    cell = (
        int(math.floor(local_x / grid.resolution)),
        int(math.floor(local_y / grid.resolution)),
    )
    return cell if _in_bounds(grid, cell) else None


def cell_center(grid: GridSpec, cell: Cell) -> tuple[float, float]:
    """World coordinates of the centre of a cell."""
    local_x = (cell[0] + 0.5) * grid.resolution
    local_y = (cell[1] + 0.5) * grid.resolution
    cy = math.cos(grid.origin_yaw)
    sy = math.sin(grid.origin_yaw)
    return (
        grid.origin_x + cy * local_x - sy * local_y,
        grid.origin_y + sy * local_x + cy * local_y,
    )


def detect_frontier_cells(
    map_data: Sequence[int], grid: GridSpec, connectivity: int = 8,
) -> set[Cell]:
    """Return the set of free cells adjacent to at least one unknown cell."""
    _validate(map_data, grid, connectivity)
    offsets = _neighbor_offsets(connectivity)
    frontier: set[Cell] = set()
    for y in range(grid.height):
        for x in range(grid.width):
            cell = (x, y)
            if not _is_free(_value_at(map_data, grid, cell)):
                continue
            for dx, dy in offsets:
                neighbour = (x + dx, y + dy)
                if not _in_bounds(grid, neighbour):
                    continue
                if _is_unknown(_value_at(map_data, grid, neighbour)):
                    frontier.add(cell)
                    break
    return frontier


def cluster_frontier_cells(
    frontier_cells: Iterable[Cell], connectivity: int = 8,
) -> list[list[Cell]]:
    """BFS connected components over the frontier cell set."""
    offsets = _neighbor_offsets(connectivity)
    unvisited = set(frontier_cells)
    clusters: list[list[Cell]] = []
    while unvisited:
        seed = unvisited.pop()
        queue: deque[Cell] = deque([seed])
        cluster = [seed]
        while queue:
            cell = queue.popleft()
            for dx, dy in offsets:
                neighbour = (cell[0] + dx, cell[1] + dy)
                if neighbour in unvisited:
                    unvisited.remove(neighbour)
                    queue.append(neighbour)
                    cluster.append(neighbour)
        clusters.append(sorted(cluster))
    return clusters


def find_robot_seed_cell(
    map_data: Sequence[int],
    grid: GridSpec,
    robot_pose: RobotPose2D,
    max_radius_cells: int = 3,
) -> Optional[Cell]:
    """Pick a free cell at or near the robot pose for reachability flood-fill."""
    start = world_to_cell(grid, robot_pose.x, robot_pose.y)
    if start is None:
        return None
    if _is_traversable(_value_at(map_data, grid, start)):
        return start

    best_cell: Optional[Cell] = None
    best_dist_sq: Optional[int] = None
    sx, sy = start
    for radius in range(1, max_radius_cells + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                cell = (sx + dx, sy + dy)
                if not _in_bounds(grid, cell):
                    continue
                if not _is_traversable(_value_at(map_data, grid, cell)):
                    continue
                dist_sq = dx * dx + dy * dy
                if best_dist_sq is None or dist_sq < best_dist_sq:
                    best_cell = cell
                    best_dist_sq = dist_sq
        if best_cell is not None:
            return best_cell
    return None


def compute_reachable_free_cells(
    map_data: Sequence[int], grid: GridSpec, seed_cell: Cell, connectivity: int = 8,
) -> set[Cell]:
    """Flood fill the free cells reachable from the seed, treating unknown / occupied as walls."""
    _validate(map_data, grid, connectivity)
    if not _in_bounds(grid, seed_cell):
        return set()
    if not _is_traversable(_value_at(map_data, grid, seed_cell)):
        return set()
    offsets = _neighbor_offsets(connectivity)
    reachable: set[Cell] = {seed_cell}
    queue: deque[Cell] = deque([seed_cell])
    while queue:
        cell = queue.popleft()
        for dx, dy in offsets:
            neighbour = (cell[0] + dx, cell[1] + dy)
            if neighbour in reachable:
                continue
            if not _in_bounds(grid, neighbour):
                continue
            if not _is_traversable(_value_at(map_data, grid, neighbour)):
                continue
            reachable.add(neighbour)
            queue.append(neighbour)
    return reachable


def _cluster_centroid(grid: GridSpec, cells: Sequence[Cell]) -> tuple[float, float]:
    sx = 0.0
    sy = 0.0
    for cell in cells:
        x, y = cell_center(grid, cell)
        sx += x
        sy += y
    n = max(len(cells), 1)
    return sx / n, sy / n


def _bounding_box(cells: Sequence[Cell]) -> tuple[int, int, int, int]:
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return min(xs), min(ys), max(xs), max(ys)


def compute_info_gain(
    map_data: Sequence[int], grid: GridSpec, cluster_cells: Sequence[Cell],
) -> int:
    """Number of unknown cells inside the cluster's axis-aligned bounding box.

    This is a cheap proxy for "how much new map will I learn if I go there?" —
    a tighter, more correct alternative (raycasting from the goal) is left for
    later iterations.
    """
    if not cluster_cells:
        return 0
    x_min, y_min, x_max, y_max = _bounding_box(cluster_cells)
    count = 0
    for y in range(y_min, y_max + 1):
        for x in range(x_min, x_max + 1):
            if _is_unknown(_value_at(map_data, grid, (x, y))):
                count += 1
    return count


def _goal_for_reachable_cluster(
    grid: GridSpec,
    reachable_cells: Sequence[Cell],
    robot_pose: RobotPose2D,
) -> tuple[Cell, float, float, float]:
    """Pick the cluster member nearest to its centroid as the goal, plus its travel cost."""
    cx, cy = _cluster_centroid(grid, reachable_cells)
    goal_cell = min(
        reachable_cells,
        key=lambda cell: math.hypot(*[cell_center(grid, cell)[i] - (cx if i == 0 else cy) for i in (0, 1)]),
    )
    gx, gy = cell_center(grid, goal_cell)
    travel_cost = math.hypot(gx - robot_pose.x, gy - robot_pose.y)
    return goal_cell, gx, gy, travel_cost


def score_cluster(info_gain: int, travel_cost: float, score_lambda: float) -> float:
    """score = info_gain - lambda * travel_cost.

    info_gain is in cells (count); travel_cost is in metres. Pick lambda so the
    two terms are comparable in your environment — a sensible default is 0.5
    when typical info_gain ~ tens-to-hundreds and travel_cost ~ single-digit metres.
    """
    return float(info_gain) - score_lambda * float(travel_cost)


def analyze_frontiers(
    map_data: Sequence[int],
    grid: GridSpec,
    robot_pose: Optional[RobotPose2D],
    connectivity: int = 8,
    min_cluster_size: int = 8,
    robot_seed_search_radius: int = 3,
    score_lambda: float = 0.5,
) -> FrontierSearchResult:
    """Run the full frontier pipeline and return all intermediates + selected cluster."""
    _validate(map_data, grid, connectivity)
    if min_cluster_size < 1:
        raise ValueError("min_cluster_size must be at least 1")

    frontier_cells = detect_frontier_cells(map_data, grid, connectivity)
    raw_clusters = cluster_frontier_cells(frontier_cells, connectivity)
    sized_clusters = [c for c in raw_clusters if len(c) >= min_cluster_size]

    reachable_free_cells: Optional[set[Cell]] = None
    robot_seed_cell: Optional[Cell] = None
    if robot_pose is not None:
        robot_seed_cell = find_robot_seed_cell(
            map_data, grid, robot_pose, max_radius_cells=robot_seed_search_radius,
        )
        if robot_seed_cell is not None:
            reachable_free_cells = compute_reachable_free_cells(
                map_data, grid, robot_seed_cell, connectivity,
            )

    cluster_infos: list[FrontierCluster] = []
    selected: Optional[FrontierCluster] = None
    for cluster_cells in sized_clusters:
        cx, cy = _cluster_centroid(grid, cluster_cells)
        info_gain = compute_info_gain(map_data, grid, cluster_cells)

        reachable_subset: list[Cell] = []
        goal_cell: Optional[Cell] = None
        gx: Optional[float] = None
        gy: Optional[float] = None
        travel_cost: Optional[float] = None
        score: Optional[float] = None

        if reachable_free_cells is not None and robot_pose is not None:
            reachable_subset = [c for c in cluster_cells if c in reachable_free_cells]
            if reachable_subset:
                goal_cell, gx, gy, travel_cost = _goal_for_reachable_cluster(
                    grid, reachable_subset, robot_pose,
                )
                score = score_cluster(info_gain, travel_cost, score_lambda)

        cluster_info = FrontierCluster(
            cells=tuple(cluster_cells),
            centroid_x=cx,
            centroid_y=cy,
            reachable_cells=tuple(sorted(reachable_subset)),
            goal_cell=goal_cell,
            goal_x=gx,
            goal_y=gy,
            travel_cost=travel_cost,
            info_gain=info_gain,
            score=score,
        )
        cluster_infos.append(cluster_info)

        if score is None:
            continue
        if selected is None or (selected.score is not None and score > selected.score):
            selected = cluster_info

    return FrontierSearchResult(
        frontier_cells=tuple(sorted(frontier_cells)),
        clusters=tuple(cluster_infos),
        selected_cluster=selected,
        robot_seed_cell=robot_seed_cell,
    )
