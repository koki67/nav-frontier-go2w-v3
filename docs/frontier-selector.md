# Frontier Selector Algorithm

This document describes the current frontier selector implementation in
`nav_frontier_go2w_frontier`. The selector is intentionally simple and explicit:
it works directly on the 2D occupancy grid, produces observable intermediate
markers, and keeps the core algorithm in pure Python so it can be tested without
ROS.

## Role In The Stack

The selector consumes the map produced by `slam_toolbox` on `/map` and publishes
the next exploration target on `/frontier_goal`.

It does not consume Nav2 costmaps. Costmaps answer "where can the robot safely
drive after inflation and obstacle processing?" The frontier selector answers a
different question: "where is the boundary between known free space and unknown
space in the current exploration map?"

That separation is deliberate:

- `/map` is the exploration state. Unknown cells are meaningful there.
- Nav2 costmaps are planning products. They include inflation and local obstacle
  processing that can hide the raw frontier boundary.
- Keeping the selector map-based makes the algorithm deterministic and easy to
  inspect in tests and RViz.

The tradeoff is that a selected frontier may be reachable in the raw map but
still hard for Nav2 to traverse after robot radius, inflation, and MPPI costs
are applied. Narrow corridors are a typical case where this distinction matters.

## Implementation Files

The core logic is split across:

| File | Responsibility |
|------|----------------|
| `humble_ws/src/nav_frontier_go2w_frontier/nav_frontier_go2w_frontier/frontier_core.py` | Pure frontier detection, clustering, reachability, scoring, and selection. |
| `humble_ws/src/nav_frontier_go2w_frontier/nav_frontier_go2w_frontier/frontier_selector_node.py` | ROS node wrapper: subscriptions, TF lookup, publishing goals and markers. |
| `humble_ws/src/nav_frontier_go2w_frontier/config/frontier_selector.yaml` | Default node parameters. |
| `humble_ws/src/nav_frontier_go2w_frontier/test/test_frontier_core.py` | Unit tests for the pure algorithm. |

## Occupancy Model

The selector uses the standard `nav_msgs/OccupancyGrid` conventions:

| Value | Meaning In This Selector |
|-------|--------------------------|
| `0` | Free and traversable. |
| `-1` | Unknown. |
| Any other value | Not traversable for selector reachability. |

The current implementation is conservative: only exact `0` cells are considered
free. It does not currently use the `occupied_threshold` field in `GridSpec`.

## Pipeline

At each timer tick, the ROS node runs this pipeline:

1. Receive the latest `/map`.
2. Look up the robot pose from TF, normally `map -> base_link`.
3. Convert the occupancy grid metadata into `GridSpec`.
4. Detect all frontier cells.
5. Cluster frontier cells with BFS.
6. Filter out clusters smaller than `min_cluster_size`.
7. Find a free seed cell at or near the robot pose.
8. Flood-fill reachable free cells from that seed.
9. For each cluster, keep only cells reachable from the robot.
10. Score reachable clusters.
11. Publish the best cluster's goal as `/frontier_goal`.
12. Publish RViz markers on `/frontier_markers`.

## Frontier Detection

A frontier cell is:

```text
a free cell with at least one unknown neighbor
```

The neighbor set is controlled by `connectivity`:

| `connectivity` | Behavior |
|----------------|----------|
| `4` | Only up, down, left, right neighbors count. |
| `8` | Diagonal neighbors also count. This is the current default. |

Using 8-connectivity tends to form smoother and more continuous frontier
clusters on diagonal map boundaries. Using 4-connectivity is stricter and can
split diagonal frontiers into multiple clusters.

## Clustering

After frontier detection, the selector clusters frontier cells with BFS connected
components over the frontier cell set.

Small clusters are removed with:

```text
min_cluster_size
```

The default is `8` cells. This rejects isolated speckles caused by map noise or
single scan artifacts.

## Robot Seed And Reachability

The selector projects the robot pose into map-cell coordinates. If the robot's
current map cell is free, that cell becomes the flood-fill seed.

If the robot is not exactly on a free cell, the selector searches nearby cells
within a small fixed radius. This avoids failing immediately when the map origin,
robot pose, or occupancy discretization puts the robot just outside a free cell.

Reachability is then computed by flood-filling free cells from the seed. Unknown
and occupied cells are treated as walls.

This prevents obvious bad goals, such as frontiers behind a known wall. It is
not a full robot-footprint feasibility check. It does not account for robot
radius, inflation radius, MPPI trajectory feasibility, or dynamic obstacles.
Those checks happen later in Nav2.

## Information Gain

For each frontier cluster, the selector estimates information gain by counting
unknown cells near the cluster.

The implementation uses an expanded axis-aligned bounding box:

```text
cluster bounding box expanded by info_radius_cells
```

Then it counts `-1` cells inside that expanded box.

The default is:

```text
info_radius_cells = 2
```

This is intentionally cheap and stable. It avoids a common failure mode where a
thin one-cell frontier line has zero unknown cells inside its exact bounding
box, while still rewarding frontiers that border larger unknown areas.

Limitations:

- It is a local unknown-cell count, not a sensor-model prediction.
- It does not raycast from the robot or from a candidate viewpoint.
- It can overvalue broad unknown regions even if the eventual approach is poor.

## Goal Cell Selection

For each reachable cluster, the selector chooses a goal on the known-free side
of the frontier:

1. Compute the cluster centroid in world coordinates.
2. Consider only reachable cells inside the cluster.
3. Pick the reachable cluster cell nearest to the centroid.
4. Publish that cell center as the goal position.

This is important: the goal is not placed inside unknown space. Nav2 is asked to
drive to a free cell at the edge of exploration, where the next scan should
reveal more map.

The goal orientation is set to face from the robot's current pose toward the
selected goal. This gives Nav2 a reasonable heading target without trying to
infer the best sensor-facing direction from the frontier geometry.

## Scoring

Each reachable cluster is scored with:

```text
score = info_gain - score_lambda * travel_cost
```

Where:

| Term | Meaning |
|------|---------|
| `info_gain` | Number of unknown cells near the cluster. |
| `travel_cost` | Euclidean distance from robot pose to the selected goal cell, in meters. |
| `score_lambda` | Penalty weight applied to travel distance. |

The default is:

```text
score_lambda = 0.5
```

Larger `score_lambda` favors nearer frontiers. Smaller `score_lambda` favors
larger unknown regions even when they are farther away.

The current travel cost is Euclidean distance, not planned path length. This is
a deliberate first implementation choice: it is fast, simple, and independent
of Nav2 planner state. The tradeoff is that it underestimates frontiers that are
geometrically close but require a long route around obstacles.

## Publishing

The node publishes:

| Topic | Type | Purpose |
|-------|------|---------|
| `/frontier_goal` | `geometry_msgs/PoseStamped` | Selected target sent to the goal executor. |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | RViz visualization of all frontiers, selected cluster, and goal. |

Marker convention:

| Marker | Meaning |
|--------|---------|
| Yellow points | All detected frontier cells. |
| Green points | Selected frontier cluster. |
| Red sphere | Published goal position. |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_topic` | `/map` | OccupancyGrid source. |
| `global_frame` | `map` | Frame used for goals and TF lookup. |
| `robot_base_frame` | `base_link` | Robot frame used for current pose lookup. |
| `goal_topic` | `/frontier_goal` | Published goal topic. |
| `markers_topic` | `/frontier_markers` | Published RViz marker topic. |
| `update_rate` | `2.0` | Selector frequency in Hz. |
| `connectivity` | `8` | Frontier neighbor connectivity, either `4` or `8`. |
| `min_cluster_size` | `8` | Minimum frontier cluster size in cells. |
| `score_lambda` | `0.5` | Distance penalty in `info_gain - lambda * travel_cost`. |
| `info_radius_cells` | `2` | Bounding-box expansion used for unknown-cell counting. |
| `publish_markers` | `true` | Enables `/frontier_markers`. |
| `tf_timeout` | `0.2` | TF lookup timeout in seconds. |

Only `score_lambda` and `info_radius_cells` are currently exposed directly by
the top-level launch file. Other selector parameters are controlled through
`frontier_selector.yaml`.

## Why This Architecture

The current architecture was chosen for four reasons:

- Testability: `frontier_core.py` has no ROS imports, so frontier behavior can
  be unit-tested with small synthetic grids.
- Inspectability: the score formula and marker output make it easy to understand
  why a goal was selected.
- Safety of responsibility: the selector proposes exploration targets, while
  Nav2 remains responsible for planning and collision-aware control.
- Low runtime cost: the algorithm is simple enough to run at a few Hz on the
  robot computer without depending on planner calls for every candidate.

This is a good baseline for field development, but it is not the final form for
hard environments.

## Known Limitations

The main limitations are:

- Reachability uses raw map free cells only, not robot footprint clearance.
- Travel cost is straight-line distance, not Nav2 path cost.
- Information gain is a bounding-box unknown-cell count, not expected sensor
  observation gain.
- The selector does not blacklist frontiers that Nav2 repeatedly fails to reach.
- The selected goal is a frontier cell, not a viewpoint offset behind the
  frontier with an optimized sensor heading.
- The selector does not use local obstacle data directly.

For narrow corridors, a frontier can be valid in `/map` but still rejected or
avoided by Nav2 because the costmap inflation and MPPI collision costs make the
corridor effectively too narrow.

## Development Directions

The most useful next improvements are likely:

- Costmap-aware frontier filtering: reject candidate goals without enough
  clearance in the global costmap.
- Path-cost scoring: call the global planner or use costmap distance fields
  instead of Euclidean travel cost.
- Viewpoint generation: place goals slightly inside known free space, facing the
  unknown region, instead of using the centroid-nearest frontier cell directly.
- Failure memory: temporarily suppress frontiers that Nav2 aborts repeatedly.
- Stability filtering: avoid rapidly switching between similar adjacent
  frontiers.
- Better information gain: raycast expected sensor visibility from the candidate
  viewpoint rather than counting unknown cells in a bounding box.

